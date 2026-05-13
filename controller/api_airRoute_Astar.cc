#include "api_airRoute_Astar.h"
#include "GridEvaluator.h"
#include <drogon/drogon.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/DQG3DProximity.h>
#include <dqg/GlobalBaseTile.h>
#include "LineToGrids.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <string>
#include <array>
#include <cmath>
#include <ctime>
#include <utility>
#include <memory>
#include <cstdlib>

using namespace drogon;
using namespace std;

namespace api {
namespace airRoute {

// === 全局配置变量 ===
int g_maxSearchSteps = 100000;

// === 配置初始化函数实现 ===
void initializeAstarConfig() {
    try {
        const Json::Value& customConfig = drogon::app().getCustomConfig();
        if (customConfig.isMember("max_search_steps")) {
            g_maxSearchSteps = customConfig["max_search_steps"].asInt();
            LOG_INFO << "A* 搜索步数上限已从配置文件加载: " << g_maxSearchSteps;
        } else {
            LOG_WARN << "配置文件中未找到 max_search_steps，使用默认值: " << g_maxSearchSteps;
        }
    } catch (const std::exception& e) {
        LOG_ERROR << "加载 A* 配置失败: " << e.what() << "，使用默认值: " << g_maxSearchSteps;
    }
}

// === 辅助函数与结构 ===

// 牛顿迭代法求平方根，用于计算欧几里得距离
static double newton(double num, int iters = 5) {
    if (num <= 0) return 0.0;
    double x = num / 2.0;
    for (int i = 0; i < iters; ++i) x = 0.5 * (x + num / x);
    return x;
}

// 根据层级获取网格大小（单位：米）
double getGridSize(int level) {
    if (level < 1 || level > 21) {
        throw std::invalid_argument("Unsupported level: " + to_string(level));
    }
    return 78125.0 / std::pow(2.0, level);
}

// A*算法搜索方向定义，共26个方向（3D空间的26连通性）
const vector<array<int, 3>> DIRECTIONS = {
    {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
    {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
    {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
    {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
    {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
    {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
};

// 各方向对应的距离（相对于网格边长的倍数）
const vector<double> DIRECTION_DISTANCES = {
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.4142135623730951, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951,
    1.4142135623730951, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951,
    1.4142135623730951, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951,
    1.7320508075688772, 1.7320508075688772, 1.7320508075688772, 1.7320508075688772,
    1.7320508075688772, 1.7320508075688772, 1.7320508075688772, 1.7320508075688772
};

// 规范化时间结构，用于时间约束检查
struct NormalizedTime {
    int wdTime;
    string wdRule;
};

// 获取北京时间（UTC+8）的当前时间戳（秒）
int getBeijingTime() {
    return static_cast<int>(time(nullptr)) + 8 * 3600;
}

// 规范化网格时间
NormalizedTime normalizeGridTime(int gridTime, int currentTime) {
    long diff = std::abs(static_cast<long>(gridTime) - static_cast<long>(currentTime));
    if (diff > 86400L) {
        const int UTC_OFFSET = 8 * 3600;
        int normalizedTime = ((gridTime + UTC_OFFSET) / 86400) * 86400 - UTC_OFFSET;
        return {normalizedTime, "wdd_11"};
    } else {
        const int UTC_OFFSET = 8 * 3600;
        int normalizedTime = ((gridTime + UTC_OFFSET) / 3600) * 3600 - UTC_OFFSET;
        return {normalizedTime, "wdh_11"};
    }
}

// 定义整型网格坐标键，用于快速比较和哈希
struct GridKey {
    int x;
    int y;
    int z;


    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator!=(const GridKey& other) const {
        return !(*this == other);
    }
};

    // 坐标键的哈希函数
    struct GridKeyHash {
        std::size_t operator()(const GridKey& k) const {
            std::size_t h = 17;
            h = h * 31 + std::hash<int>()(k.x);
            h = h * 31 + std::hash<int>()(k.y);
            h = h * 31 + std::hash<int>()(k.z);
            return h;
        }
    };

// A*算法配置选项
struct AStarOptions {
    double speed = 15.0; // 飞行器速度，单位：米/秒
};

// A*算法执行结果
struct AStarResult {
    bool success;
    vector<string> path;
    string reason;
};

// === 协程适配器 ===
struct GridCheckAwaiter {
    std::shared_ptr<GridEvaluator> evaluator;
    const std::vector<CandidateInfo>& candidates;

    std::shared_ptr<std::unordered_map<std::string, GridEvaluator::CheckResult>> result;

    bool await_ready() { return false; }

    void await_suspend(std::coroutine_handle<> h) {
        evaluator->checkCandidates(candidates, [this, h](const std::unordered_map<std::string, GridEvaluator::CheckResult>& res) mutable {
            this->result = std::make_shared<std::unordered_map<std::string, GridEvaluator::CheckResult>>(res);
            h.resume();
        });
    }

    std::shared_ptr<std::unordered_map<std::string, GridEvaluator::CheckResult>> await_resume() { return result; }
};


// === A* 核心逻辑 (简化版 - 无约束条件) ===
AStarResult aStarPathSimple(
    array<int, 3> start,
    array<int, 3> end,
    const AStarOptions& options,
    int level,
    int workLayer
) {
    double gridSize;
    try {
        gridSize = getGridSize(level);
    } catch (const exception& e) {
        return {false, {}, string("不支持的 level: ") + to_string(level)};
    }

    int sx = start[0], sy = start[1], sz = start[2];
    int ex = end[0], ey = end[1], ez = end[2];

    if (sx < 0 || sy < 0 ||  ex < 0 || ey < 0 ) {
        return {false, {}, "行列坐标不能为负数"};
    }
    double dx2 = sx - ex, dy2 = sy - ey, dz2 = sz - ez;
    double lineLength = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
    // 定义 A* 算法的启发式函数 (使用原生 std::sqrt 并加入 Tie-Breaker)
    auto heuristic = [&](int x, int y, int z) {
        // 1. 基础物理距离
        double dx = x - ex, dy = y - ey, dz = z - ez;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize;


        // 如果起终点重合，直接返回
        if (lineLength < 1e-6) return dist;
        // 3. 叉乘计算
        double crossX = dy * dz2 - dz * dy2;
        double crossY = dz * dx2 - dx * dz2;
        double crossZ = dx * dy2 - dy * dx2;

        // 使用 std::sqrt 获取精确的叉乘面积和底边长
        double crossMag = std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);



        // 4. 计算垂直偏离网格数并施加 0.001 的微小惩罚
        double perpDist = crossMag / lineLength;
        return dist + (perpDist * 0.001 * gridSize);
    };

    // A*节点结构
    struct Node {
        int x, y, z;
        double g, h, f;
        GridKey key;
        size_t seq;


        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, size_t s_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), key({x_, y_, z_}), seq(s_) {}

        bool operator>(const Node& o) const {
            if (abs(f - o.f) > 1e-6) return f > o.f;
            return seq > o.seq;
        }
    };

    static size_t globalSeqSimple = 0;
    GridKey startKey = {sx, sy, sz}; // 起点无方向

    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    std::unordered_map<GridKey, Node, GridKeyHash> openMap;
    std::unordered_set<GridKey, GridKeyHash> closedSet;
    std::unordered_map<GridKey, GridKey, GridKeyHash> parent;

    double h0 = heuristic(sx, sy, sz);
    Node startNode(sx, sy, sz, 0.0, h0, globalSeqSimple++);
    openSet.push(startNode);
    openMap[startKey] = startNode;

    int searchSteps = 0;
    const int MAX_SEARCH_STEPS = g_maxSearchSteps;
    uint64_t maxCoord = (1ULL << (3 * level));

    while (!openSet.empty()) {
        if (++searchSteps > MAX_SEARCH_STEPS) {
            return {false, {}, "路径计算超时: 搜索范围过大"};
        }

        Node cur = openSet.top();
        openSet.pop();

        if (closedSet.count(cur.key)) continue;

        auto it = openMap.find(cur.key);
        if (it == openMap.end() || abs(it->second.g - cur.g) > 1e-6) continue;

        // 到达终点判定：通过坐标对比，不对比方向
        if (cur.x == ex && cur.y == ey && cur.z == ez) {
            vector<string> path;
            GridKey currKey = cur.key;

            IJH lastIJH = {(uint32_t)currKey.y, (uint32_t)currKey.x, (uint32_t)currKey.z};
            path.push_back(rchToCode(lastIJH, static_cast<uint8_t>(level)));

            while (parent.count(currKey)) {
                currKey = parent[currKey];
                IJH ijh = {(uint32_t)currKey.y, (uint32_t)currKey.x, (uint32_t)currKey.z};
                path.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
            }
            reverse(path.begin(), path.end());
            return {true, path, ""};
        }

        closedSet.insert(cur.key);
        openMap.erase(cur.key);

        // 扩展 26 个邻居
        for (size_t i = 0; i < DIRECTIONS.size(); ++i) {
            const auto& d = DIRECTIONS[i];
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            int nz = cur.z + d[2];

            if (nx < 0 || ny < 0 || nz < 0) continue;
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;

            GridKey nKey = {nx, ny, nz};
            if (closedSet.count(nKey)) continue;


            double moveDist = DIRECTION_DISTANCES[i] * gridSize;



            double newG = cur.g + moveDist ;

            auto existing = openMap.find(nKey);
            if (existing != openMap.end() && newG >= existing->second.g) continue;

            double newH = heuristic(nx, ny, nz);
            Node next(nx, ny, nz, newG, newH, globalSeqSimple++);
            openSet.push(next);
            openMap[nKey] = next;
            parent[nKey] = cur.key;
        }
    }

    return {false, {}, "未找到路径"};
}

// === A* 核心逻辑 (协程版 - 有约束条件) ===
Task<AStarResult> aStarPath(
    array<int, 3> start,
    array<int, 3> end,
    int startTime,
    double planeRadius,
    const AStarOptions& options,
    int level,
    std::shared_ptr<GridEvaluator> evaluator,
    int workLayer,
    RouteMode routeMode
) {
    int currentTime = getBeijingTime();

    double gridSize;
    try {
        gridSize = getGridSize(level);
    } catch (const exception& e) {
        co_return {false, {}, string("不支持的 level: ") + to_string(level)};
    }

    int sx = start[0], sy = start[1], sz = start[2];
    int ex = end[0], ey = end[1], ez = end[2];

    if (sx < 0 || sy < 0 ||  ex < 0 || ey < 0 ) {
        co_return {false, {}, "行列坐标不能为负数"};
    }
    RouteWeights weights = getWeightsByMode(routeMode);

    IJH startIJH = {(uint32_t)sy, (uint32_t)sx, (uint32_t)sz};
    std::string startCode = rchToCode(startIJH, static_cast<uint8_t>(level));
    IJH endIJH = {(uint32_t)ey, (uint32_t)ex, (uint32_t)ez};
    std::string endCode = rchToCode(endIJH, static_cast<uint8_t>(level));

    // ==========================================
    // 1. 起点/终点 基础有效性检查
    // ==========================================
    {
        auto startNorm = normalizeGridTime(startTime, currentTime);
        CandidateInfo startCand = { startCode, startTime, startNorm.wdTime, startNorm.wdRule, true };
        CandidateInfo endCand = { endCode, 0, 0, "", false };

        std::vector<CandidateInfo> preCheckCands = {startCand, endCand};
        auto preCheckResultsPtr = co_await GridCheckAwaiter{evaluator, preCheckCands};

        if (preCheckResultsPtr->count(startCode)) {
            const auto& res = preCheckResultsPtr->at(startCode);
            if (!res.pass) {
                co_return {false, {}, "起点不可通行: " + res.reason};
            }
        }

        if (preCheckResultsPtr->count(endCode)) {
            const auto& res = preCheckResultsPtr->at(endCode);
            if (!res.pass) {
                co_return {false, {}, "终点不可通行: " + res.reason};
            }
        }
    }

    // ==========================================
    // 2. A* 主循环 (引入运动学约束与终点特权)
    // ==========================================
    double dx2 = sx - ex, dy2 = sy - ey, dz2 = sz - ez;
    double lineLength = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
    auto heuristic = [&](int x, int y, int z) {
        double dx = x - ex, dy = y - ey, dz = z - ez;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize;

        if (lineLength < 1e-6) return dist;

        double crossX = dy * dz2 - dz * dy2;
        double crossY = dz * dx2 - dx * dz2;
        double crossZ = dx * dy2 - dy * dx2;

        double crossMag = std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);




        double perpDist = crossMag / lineLength;
        return dist + (perpDist * 0.001 * gridSize);
    };

    struct Node {
        int x, y, z;
        double g, h, f;
        int arrivalTime;
        GridKey key;
        size_t seq;


        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, int at_, size_t s_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), arrivalTime(at_), key({x_, y_, z_}), seq(s_) {}

        bool operator>(const Node& o) const {
            if (abs(f - o.f) > 1e-6) return f > o.f;
            return seq > o.seq;
        }
    };

    static size_t globalSeq = 0;
    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    std::unordered_map<GridKey, Node, GridKeyHash> openMap;
    std::unordered_set<GridKey, GridKeyHash> closedSet;
    std::unordered_map<GridKey, GridKey, GridKeyHash> parent;

    // 起点初始化：无方向(-1)，步数 0
    GridKey startKey = {sx, sy, sz};
    double h0 = heuristic(sx, sy, sz) * weights.distance;
    Node startNode(sx, sy, sz, 0.0, h0, startTime, globalSeq++);
    openSet.push(startNode);
    openMap[startKey] = startNode;

    string lastFailReason = "no_path_found";
    int searchSteps = 0;
    const int MAX_SEARCH_STEPS = g_maxSearchSteps;

    while (!openSet.empty()) {
        if (++searchSteps > MAX_SEARCH_STEPS) {
            co_return {false, {}, "路径计算超时: 搜索范围过大或目标不可达 (" + lastFailReason + ")"};
        }

        Node cur = openSet.top();
        openSet.pop();

        if (closedSet.count(cur.key)) continue;

        auto it = openMap.find(cur.key);
        if (it == openMap.end() || abs(it->second.g - cur.g) > 1e-6) continue;

        // 到达终点判定：通过坐标严格判定，无视到达方向
        if (cur.x == ex && cur.y == ey && cur.z == ez) {
            vector<string> path;
            GridKey currKey = cur.key;

            IJH lastIJH = {(uint32_t)currKey.y, (uint32_t)currKey.x, (uint32_t)currKey.z};
            path.push_back(rchToCode(lastIJH, static_cast<uint8_t>(level)));

            while (parent.count(currKey)) {
                currKey = parent[currKey];
                IJH ijh = {(uint32_t)currKey.y, (uint32_t)currKey.x, (uint32_t)currKey.z};
                path.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
            }
            reverse(path.begin(), path.end());
            co_return {true, path, ""};
        }

        closedSet.insert(cur.key);
        openMap.erase(cur.key);

        struct NeighborMeta { int x, y, z; string code; double moveCost; int arrival; };
        vector<NeighborMeta> validNeighbors;
        vector<CandidateInfo> candidateListForChecker;
        validNeighbors.reserve(26);
        candidateListForChecker.reserve(26);

        uint64_t maxCoord = (1ULL << (3 * level));

        // 遍历 26 个方向
        for (size_t i = 0; i < DIRECTIONS.size(); ++i) {
            const auto& d = DIRECTIONS[i];
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            int nz = cur.z + d[2];

            if (nx < 0 || ny < 0 || nz < 0) continue;
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;

            GridKey nKey = {nx, ny, nz};
            if (closedSet.count(nKey)) continue;

            double moveDist = DIRECTION_DISTANCES[i] * gridSize;
            int stepTime = static_cast<int>(moveDist / options.speed);
            int arrival = cur.arrivalTime + stepTime;

            auto norm = normalizeGridTime(arrival, currentTime);
            IJH nextIJH = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
            string code = rchToCode(nextIJH, static_cast<uint8_t>(level));


            validNeighbors.push_back({nx, ny, nz, code, moveDist, arrival});
            candidateListForChecker.push_back({code, arrival, norm.wdTime, norm.wdRule, true});
        }

        std::shared_ptr<std::unordered_map<string, GridEvaluator::CheckResult>> checkResultsPtr;
        if (!candidateListForChecker.empty()) {
            checkResultsPtr = co_await GridCheckAwaiter{evaluator, candidateListForChecker};
        }

        // 邻居缓冲区检查：如果任何一个生成的 26 邻居网格被阻挡，抛弃整个当前节点扩展
        bool allNeighborsPassable = true;
        bool skipNeighborBufferCheck = (cur.key == startKey);
        if (!skipNeighborBufferCheck) {
            for (const auto& nb : validNeighbors) {
                if (checkResultsPtr && checkResultsPtr->count(nb.code)) {
                    const auto& res = checkResultsPtr->at(nb.code);
                    if (!res.pass) {
                        allNeighborsPassable = false;
                        lastFailReason = "缓冲区检查失败: 邻居网格 " + nb.code + " " + res.reason;
                        break;
                    }
                } else {
                    allNeighborsPassable = false;
                    lastFailReason = "缓冲区检查失败: 邻居网格 " + nb.code + " 无检查结果";
                    break;
                }
            }
        }

        if (!allNeighborsPassable) {
            continue;
        }

        // ==========================================
        // 计算邻居节点的累积代价值 (融合转向惩罚)
        // ==========================================
        for (const auto& nb : validNeighbors) {
            const auto& res = checkResultsPtr->at(nb.code);

            double distanceCost = nb.moveCost;
            double altitudeChange = std::abs(nb.z - cur.z) * gridSize;
            double efficiencyCost = (altitudeChange > 0) ? 1.0 : 0.0;

            double baseG = weights.distance * distanceCost;
            double effPenalty = weights.efficiency * efficiencyCost;

            double safetyPenalty =
                (weights.comm  * res.commCost) +
                (weights.nav   * res.navCost)  +
                (weights.surv  * res.survCost) +
                (weights.wind  * res.windCost) +
                (weights.rain  * res.rainCost) +
                (weights.vis   * res.visCost)  +
                (weights.temp  * res.tempCost) +
                (weights.hum   * res.humCost)  +
                (weights.press * res.pressCost)+
                (weights.em    * res.emCost);

            double riskPenalty = weights.riskArea * res.riskCost;
            double privacyPenalty = weights.privacy * res.privacyCost;

            double totalExtraFactor = effPenalty + safetyPenalty + riskPenalty + privacyPenalty;
            double extraCost = distanceCost * totalExtraFactor;




            // 总代价值汇总
            double newG = cur.g + baseG + extraCost ;

            GridKey nKey = {nb.x, nb.y, nb.z};
            auto existing = openMap.find(nKey);
            if (existing != openMap.end() && newG >= existing->second.g) continue;

            double newH = heuristic(nb.x, nb.y, nb.z) * weights.distance;
            Node next(nb.x, nb.y, nb.z, newG, newH, nb.arrival, globalSeq++);
            openSet.push(next);
            openMap[nKey] = next;
            parent[nKey] = cur.key;
        }
    }

    co_return {false, {}, lastFailReason};
}

// === 接口实现 ===
Task<void> Astar::AstarPathPlane(const drogon::HttpRequestPtr req,
                                 std::function<void (const drogon::HttpResponsePtr &)> callback)
{
    auto jsonBody = req->getJsonObject();
    if (!jsonBody) {
        Json::Value response; response["status"] = "error"; response["message"] = "请求体必须是有效的JSON格式";
        auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
        callback(resp); co_return;
    }

    try {
        if (!jsonBody->isMember("points")) {
            Json::Value response; response["status"] = "error"; response["message"] = "缺少必需参数: points";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        Json::Value pointsArr = (*jsonBody)["points"];
        if (!pointsArr.isArray() || pointsArr.size() < 2) {
            Json::Value response; response["status"] = "error"; response["message"] = "points必须是包含至少2个点的数组";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        int level = (*jsonBody).get("level", 14).asInt();
        try { getGridSize(level); }
        catch (...) {
            Json::Value response; response["status"] = "error"; response["message"] = "不支持的level";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        const BaseTile& baseTile = ::getProjectBaseTile();
        vector<array<int, 3>> waypoints;

        for (unsigned int i = 0; i < pointsArr.size(); ++i) {
            Json::Value point = pointsArr[i];
            double lon = point[0].asDouble();
            double lat = point[1].asDouble();
            double height = point[2].asDouble();

            if (lon < -180 || lon > 180 || lat < -90 || lat > 90 || height < baseTile.bottom) {
                 Json::Value response; response["status"] = "error"; response["message"] = "坐标值不合法";
                 auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
                 callback(resp); co_return;
            }

            IJH ijh = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, height, baseTile);
            int layer = static_cast<int32_t>(ijh.layer);
            waypoints.push_back({static_cast<int>(ijh.column), static_cast<int>(ijh.row), layer});
        }

        long long rawStartTime = (*jsonBody).get("startTime", static_cast<Json::Int64>(getBeijingTime())).asInt64();
        int startTime = (rawStartTime > 9999999999LL) ? static_cast<int>(rawStartTime / 1000) : rawStartTime;

        double planeRadius = (*jsonBody).get("planeRadius", 0.75).asDouble();
        double cruisingSpeed = (*jsonBody).get("cruisingSpeed", 15.0).asDouble();

        if (!jsonBody->isMember("workHeight")) {
            Json::Value response; response["status"] = "error"; response["message"] = "缺少必需参数: workHeight";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        double workHeight = (*jsonBody)["workHeight"].asDouble();

        vector<string> verticalPath;
        int workLayer = 0;
        if (pointsArr.size() > 0) {
            Json::Value firstPoint = pointsArr[0];
            double lon = firstPoint[0].asDouble();
            double lat = firstPoint[1].asDouble();
            double originalHeight = firstPoint[2].asDouble();

            double absoluteWorkHeight = originalHeight + workHeight;
            IJH workIJH = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, absoluteWorkHeight, baseTile);
            workLayer = static_cast<int>(workIJH.layer);

            IJH originalIJH = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, originalHeight, baseTile);
            int col = static_cast<int>(originalIJH.column);
            int row = static_cast<int>(originalIJH.row);
            int originalLayer = static_cast<int>(originalIJH.layer);

            if (originalLayer > workLayer) {
                for (int h = originalLayer; h >= workLayer; --h) {
                    IJH ijh = {(uint32_t)row, (uint32_t)col, (uint32_t)h};
                    verticalPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            } else {
                for (int h = originalLayer; h <= workLayer; ++h) {
                    IJH ijh = {(uint32_t)row, (uint32_t)col, (uint32_t)h};
                    verticalPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
        }

        vector<string> landingPath;
        if (pointsArr.size() > 1) {
            Json::Value lastPoint = pointsArr[pointsArr.size() - 1];
            double endLon = lastPoint[0].asDouble();
            double endLat = lastPoint[1].asDouble();
            double endHeight = lastPoint[2].asDouble();

            IJH endOriginalIJH = localRowColHeiNumber(static_cast<uint8_t>(level), endLon, endLat, endHeight, baseTile);
            int endCol = static_cast<int>(endOriginalIJH.column);
            int endRow = static_cast<int>(endOriginalIJH.row);
            int endOriginalLayer = static_cast<int>(endOriginalIJH.layer);

            if (workLayer > endOriginalLayer) {
                for (int h = workLayer - 1; h >= endOriginalLayer; --h) {
                    IJH ijh = {(uint32_t)endRow, (uint32_t)endCol, (uint32_t)h};
                    landingPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
            else if (workLayer < endOriginalLayer) {
                for (int h = workLayer + 1; h <= endOriginalLayer; ++h) {
                    IJH ijh = {(uint32_t)endRow, (uint32_t)endCol, (uint32_t)h};
                    landingPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
        }

        for (auto& wp : waypoints) {
            wp[2] = workLayer;
        }

        AStarOptions options;
        options.speed = cruisingSpeed;

        RouteMode currentMode = RouteMode::BALANCED;
        if (jsonBody->isMember("route_type")) {
            std::string reqMode = (*jsonBody)["route_type"].asString();
            if (reqMode == "shortest") currentMode = RouteMode::SHORTEST;
            else if (reqMode == "safest") currentMode = RouteMode::SAFEST;
        } else if (jsonBody->isMember("mode")) {
            std::string reqMode = (*jsonBody)["mode"].asString();
            if (reqMode == "shortest") currentMode = RouteMode::SHORTEST;
            else if (reqMode == "safest") currentMode = RouteMode::SAFEST;
        }

        Json::Value ruleOptions;
        if (jsonBody->isMember("options")) {
            ruleOptions = (*jsonBody)["options"];
        } else {
            ruleOptions = jsonBody->get("condition", Json::Value(Json::objectValue));
        }

        bool isUnconstrained = ruleOptions.isNull() || (ruleOptions.isObject() && ruleOptions.empty());

        vector<string> fullPath;
        vector<int> pathIndexes;
        vector<string> waypointCodes;
        bool pathSuccess = true;
        string failReason;

        std::shared_ptr<GridEvaluator> gridEvaluator = nullptr;
        if (!isUnconstrained) {
            gridEvaluator = GridEvaluator::create(ruleOptions);
        }

        int currentSegmentStartTime = startTime;

        for (size_t i = 0; i < waypoints.size() - 1 && pathSuccess; ++i) {
            AStarResult segmentResult;

            if (isUnconstrained) {
                LOG_INFO << "[A*] 航段 " << i+1 << " 使用无约束模式（简化版A*）";
                segmentResult = aStarPathSimple(waypoints[i], waypoints[i + 1], options, level, workLayer);
            } else {
                LOG_INFO << "[A*] 航段 " << i+1 << " 使用约束模式（协程版A*）";
                segmentResult = co_await aStarPath(
                    waypoints[i], waypoints[i + 1], currentSegmentStartTime, planeRadius, options, level, gridEvaluator,
                    workLayer, currentMode
                );
            }

            if (!segmentResult.success) {
                pathSuccess = false;
                failReason = segmentResult.reason;
                break;
            }



            if (!segmentResult.path.empty()) {
                double stepGridSize = getGridSize(level);
                int segmentDuration = 0;
                for(size_t j = 0; j < segmentResult.path.size() - 1; ++j) {
                    IJH p1 = getLocalTileRHC(segmentResult.path[j]);
                    IJH p2 = getLocalTileRHC(segmentResult.path[j+1]);
                    double dx = (int)p2.column - (int)p1.column;
                    double dy = (int)p2.row - (int)p1.row;
                    double dz = (int)p2.layer - (int)p1.layer;
                    // 精确计算 26方向 实际发生的欧几里得距离，累加时间
                    segmentDuration += static_cast<int>(std::sqrt(dx*dx + dy*dy + dz*dz) * stepGridSize / options.speed);
                }
                currentSegmentStartTime += segmentDuration;
            }

            size_t segmentIndex = i + 1;
            if (i == 0) {
                fullPath = segmentResult.path;
                pathIndexes.assign(segmentResult.path.size(), segmentIndex);
            } else {
                fullPath.insert(fullPath.end(), segmentResult.path.begin() + 1, segmentResult.path.end());
                pathIndexes.insert(pathIndexes.end(), segmentResult.path.size() - 1, segmentIndex);
            }

            if (i < waypoints.size() - 2 && !segmentResult.path.empty()) {
                waypointCodes.push_back(segmentResult.path.back());
            }
        }

        Json::Value response;
        if (pathSuccess) {
            Json::Value results;
            results["success"] = true;
            results["path"] = Json::Value(Json::arrayValue);
            results["reason"] = Json::Value::null;

            vector<string> finalPath;
            vector<int> finalPathIndexes;
            vector<bool> finalIsVertical;

            finalPath.insert(finalPath.end(), verticalPath.begin(), verticalPath.end());
            finalPathIndexes.insert(finalPathIndexes.end(), verticalPath.size(), 0);
            finalIsVertical.insert(finalIsVertical.end(), verticalPath.size(), true);

            if (!finalPath.empty() && !fullPath.empty()) {
                finalPath.pop_back();
                finalPathIndexes.pop_back();
                finalIsVertical.pop_back();
            }

            finalPath.insert(finalPath.end(), fullPath.begin(), fullPath.end());
            finalPathIndexes.insert(finalPathIndexes.end(), pathIndexes.begin(), pathIndexes.end());
            finalIsVertical.insert(finalIsVertical.end(), fullPath.size(), false);

            if (!landingPath.empty()) {
                int landingIndex = static_cast<int>(waypoints.size());
                finalPath.insert(finalPath.end(), landingPath.begin(), landingPath.end());
                finalPathIndexes.insert(finalPathIndexes.end(), landingPath.size(), landingIndex);
                finalIsVertical.insert(finalIsVertical.end(), landingPath.size(), true);
            }

            double exactTimeAcc = startTime;
            double currentGridSize = getGridSize(level);

            for (size_t i = 0; i < finalPath.size(); ++i) {
                const auto& code = finalPath[i];

                if (i > 0) {
                    IJH p1 = getLocalTileRHC(finalPath[i-1]);
                    IJH p2 = getLocalTileRHC(finalPath[i]);
                    double dx = (int)p2.column - (int)p1.column;
                    double dy = (int)p2.row - (int)p1.row;
                    double dz = (int)p2.layer - (int)p1.layer;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz) * currentGridSize;
                    exactTimeAcc += (dist / options.speed);
                }

                LatLonHei boundary = getLocalTileLatLon(code, baseTile);

                Json::Value gridInfo;
                Json::Value centerArray(Json::arrayValue);
                centerArray.append(boundary.longitude);
                centerArray.append(boundary.latitude);
                centerArray.append(boundary.height);
                gridInfo["center"] = centerArray;
                gridInfo["minlon"] = boundary.west;
                gridInfo["maxlon"] = boundary.east;
                gridInfo["minlat"] = boundary.south;
                gridInfo["maxlat"] = boundary.north;
                gridInfo["top"] = boundary.top;
                gridInfo["bottom"] = boundary.bottom;
                gridInfo["code"] = code;
                gridInfo["interopCode"] = toInteropLocalCode(code, static_cast<uint8_t>(level));

                gridInfo["arrivalTime"] = static_cast<int>(exactTimeAcc);
                gridInfo["pathIndex"] = finalPathIndexes[i];

                if (finalIsVertical[i]) gridInfo["isVertical"] = true;
                if (i == 0) gridInfo["isStart"] = true;
                if (i == finalPath.size() - 1) gridInfo["isEnd"] = true;
                if (std::find(waypointCodes.begin(), waypointCodes.end(), code) != waypointCodes.end()) {
                    gridInfo["isWaypoint"] = true;
                }

                results["path"].append(gridInfo);
            }
            response["results"] = results;
            callback(HttpResponse::newHttpJsonResponse(response));
        } else {
            Json::Value results;
            results["success"] = false;
            results["path"] = Json::Value(Json::arrayValue);
            results["reason"] = failReason;

            response["results"] = results;
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
        }

    } catch (const exception& e) {
        Json::Value response;
        response["status"] = "error";
        response["message"] = string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
    co_return;
}

} // namespace airRoute
} // namespace api