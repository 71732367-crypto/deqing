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
#include "TIFF.h"
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
/*
// 牛顿迭代法求平方根，用于计算欧几里得距离
static double newton(double num, int iters = 5) {
    if (num <= 0) return 0.0;
    double x = num / 2.0;
    for (int i = 0; i < iters; ++i) x = 0.5 * (x + num / x);
    return x;
}
*/

// 根据层级获取网格大小（单位：米）
    double getGridSize(int level) {
    // 动态获取全局的基础瓦片配置
    const BaseTile& baseTile = ::getProjectBaseTile();

    // 使用您更新后的公式来计算网格物理大小
    // （注意：需要确保 std::pow 的分母不为 0，当然 2.0^level 不会为 0）
    double size = baseTile.top / std::pow(2.0, level);

    // 如果对层级有合法性要求，可以保留校验（视您的业务逻辑而定）
    if (level < 0 || level > 31) {
        throw std::invalid_argument("Unsupported level: " + std::to_string(level));
    }

    return size;
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
    int workLayer,
    bool enableTrueHeightCheck
) {
    double gridSize;
    try {
        gridSize = getGridSize(level);
    } catch (const exception& e) {
        return {false, {}, string("不支持的 level: ") + to_string(level)};
    }
    // === [新增] 获取全局基础瓦片 ===
    const BaseTile& baseTile = ::getProjectBaseTile();
    int sx = start[0], sy = start[1], sz = start[2];
    int ex = end[0], ey = end[1], ez = end[2];

    if (sx < 0 || sy < 0 ||  ex < 0 || ey < 0 ) {
        return {false, {}, "行列坐标不能为负数"};
    }
    // ==========================================
    // [新增] 起点/终点 120米真高前置校验
    // ==========================================
    auto checkNodeTrueHeight = [&](int x, int y, int z, const string& pointName) -> string {
        if (!enableTrueHeightCheck) return "";
        IJH ijh = {(uint32_t)y, (uint32_t)x, (uint32_t)z};
        string code = rchToCode(ijh, static_cast<uint8_t>(level));
        LatLonHei boundary = getLocalTileLatLon(code, baseTile);
        float ground = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
        float tHeight = boundary.height - ground;

        // 注意：真高计算 (boundary.height - ground) 天然兼容负数海拔(如水下/低洼区)
        if (tHeight > 120.0f) return pointName + "超出空域限制，超出120米真高适飞空域";
        if (tHeight < 15.0f) return pointName + "低于15米安全真高，存在撞地危险";
        return "";
    };

    string startErr = checkNodeTrueHeight(sx, sy, sz, "起点");
    if (!startErr.empty()) return {false, {}, startErr};

    string endErr = checkNodeTrueHeight(ex, ey, ez, "终点");
    if (!endErr.empty()) return {false, {}, endErr};
    // ==========================================
    double dx2 = sx - ex, dy2 = sy - ey, dz2 = sz - ez;
    double lineLength = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
    // 定义 A* 算法的启发式函数 (使用原生 std::sqrt 并加入 Tie-Breaker)
    auto heuristic = [&](int x, int y, int z) {
        double dx = x - ex, dy = y - ey, dz = z - ez;
        // 使用 std::sqrt 保证精度，乘以 1.3 的权重打破平衡，防止 A* 在空旷区盲目扩散
        return std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize * 1.3;
    };
    // A*节点结构
    struct Node {
        int x, y, z;
        double g, h, f;
        GridKey key;



        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, size_t s_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), key({x_, y_, z_}) {}

        bool operator>(const Node& o) const {
            return f > o.f; // 只需要比较 f 值，性能更好
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
    string failReason = "未找到路径"; // [新增] 追踪失败原因
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
            // ==========================================
            // [新增] 简化版的物理真高安全限制
            // ==========================================
            IJH nextIJH = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
            string code = rchToCode(nextIJH, static_cast<uint8_t>(level));
            LatLonHei boundary = getLocalTileLatLon(code, baseTile);

            if (enableTrueHeightCheck) {
                float groundElevation = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
                float trueHeight = boundary.height - groundElevation;

                if (trueHeight > 120.0f) {
                    failReason = "超出空域限制，超出120米真高适飞空域";
                    continue;
                }
                if (trueHeight < 15.0f) {
                    failReason = "低于15米安全真高，存在撞地危险";
                    continue;
                }
            }
            // ==========================================

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
    RouteMode routeMode,
    bool enableTrueHeightCheck
) {
    int currentTime = getBeijingTime();
    // === [新增] 获取全局基础瓦片，用于后续的网格与经纬度转换 ===
    const BaseTile& baseTile = ::getProjectBaseTile();
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
        // [新增] 独立真高校验，防止起终点直接违规
        auto checkNodeTrueHeight = [&](const string& code, const string& pointName) -> string {
            if (!enableTrueHeightCheck) return "";
            LatLonHei boundary = getLocalTileLatLon(code, baseTile);
            float ground = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
            float tHeight = boundary.height - ground;
            if (tHeight > 120.0f) return pointName + "超出空域限制，超出120米真高适飞空域";
            if (tHeight < 15.0f) return pointName + "低于15米安全真高，存在撞地危险";
            return "";
        };

        string startHeightErr = checkNodeTrueHeight(startCode, "起点");
        if (!startHeightErr.empty()) co_return {false, {}, startHeightErr};

        string endHeightErr = checkNodeTrueHeight(endCode, "终点");
        if (!endHeightErr.empty()) co_return {false, {}, endHeightErr};
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

    // [核心修改 1]：动态启发式权重 (Weighted A*)
    // 默认 1.3 用于打破平衡；如果是 safest 模式，大幅提高权重以抵消巨大的 g(n) 惩罚

    double hWeight = 1.2;
    if (routeMode == RouteMode::SAFEST) {
        hWeight = 1.2;
    } else if (routeMode == RouteMode::BALANCED) {
        hWeight = 1.2;
    } else if (routeMode == RouteMode::SHORTEST) {
        hWeight = 1.2;
    }

    auto heuristic = [&](int x, int y, int z) {
        double dx = x - ex, dy = y - ey, dz = z - ez;
        return std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize * hWeight;
    };

    struct Node {
        int x, y, z;
        double g, h, f;
        int arrivalTime;
        GridKey key;



        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, int at_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), arrivalTime(at_), key({x_, y_, z_}) {}

        bool operator>(const Node& o) const {
            return f > o.f; // 只需要比较 f 值，性能更好
        }
    };


    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    std::unordered_map<GridKey, Node, GridKeyHash> openMap;
    std::unordered_set<GridKey, GridKeyHash> closedSet;
    std::unordered_map<GridKey, GridKey, GridKeyHash> parent;

    // 起点初始化：无方向(-1)，步数 0
    GridKey startKey = {sx, sy, sz};
    double h0 = heuristic(sx, sy, sz) * weights.distance;
    Node startNode(sx, sy, sz, 0.0, h0, startTime);
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

            // ==========================================
            // [新增] 120米适飞区与防撞地真高校验
            // ==========================================

            // 1. 将邻居网格编码转换为实际的经纬度和绝对高度
            LatLonHei boundary = getLocalTileLatLon(code, baseTile);

            if (enableTrueHeightCheck) {
                // 2. 从 TiffReader 获取此经纬度下的真实地面高程
                float groundElevation = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);

                // 3. 计算相对高差(真高)。
                // 算法天然兼容负数高程（如水下测绘或低洼盆地），
                // 假设无人机网格绝对高度 20m，地面海拔 -50m，真高为 20 - (-50) = 70m。
                float trueHeight = boundary.height - groundElevation;
                // 4. 适飞区判定限制：最高 120 米，最低安全距离 5 米
                float maxFlyableTrueHeight = 120.0f;
                float minSafeTrueHeight = 15.0f;

                if (trueHeight > maxFlyableTrueHeight) {
                    lastFailReason = "路径受阻: 前方超出空域限制，超出120米真高适飞空域";
                    continue;
                }
                if (trueHeight < minSafeTrueHeight) {
                    lastFailReason = "路径受阻: 前方低于15米安全真高，存在撞地危险";
                    continue;
                }
            }
            // ==========================================
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
            Node next(nb.x, nb.y, nb.z, newG, newH, nb.arrival);
            openSet.push(next);
            openMap[nKey] = next;
            parent[nKey] = cur.key;
        }
    }

    co_return {false, {}, lastFailReason};
}
//----------------------------抽稀函数--------------------------------------
    Task<vector<string>> thinPathGreedy(
        const vector<string>& originalPath, //存取抽稀前路径
        std::shared_ptr<GridEvaluator> evaluator, // 修复：统一变量名为 evaluator
        int startTime,
        uint8_t level,
        bool enableTrueHeightCheck
    )
    {
        if (originalPath.size() <= 2) co_return originalPath;
        vector<string> smoothPath; //存储平滑后的结果
        smoothPath.push_back(originalPath[0]); //将起点存入平滑路径
        size_t currentIndex = 0; //当前节点
        size_t targetIndex = 2; //相隔一个网格的节点
        const BaseTile& baseTile = ::getProjectBaseTile(); // 获取基准瓦片范围，用于坐标转换
        uint64_t maxCoord = (1ull << (3 * level)); //用于边界检测
        int currentTime = getBeijingTime(); //用于时间规则统一

        while (targetIndex < originalPath.size())
        {
            //获取物理坐标并调用DDA射线算法
            LatLonHei p1 = getLocalTileLatLon(originalPath[currentIndex], baseTile); //获取当前点坐标
            LatLonHei p2 = getLocalTileLatLon(originalPath[targetIndex], baseTile); //获取目标点的坐标
            std::vector<std::array<double, 3>> lineReq{  //记录p1和p2的经纬高
                {p1.longitude, p1.latitude, p1.height},
                {p2.longitude, p2.latitude, p2.height}
            };
            std::vector<std::string> lineGrids = singleLineToGrids2(lineReq, level, baseTile); //拉取直线
            //26个方向膨胀建立缓冲区
            std::unordered_set<std::string> expandedGridSet;
            for (const auto& code : lineGrids)
            {
                expandedGridSet.insert(code); //加入中心网格
                IJH centerIJH = getLocalTileRHC(code);
                int cx = centerIJH.column;
                int cy = centerIJH.row;
                int cz = centerIJH.layer;
                //扩展26个邻居保持一格宽缓冲区
                for (const auto& d : DIRECTIONS)
                {
                    int nx = cx + d[0];
                    int ny = cy + d[1];
                    int nz = cz + d[2];
                    // 边界保护：兼容负高度和坐标越界
                    if (nx < 0 || ny < 0 || nz < 0) continue;
                    if (static_cast<uint64_t>(nx) >= maxCoord || static_cast<uint64_t>(ny) >= maxCoord || static_cast<uint64_t>(nz) >= maxCoord) continue;
                    IJH nIjh = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
                    expandedGridSet.insert(rchToCode(nIjh, level));
                }
            }

            // ================= 修复核心：循环分离 =================
            // 1. 先把所有要校验的网格塞进数组
            vector<CandidateInfo> checkCands;
            auto norm = normalizeGridTime(startTime, currentTime);
            for (const auto& code : expandedGridSet)
            {
                checkCands.push_back({code, startTime, norm.wdTime, norm.wdRule, true});
            }
            // 注意：装填数据的 for 循环在这里结束了！

            // 2. 挂起协程，统一等待 Redis 批量校验完成
            auto checkResultsPtr = co_await GridCheckAwaiter{evaluator, checkCands};
            // 3. 双指针判定与滑动机制
            bool isLineSafe = true;
            for (const auto& code : expandedGridSet) {
                // 【新增】真高安全校验：拉直的视线绝不能越过 120m 适飞区或 15m 撞地红线
                if (enableTrueHeightCheck) {
                    LatLonHei boundary = getLocalTileLatLon(code, baseTile);
                    float ground = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
                    float tHeight = boundary.height - ground;

                    if (tHeight > 120.0f || tHeight < 15.0f) {
                        isLineSafe = false;
                        break;
                    }
                }

                // 规则校验
                if (checkResultsPtr->count(code) && !checkResultsPtr->at(code).pass)
                {
                    isLineSafe = false; // 只要缓冲区里有一个网格违规，这条视线就被否决
                    break;
                }
            }

            if (isLineSafe)
            {
                targetIndex++;
            }
            else
            {
                // 不安全撞墙了，退回到上一个确认安全的点作为必经拐点
                smoothPath.push_back(originalPath[targetIndex - 1]);
                //将退回的节点当作下一次探索的起点
                currentIndex = targetIndex - 1;
                targetIndex = currentIndex + 2;
            }
        }
        smoothPath.push_back(originalPath.back());
        co_return smoothPath;
    }
// === 接口实现 ===
    // 1. 公开接口：原始路径 (原有的接口，保持向后兼容)
    Task<void> Astar::AstarPathPlane(const drogon::HttpRequestPtr req,
                                     std::function<void (const drogon::HttpResponsePtr &)> callback)
{
    // 调用私有核心方法，传入 false 表示不进行抽稀处理
    co_await processPathRequest(req, callback, false);
}

    // 2. 公开接口：抽稀路径 (新增的接口)
    Task<void> Astar::SmoothAstarPathPlane(const drogon::HttpRequestPtr req,
                                           std::function<void (const drogon::HttpResponsePtr &)> callback)
{
    // 调用私有核心方法，传入 true 表示开启 thinPathGreedy 抽稀逻辑
    co_await processPathRequest(req, callback, true);
}

    // 3. 私有核心逻辑：承载原本的 A* 寻路与路径处理代码
    Task<void> Astar::processPathRequest(const drogon::HttpRequestPtr req,
                                         std::function<void (const drogon::HttpResponsePtr &)> callback,
                                         bool applySmoothing)

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
       bool enableTrueHeightCheck = (*jsonBody).get("enableTrueHeightCheck", false).asBool();

        // ==========================================
        // [修改 1]：起飞垂直航线 (verticalPath)
        // ==========================================
        vector<string> verticalPath;
        int startWorkLayer = 0;
        if (pointsArr.size() > 0) {
            Json::Value firstPoint = pointsArr[0];
            double lon = firstPoint[0].asDouble();
            double lat = firstPoint[1].asDouble();
            double originalHeight = firstPoint[2].asDouble(); // 起点地面海拔

            double absoluteWorkHeight = originalHeight + workHeight; // 起点绝对作业海拔
            IJH workIJH = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, absoluteWorkHeight, baseTile);
            startWorkLayer = static_cast<int>(workIJH.layer);

            IJH originalIJH = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, originalHeight, baseTile);
            int col = static_cast<int>(originalIJH.column);
            int row = static_cast<int>(originalIJH.row);
            int originalLayer = static_cast<int>(originalIJH.layer);

            if (originalLayer > startWorkLayer) {
                for (int h = originalLayer; h >= startWorkLayer; --h) {
                    IJH ijh = {(uint32_t)row, (uint32_t)col, (uint32_t)h};
                    verticalPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            } else {
                for (int h = originalLayer; h <= startWorkLayer; ++h) {
                    IJH ijh = {(uint32_t)row, (uint32_t)col, (uint32_t)h};
                    verticalPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
        }

        // ==========================================
        // [修改 2]：降落垂直航线 (landingPath) - 动态终点高度
        // ==========================================
        vector<string> landingPath;
        if (pointsArr.size() > 1) {
            Json::Value lastPoint = pointsArr[pointsArr.size() - 1];
            double endLon = lastPoint[0].asDouble();
            double endLat = lastPoint[1].asDouble();
            double endHeight = lastPoint[2].asDouble(); // 终点地面海拔

            // 计算终点地面层
            IJH endOriginalIJH = localRowColHeiNumber(static_cast<uint8_t>(level), endLon, endLat, endHeight, baseTile);
            int endCol = static_cast<int>(endOriginalIJH.column);
            int endRow = static_cast<int>(endOriginalIJH.row);
            int endOriginalLayer = static_cast<int>(endOriginalIJH.layer);

            // 计算终点的高空作业层 (终点地面 + workHeight)
            double endAbsoluteWorkHeight = endHeight + workHeight;
            IJH endWorkIJH = localRowColHeiNumber(static_cast<uint8_t>(level), endLon, endLat, endAbsoluteWorkHeight, baseTile);
            int endWorkLayer = static_cast<int>(endWorkIJH.layer);

            if (endWorkLayer > endOriginalLayer) {
                for (int h = endWorkLayer - 1; h >= endOriginalLayer; --h) {
                    IJH ijh = {(uint32_t)endRow, (uint32_t)endCol, (uint32_t)h};
                    landingPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
            else if (endWorkLayer < endOriginalLayer) {
                for (int h = endWorkLayer + 1; h <= endOriginalLayer; ++h) {
                    IJH ijh = {(uint32_t)endRow, (uint32_t)endCol, (uint32_t)h};
                    landingPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
        }

        // ==========================================
        // [修改 3]：仿地飞行航路点 Z 轴分配 (核心修复)
        // ==========================================
        for (size_t i = 0; i < waypoints.size(); ++i) {
            Json::Value point = pointsArr[static_cast<int>(i)];
            double groundHeight = point[2].asDouble(); // 提取该点自身的地面海拔

            // 目标海拔 = 该点地面海拔 + 作业高度 (实现完美贴地)
            double absoluteTargetHeight = groundHeight + workHeight;

            IJH wpIJH = localRowColHeiNumber(static_cast<uint8_t>(level),
                                             point[0].asDouble(),
                                             point[1].asDouble(),
                                             absoluteTargetHeight,
                                             baseTile);
            waypoints[i][2] = static_cast<int>(wpIJH.layer);
        }

        // 为了兼容后续调用 A* 时传入的 workLayer 参数，将其指向起点的作业层
        int workLayer = startWorkLayer;

        AStarOptions options;
        options.speed = cruisingSpeed;

        RouteMode currentMode = RouteMode::ORIGINAL; //默认为原始A星
        if (jsonBody->isMember("route_type")) {
            std::string reqMode = (*jsonBody)["route_type"].asString();
            if (reqMode == "shortest") currentMode = RouteMode::SHORTEST;
            else if (reqMode == "safest") currentMode = RouteMode::SAFEST;
            else if (reqMode == "balanced") currentMode = RouteMode::BALANCED;
            else if (reqMode == "original") currentMode = RouteMode::ORIGINAL;
        } else if (jsonBody->isMember("mode")) {
            std::string reqMode = (*jsonBody)["mode"].asString();
            if (reqMode == "shortest") currentMode = RouteMode::SHORTEST;
            else if (reqMode == "safest") currentMode = RouteMode::SAFEST;
            else if (reqMode == "balanced") currentMode = RouteMode::BALANCED;
            else if (reqMode == "original") currentMode = RouteMode::ORIGINAL;
        }

        Json::Value baseRules;

        // 1. 默认提取 weight.json 的 rules 节点进行算路拦截和代价计算
        if (g_weightConfig.isObject() && g_weightConfig.isMember("rules")) {
            baseRules = g_weightConfig["rules"];
        } else {
            baseRules = Json::Value(Json::objectValue);
        }

        Json::Value ruleOptions(Json::objectValue);

        // 2. 解析前端 condition 中指定的各影响因素网格层级（格式如: {"dc_7": {}}）
        auto processFrontendCond = [&](const Json::Value& cond) {
            Json::Value merged(Json::objectValue);
            for (const auto& key : cond.getMemberNames()) {
                std::string baseKey = key;
                std::string levelStr = "";
                // 解析前端传的键名，例如从 "dc_7" 中提取出 "dc" 和 "7"
                size_t underscore = key.find('_');
                if (underscore != std::string::npos) {
                    baseKey = key.substr(0, underscore);
                    levelStr = key.substr(underscore + 1);
                }

                // 寻找 weight.json 中对应的基础配置
                std::string configKey = baseKey;
                if (!baseRules.isMember(baseKey)) {
                    for (const auto& bk : baseRules.getMemberNames()) {
                        if (bk.find(baseKey + "_") == 0) {
                            configKey = bk;
                            break;
                        }
                    }
                }

                // 如果找到配置，则将动态指定的层级拼接到键名上，供 GridEvaluator 解析
                if (baseRules.isMember(configKey) && cond[key].empty()) {
                    // 前端传的是空对象（如 "dc_14": {}），采用后端 weight.json 默认配置
                    if (!levelStr.empty()) {
                        merged[baseKey + "_" + levelStr] = baseRules[configKey];
                    } else {
                        merged[key] = baseRules[configKey];
                    }
                } else {
                    // 前端传了具体内容（或者是一个全新未知的key），直接使用前端传的内容
                    merged[key] = cond[key];
                }
            }
            return merged;
        };

        if (jsonBody->isMember("condition") && !(*jsonBody)["condition"].empty()) {
            ruleOptions = processFrontendCond((*jsonBody)["condition"]);
        } else if (jsonBody->isMember("options") && !(*jsonBody)["options"].empty()) {
            ruleOptions = processFrontendCond((*jsonBody)["options"]);
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
                segmentResult = aStarPathSimple(waypoints[i], waypoints[i + 1], options, level, workLayer, enableTrueHeightCheck);
            } else {
                LOG_INFO << "[A*] 航段 " << i+1 << " 使用约束模式（协程版A*）";
                segmentResult = co_await aStarPath(
                    waypoints[i], waypoints[i + 1], currentSegmentStartTime, planeRadius, options, level, gridEvaluator,
                    workLayer, currentMode, enableTrueHeightCheck
                );
            }

            if (!segmentResult.success) {
                pathSuccess = false;
                failReason = segmentResult.reason;
                break;
            }
            // 调用平滑函数 (根据 applySmoothing 标志决定是否执行)
            if (applySmoothing && !isUnconstrained && !segmentResult.path.empty() && gridEvaluator) {
                LOG_INFO << "[A*] 航段 " << i+1 << " 开始执行A*航线抽稀...";
                segmentResult.path = co_await thinPathGreedy(segmentResult.path, gridEvaluator, currentSegmentStartTime, level, enableTrueHeightCheck);
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
                if (applySmoothing) {
                    // 1. 抽稀接口专属返回格式：仅保留 [lon, lat, height]
                    Json::Value pointArray(Json::arrayValue);
                    pointArray.append(boundary.longitude);
                    pointArray.append(boundary.latitude);
                    pointArray.append(boundary.height);
                    results["path"].append(pointArray);
                }else{
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