#include "api_airRoute_Astar.h"
#include "GridEvaluator.h"
#include <drogon/drogon.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/DQG3DProximity.h>
#include <dqg/GlobalBaseTile.h>

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
// num: 要求平方根的数
// iters: 迭代次数，默认5次即可达到较高精度
static double newton(double num, int iters = 5) {
    if (num <= 0) return 0.0;
    double x = num / 2.0;
    for (int i = 0; i < iters; ++i) x = 0.5 * (x + num / x);
    return x;
}

// 根据层级获取网格大小（单位：米）
// level: 网格层级，支持7-14级，层级越高网格越精细
// 返回值：该层级对应的网格边长（米）
    double getGridSize(int level) {
    if (level < 1 || level > 21) {
        throw std::invalid_argument("Unsupported level: " + to_string(level));
    }
    return 78125.0 / std::pow(2.0, level);
}

// A*算法搜索方向定义，共26个方向（3D空间的26连通性）
// 前6个：6个主轴方向（上下左右前后）
// 中12个：12个面对角线方向
// 后8个：8个体对角线方向
const vector<array<int, 3>> DIRECTIONS = {
    {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
    {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
    {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
    {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
    {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
    {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
};

// 各方向对应的距离（相对于网格边长的倍数）
// 前6个：1.0（主轴方向）
// 中12个：1.4142135623730951（面对角线，√2）
// 后8个：1.7320508075688772（体对角线，√3）
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
    int wdTime;    // 规范化后的时间戳（按天或按小时对齐）
    string wdRule; // 时间规则标识（"wdd_11"表示天级别，"wdh_11"表示小时级别）
};

// 获取北京时间（UTC+8）的当前时间戳（秒）
// @return 北京时间的Unix时间戳
int getBeijingTime() {
    // 直接获取UTC时间并加上8小时（北京时间 = UTC + 8）
    return static_cast<int>(time(nullptr)) + 8 * 3600;
}

// 规范化网格时间，将时间戳对齐到合适的时间粒度（基于北京时间）
// gridTime: 到达网格的时间戳（北京时间）
// currentTime: 当前系统时间戳（北京时间）
// 返回值：规范化后的时间和规则标识
// 注意：所有时间戳都应该是北京时间（UTC+8）
NormalizedTime normalizeGridTime(int gridTime, int currentTime) {
    // 计算与当前时间的时间差
    long diff = std::abs(static_cast<long>(gridTime) - static_cast<long>(currentTime));

    // 如果时间差超过一天，使用天级别的粒度（wdd_11）
    if (diff > 86400L) {
        // 将时间戳归一化到北京时间该天的0时（北京时间午夜）
        // 步骤：先加8小时转到北京时区 -> 按天取整 -> 再减8小时转回UTC基准
        // 公式：((timestamp + 8*3600) / 86400) * 86400 - 8*3600
        const int UTC_OFFSET = 8 * 3600;
        int normalizedTime = ((gridTime + UTC_OFFSET) / 86400) * 86400 - UTC_OFFSET;
        return {normalizedTime, "wdd_11"};
    }
    // 否则使用小时级别的粒度（wdh_11）
    else {
        // 将时间戳归一化到北京时间的小时整点
        // 步骤：先加8小时转到北京时区 -> 按小时取整 -> 再减8小时转回UTC基准
        const int UTC_OFFSET = 8 * 3600;
        int normalizedTime = ((gridTime + UTC_OFFSET) / 3600) * 3600 - UTC_OFFSET;
        return {normalizedTime, "wdh_11"};
    }
}

// [优化] 定义整型网格坐标键，用于快速比较和哈希，避免字符串操作开销
struct GridKey {
    int x; // 列坐标（col）
    int y; // 行坐标（row）
    int z; // 层坐标（layer）

    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const GridKey& other) const {
        return !(*this == other);
    }
};

// [优化] 坐标键的哈希函数，用于unordered_map/unordered_set
struct GridKeyHash {
    std::size_t operator()(const GridKey& k) const {
        // 使用质数乘法和哈希组合，减少冲突
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
    bool success;           // 是否成功找到路径
    vector<string> path;     // 路径上的网格编码列表
    string reason;          // 失败原因（success为false时有意义）
};

// === 协程适配器 ===
// 将GridEvaluator的异步检查接口适配为C++20协程的awaitable对象
struct GridCheckAwaiter {
    std::shared_ptr<GridEvaluator> evaluator;           // 网格评估器实例
    const std::vector<CandidateInfo>& candidates;        // 待检查的候选网格列表

    // 使用 shared_ptr 存储结果 Map，避免编译器构造/析构问题
    std::shared_ptr<std::unordered_map<std::string, GridEvaluator::CheckResult>> result;

    // await_ready: 是否已经准备好（不等待），这里总是返回false，表示需要挂起
    bool await_ready() { return false; }

    // await_suspend: 挂起协程，启动异步检查
    // h: 当前协程句柄，检查完成后用于恢复协程执行
    void await_suspend(std::coroutine_handle<> h) {
        // 调用evaluator的异步检查接口，传入lambda回调
        evaluator->checkCandidates(candidates, [this, h](const std::unordered_map<std::string, GridEvaluator::CheckResult>& res) mutable {
            // 拷贝结果到shared_ptr
            this->result = std::make_shared<std::unordered_map<std::string, GridEvaluator::CheckResult>>(res);
            // 恢复协程执行
            h.resume();
        });
    }

    // await_resume: 协程恢复后返回检查结果
    std::shared_ptr<std::unordered_map<std::string, GridEvaluator::CheckResult>> await_resume() { return result; }
};

// === A* 核心逻辑 (简化版 - 无约束条件) ===
// 当condition为空时使用，不调用GridEvaluator，直接进行常规A*搜索
// 参数说明：
//   start: 起点网格坐标 [col, row, layer]
//   end: 终点网格坐标 [col, row, layer]
//   options: A*算法配置（速度等）
//   level: 网格层级（7-14）
// 返回值：包含路径信息的结果结构
AStarResult aStarPathSimple(
    array<int, 3> start,
    array<int, 3> end,
    const AStarOptions& options,
    int level,
    int workLayer
) {
    // 获取当前层级的网格大小
    double gridSize;
    try {
        gridSize = getGridSize(level);
    } catch (const exception& e) {
        return {false, {}, string("不支持的 level: ") + to_string(level)};
    }

    // 解构起点和终点坐标
    int sx = start[0], sy = start[1], sz = start[2];
    int ex = end[0], ey = end[1], ez = end[2];

    // 坐标合法性检查
    if (sx < 0 || sy < 0 ||  ex < 0 || ey < 0 ) {
        return {false, {}, "行列坐标不能为负数"};
    }

    // 启发式函数：估算从当前点到终点的欧几里得距离
    auto heuristic = [&](int x, int y, int z) {
        return newton((x - ex)*(x - ex) + (y - ey)*(y - ey) + (z - ez)*(z - ez)) * gridSize;
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
    GridKey startKey = {sx, sy, sz};
    GridKey endKey = {ex, ey, ez};

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

    // A*主循环
    while (!openSet.empty()) {
        if (++searchSteps > MAX_SEARCH_STEPS) {
            return {false, {}, "路径计算超时: 搜索范围过大"};
        }

        Node cur = openSet.top();
        openSet.pop();

        if (closedSet.count(cur.key)) continue;

        auto it = openMap.find(cur.key);
        if (it == openMap.end() || abs(it->second.g - cur.g) > 1e-6) continue;

        // 到达终点：回溯路径
        if (cur.key == endKey) {
            vector<string> path;
            GridKey currKey = endKey;

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

        // 扩展26个邻居（无约束条件，直接全部加入开放列表）
        for (size_t i = 0; i < DIRECTIONS.size(); ++i) {
            const auto& d = DIRECTIONS[i];
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            int nz = cur.z + d[2];

            // 边界检查
            if (nx < 0 || ny < 0 || nz < 0) continue;
            //if (nz != workLayer) continue;
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;

            GridKey nKey = {nx, ny, nz};
            if (closedSet.count(nKey)) continue;

            double moveDist = DIRECTION_DISTANCES[i] * gridSize;
            double newG = cur.g + moveDist;

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
// 使用A*算法在3D网格空间中寻找从起点到终点的可行路径
// 参数说明：
//   start: 起点网格坐标 [col, row, layer]
//   end: 终点网格坐标 [col, row, layer]
//   startTime: 起飞时间戳（秒，北京时间 UTC+8）
//   planeRadius: 飞行器半径（米）
//   options: A*算法配置（速度等）
//   level: 网格层级（7-14）
//   evaluator: 网格评估器，用于检查网格通行性
// 返回值：包含路径信息的结果结构
Task<AStarResult> aStarPath(
    array<int, 3> start,
    array<int, 3> end,
    int startTime,
    double planeRadius,
    const AStarOptions& options,
    int level,
    std::shared_ptr<GridEvaluator> evaluator,
    int workLayer
) {
    // 获取当前北京时间（UTC+8），用于规范化时间
    int currentTime = getBeijingTime();

    // 获取当前层级的网格大小
    double gridSize;
    try {
        gridSize = getGridSize(level);
    } catch (const exception& e) {
        co_return {false, {}, string("不支持的 level: ") + to_string(level)};
    }

    // 解构起点坐标
    int sx = start[0], sy = start[1], sz = start[2];
    // 解构终点坐标
    int ex = end[0], ey = end[1], ez = end[2];

    // 坐标合法性检查：不允许负数坐标
    if (sx < 0 || sy < 0 ||  ex < 0 || ey < 0 ) {
        co_return {false, {}, "行列坐标不能为负数"};
    }

    // 将起点坐标转换为网格编码
    IJH startIJH = {(uint32_t)sy, (uint32_t)sx, (uint32_t)sz};
    std::string startCode = rchToCode(startIJH, static_cast<uint8_t>(level));
    // 将终点坐标转换为网格编码
    IJH endIJH = {(uint32_t)ey, (uint32_t)ex, (uint32_t)ez};
    std::string endCode = rchToCode(endIJH, static_cast<uint8_t>(level));

    // 创建起点和终点的GridKey（用于快速比较）
    GridKey startKey = {sx, sy, sz};
    GridKey endKey = {ex, ey, ez};

    // ==========================================
    // 1. 起点/终点 基础有效性检查
    // ==========================================
    {
        // 规范化起点时间，获取合适的时间粒度
        auto startNorm = normalizeGridTime(startTime, currentTime);

        // 起点：全量检查，包括时间规则
        CandidateInfo startCand = {
            startCode,      // 网格编码
            startTime,      // 到达时间
            startNorm.wdTime,   // 规范化时间
            startNorm.wdRule,   // 时间规则
            true            // 应用时间规则
        };

        // 终点：忽略时间规则（dt, wdd, wdh, dp）
        // 因为到达终点时无需考虑时间约束
        CandidateInfo endCand = {
            endCode,
            0,              // 时间设为0，表示忽略
            0,
            "",
            false           // 不应用时间规则
        };

        // 将起点和终点加入预检查列表
        std::vector<CandidateInfo> preCheckCands = {startCand, endCand};

        // 执行异步检查
        auto preCheckResultsPtr = co_await GridCheckAwaiter{evaluator, preCheckCands};

        // 验证起点是否可通行
        if (preCheckResultsPtr->count(startCode)) {
            const auto& res = preCheckResultsPtr->at(startCode);
            if (!res.pass) {
                co_return {false, {}, "起点不可通行: " + res.reason};
            }
        }

        // 验证终点是否可通行
        if (preCheckResultsPtr->count(endCode)) {
            const auto& res = preCheckResultsPtr->at(endCode);
            if (!res.pass) {
                co_return {false, {}, "终点不可通行: " + res.reason};
            }
        }
    }
    // [已删除] 终点隔离性预检查 (BFS) - 之前用于检查终点是否被障碍物完全包围

    // ==========================================
    // 2. A* 主循环 (使用 GridKey 优化)
    // ==========================================

    // 启发式函数：估算从当前点到终点的欧几里得距离
    // 使用平方根近似计算，作为A*的h(n)估计值
    auto heuristic = [&](int x, int y, int z) {
        return newton((x - ex)*(x - ex) + (y - ey)*(y - ey) + (z - ez)*(z - ez)) * gridSize;
    };

    // A*节点结构
    struct Node {
        int x, y, z;        // 网格坐标
        double g, h, f;     // g: 从起点到当前的实际代价，h: 启发式估计到终点的代价，f = g + h
        int arrivalTime;    // 到达该节点的时间戳
        GridKey key;        // 唯一标识符
        size_t seq;         // 序列号，用于打破优先队列中的平局

        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, int at_, size_t s_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), arrivalTime(at_), key({x_, y_, z_}), seq(s_) {}

        // 优先队列比较：f值越小优先级越高，f相同时seq越小优先级越高
        bool operator>(const Node& o) const {
            if (abs(f - o.f) > 1e-6) return f > o.f;
            return seq > o.seq;
        }
    };

    // 全局序列号计数器，用于保持节点插入顺序
    static size_t globalSeq = 0;

    // A*核心数据结构
    priority_queue<Node, vector<Node>, greater<Node>> openSet;      // 开放列表：待探索节点，按f值排序的小顶堆
    std::unordered_map<GridKey, Node, GridKeyHash> openMap;          // 开放列表映射：快速查找和更新节点
    std::unordered_set<GridKey, GridKeyHash> closedSet;              // 关闭列表：已探索节点
    std::unordered_map<GridKey, GridKey, GridKeyHash> parent;        // 父节点映射：用于路径回溯

    // 初始化：将起点加入开放列表
    double h0 = heuristic(sx, sy, sz);
    Node startNode(sx, sy, sz, 0.0, h0, startTime, globalSeq++);
    openSet.push(startNode);
    openMap[startKey] = startNode;

    // 记录最后一次失败的原因，用于调试
    string lastFailReason = "no_path_found";
    // === 新增：记录发生冲突时，距离终点的最小预估距离 ===
    double minDistanceToTargetOnFail = std::numeric_limits<double>::max();
    // 搜索步数计数器
    int searchSteps = 0;
    // 搜索步数上限：防止死循环或搜索过久
    const int MAX_SEARCH_STEPS = g_maxSearchSteps;

    // A*主循环：从开放列表中取出f值最小的节点进行扩展
    while (!openSet.empty()) {
        // 超时/超步数检查
        if (++searchSteps > MAX_SEARCH_STEPS) {
            co_return {false, {}, "路径计算超时: 搜索范围过大或目标不可达 (" + lastFailReason + ")"};
        }

        // 取出f值最小的节点
        Node cur = openSet.top();
        openSet.pop();

        // 跳过已关闭的节点（重复处理）
        if (closedSet.count(cur.key)) continue;

        // 验证节点是否在openMap中且g值一致（防止过期节点）
        auto it = openMap.find(cur.key);
        if (it == openMap.end() || abs(it->second.g - cur.g) > 1e-6) continue;

        // 到达终点：回溯路径并返回
        if (cur.key == endKey) {
            vector<string> path;
            GridKey currKey = endKey;

            // 将终点加入路径
            IJH lastIJH = {(uint32_t)currKey.y, (uint32_t)currKey.x, (uint32_t)currKey.z};
            path.push_back(rchToCode(lastIJH, static_cast<uint8_t>(level)));

            // 从终点回溯到起点
            while (parent.count(currKey)) {
                currKey = parent[currKey];
                IJH ijh = {(uint32_t)currKey.y, (uint32_t)currKey.x, (uint32_t)currKey.z};
                path.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
            }
            // 反转路径，使其从起点到终点
            reverse(path.begin(), path.end());
            co_return {true, path, ""};
        }

        // 将当前节点加入关闭列表
        closedSet.insert(cur.key);
        openMap.erase(cur.key);

        // ==========================================
        // 邻居约束批量检查（26连通性 + 缓冲区）
        // ==========================================
        // 邻居元数据结构
        struct NeighborMeta { int x, y, z; string code; double moveCost; int arrival; };
        vector<NeighborMeta> validNeighbors;
        vector<CandidateInfo> candidateListForChecker;
        validNeighbors.reserve(26);
        candidateListForChecker.reserve(26);

        // 计算最大坐标值：2^(3*level)，用于边界检查
        uint64_t maxCoord = (1ULL << (3 * level));

        // 遍历26个方向，生成邻居节点
        for (size_t i = 0; i < DIRECTIONS.size(); ++i) {
            const auto& d = DIRECTIONS[i];
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            int nz = cur.z + d[2];

            // 坐标边界检查：不允许负数
            if (nx < 0 || ny < 0 || nz < 0) continue;
            //if (nz != workLayer) continue;
            // 坐标上界检查
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;

            GridKey nKey = {nx, ny, nz};

            // 如果节点已经关闭，直接跳过，省去字符串生成
            if (closedSet.count(nKey)) continue;

            // 计算移动距离和时间
            double moveDist = DIRECTION_DISTANCES[i] * gridSize;
            int stepTime = static_cast<int>(moveDist / options.speed);
            int arrival = cur.arrivalTime + stepTime;

            // 生成邻居网格编码
            auto norm = normalizeGridTime(arrival, currentTime);
            IJH nextIJH = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
            string code = rchToCode(nextIJH, static_cast<uint8_t>(level));

            // 记录邻居信息
            validNeighbors.push_back({nx, ny, nz, code, moveDist, arrival});

            // 加入候选检查列表
            candidateListForChecker.push_back({
                code,
                arrival,
                norm.wdTime,
                norm.wdRule,
                true
            });
        }

        // 批量异步检查所有邻居的通行性（26个网格）
        std::shared_ptr<std::unordered_map<string, GridEvaluator::CheckResult>> checkResultsPtr;
        if (!candidateListForChecker.empty()) {
            checkResultsPtr = co_await GridCheckAwaiter{evaluator, candidateListForChecker};
        }

        // ==========================================
        // 新增：缓冲区检查 - 如果任意一个邻居不可通行，则跳过当前节点
        // 起点节点不做邻居缓冲检查，只检查自身通行性（已在前置检查完成）
        // ==========================================
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

        // 如果有任何一个邻居不可通行，跳过当前节点（不扩展任何邻居）
        if (!allNeighborsPassable) {
            continue; // 跳过当前节点，继续下一个节点
        }

        // ==========================================
        // 所有26个邻居都可通行，开始将邻居加入开放列表
        // ==========================================
        for (const auto& nb : validNeighbors) {
            // 此时所有邻居都已通过检查，无需再次验证通行性

            // 计算新的g值（从起点到该邻居的实际代价）
            double newG = cur.g + nb.moveCost;

            // 如果该邻居已在开放列表中且新g值不更优，跳过
            GridKey nKey = {nb.x, nb.y, nb.z};
            auto existing = openMap.find(nKey);
            if (existing != openMap.end() && newG >= existing->second.g) continue;

            // 计算启发式h值，创建新节点
            double newH = heuristic(nb.x, nb.y, nb.z);
            Node next(nb.x, nb.y, nb.z, newG, newH, nb.arrival, globalSeq++);

            // 将新节点加入开放列表
            openSet.push(next);
            openMap[nKey] = next;
            parent[nKey] = cur.key;
        }
    }

    // 开放列表为空，未找到路径
    co_return {false, {}, lastFailReason};
}

// === 接口实现 ===

// A*无人机路径规划接口处理器
// 参数：
//   req: HTTP请求对象
//   callback: 响应回调函数
Task<void> Astar::AstarPathPlane(const drogon::HttpRequestPtr req,
                                 std::function<void (const drogon::HttpResponsePtr &)> callback)
{
    // 解析JSON请求体
    auto jsonBody = req->getJsonObject();
    if (!jsonBody) {
        // 请求体不是有效JSON
        Json::Value response;
        response["status"] = "error";
        response["message"] = "请求体必须是有效的JSON格式";
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        co_return;
    }

    try {
        // 检查必需参数：points（航点数组）
        if (!jsonBody->isMember("points")) {
            Json::Value response; response["status"] = "error"; response["message"] = "缺少必需参数: points";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        // 验证points参数格式：必须包含至少2个点
        Json::Value pointsArr = (*jsonBody)["points"];
        if (!pointsArr.isArray() || pointsArr.size() < 2) {
            Json::Value response; response["status"] = "error"; response["message"] = "points必须是包含至少2个点的数组";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        // 获取并验证level参数（网格层级），默认15
        int level = (*jsonBody).get("level", 14).asInt();
        try { getGridSize(level); }
        catch (...) {
            Json::Value response; response["status"] = "error"; response["message"] = "不支持的level";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        // 获取基础瓦片信息，用于坐标转换
        const BaseTile& baseTile = ::getProjectBaseTile();
        vector<array<int, 3>> waypoints;

        // 将经纬度坐标转换为网格坐标
        for (unsigned int i = 0; i < pointsArr.size(); ++i) {
            Json::Value point = pointsArr[i];
            double lon = point[0].asDouble();  // 经度
            double lat = point[1].asDouble();  // 纬度
            double height = point[2].asDouble();  // 高度（米）

            // 坐标值合法性检查
            if (lon < -180 || lon > 180 || lat < -90 || lat > 90 || height < baseTile.bottom) {
                 Json::Value response; response["status"] = "error"; response["message"] = "坐标值不合法";
                 auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
                 callback(resp); co_return;
            }

            // 经纬高 -> 网格行列层坐标
            IJH ijh = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, height, baseTile);
            int layer = static_cast<int32_t>(ijh.layer); // uint32_t → int32_t，负值会正确还原
            waypoints.push_back({static_cast<int>(ijh.column), static_cast<int>(ijh.row), layer});
        }

        // 解析起飞时间参数（支持毫秒时间戳，所有输入时间都应为北京时间 UTC+8）
        // 如果未提供，默认使用当前北京时间
        long long rawStartTime = (*jsonBody).get("startTime", static_cast<Json::Int64>(getBeijingTime())).asInt64();
        int startTime = (rawStartTime > 9999999999LL) ? static_cast<int>(rawStartTime / 1000) : rawStartTime;

        // 解析飞行器参数
        double planeRadius = (*jsonBody).get("planeRadius", 0.75).asDouble();  // 飞行器半径，默认0.75米
        double cruisingSpeed = (*jsonBody).get("cruisingSpeed", 15.0).asDouble();  // 巡航速度，默认15米/秒

        // 检查必需参数：workHeight（工作高度）
        if (!jsonBody->isMember("workHeight")) {
            Json::Value response; response["status"] = "error"; response["message"] = "缺少必需参数: workHeight";
            auto resp = HttpResponse::newHttpJsonResponse(response); resp->setStatusCode(k400BadRequest);
            callback(resp); co_return;
        }

        // 解析工作高度参数（单位：米）
        double workHeight = (*jsonBody)["workHeight"].asDouble();  // 工作高度，必需参数

        // 生成竖直段路径：从起点原始高度到工作高度（支持上升或下降）
        vector<string> verticalPath;
        int workLayer = 0;
        if (pointsArr.size() > 0) {
            Json::Value firstPoint = pointsArr[0];
            double lon = firstPoint[0].asDouble();
            double lat = firstPoint[1].asDouble();
            //这里是起飞点的绝对地面海拔
            double originalHeight = firstPoint[2].asDouble();
            // =================================================================
            // 【关键修改】：绝对巡航高程 = 起飞点绝对海拔 + 相对巡航高度
            // =================================================================
            double absoluteWorkHeight = originalHeight + workHeight;
            // 使用 localRowColHeiNumber 统一坐标系计算 workLayer
            // 避免直接除以 getGridSize 导致忽略 baseTile.bottom 偏移的问题
            IJH workIJH = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, absoluteWorkHeight, baseTile);
            workLayer = static_cast<int>(workIJH.layer);

            // 获取起点原始高度对应的层级
            IJH originalIJH = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, originalHeight, baseTile);
            int col = static_cast<int>(originalIJH.column);
            int row = static_cast<int>(originalIJH.row);
            int originalLayer = static_cast<int>(originalIJH.layer);

            // 判断是上升还是下降
            if (originalLayer > workLayer) {
                // 下降：从原始高度降到工作高度
                for (int h = originalLayer; h >= workLayer; --h) {
                    IJH ijh = {(uint32_t)row, (uint32_t)col, (uint32_t)h};
                    verticalPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            } else {
                // 上升：从原始高度升到工作高度
                for (int h = originalLayer; h <= workLayer; ++h) {
                    IJH ijh = {(uint32_t)row, (uint32_t)col, (uint32_t)h};
                    verticalPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
        }
        vector<string> landingPath;
        if (pointsArr.size() > 1) {
            // 获取用户传入的最后一个真实终点
            Json::Value lastPoint = pointsArr[pointsArr.size() - 1];
            double endLon = lastPoint[0].asDouble();
            double endLat = lastPoint[1].asDouble();
            double endHeight = lastPoint[2].asDouble(); // 例如你测试时的 70 米

            IJH endOriginalIJH = localRowColHeiNumber(static_cast<uint8_t>(level), endLon, endLat, endHeight, baseTile);
            int endCol = static_cast<int>(endOriginalIJH.column);
            int endRow = static_cast<int>(endOriginalIJH.row);
            int endOriginalLayer = static_cast<int>(endOriginalIJH.layer);

            // 如果工作高度(100米) > 终点高度(70米)，需要下降
            if (workLayer > endOriginalLayer) {
                // 从工作高度的下一层开始，往下一直降落到终点真实层级
                for (int h = workLayer - 1; h >= endOriginalLayer; --h) {
                    IJH ijh = {(uint32_t)endRow, (uint32_t)endCol, (uint32_t)h};
                    landingPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
            // 如果工作高度(50米) < 终点高度(70米)，需要最终爬升
            else if (workLayer < endOriginalLayer) {
                for (int h = workLayer + 1; h <= endOriginalLayer; ++h) {
                    IJH ijh = {(uint32_t)endRow, (uint32_t)endCol, (uint32_t)h};
                    landingPath.push_back(rchToCode(ijh, static_cast<uint8_t>(level)));
                }
            }
        }
        // 修改航点的高度为工作高度层级
        for (auto& wp : waypoints) {
            wp[2] = workLayer;
        }
        // 设置A*算法选项
        AStarOptions options;
        options.speed = cruisingSpeed;

        // 获取规则配置：支持options或condition两种字段名
        Json::Value ruleOptions;
        if (jsonBody->isMember("options")) ruleOptions = (*jsonBody)["options"];
        else ruleOptions = jsonBody->get("condition", Json::Value(Json::objectValue));

        // 判断是否为无约束条件（不启用GridEvaluator）
        // 条件：condition为null、空对象{}、或不存在
        bool isUnconstrained = ruleOptions.isNull() ||
                               (ruleOptions.isObject() && ruleOptions.empty());

        // 存储完整路径
        vector<string> fullPath;
        vector<int> pathIndexes;  // 记录每个网格属于第几段路径
        vector<string> waypointCodes;  // 记录途径点的网格编码
        bool pathSuccess = true;
        string failReason;

        if (isUnconstrained) {
            // ==========================================
            // 无约束条件分支：使用简化版A*（不创建GridEvaluator）
            // ==========================================
            LOG_INFO << "[A*] 使用无约束条件模式（简化版A*）";

            // 逐段规划路径：依次连接相邻航点
            for (size_t i = 0; i < waypoints.size() - 1 && pathSuccess; ++i) {
                AStarResult segmentResult = aStarPathSimple(
     waypoints[i], waypoints[i + 1], options, level, workLayer
 );

                if (!segmentResult.success) {
                    // 路径规划失败，记录失败原因
                    pathSuccess = false;
                    failReason = segmentResult.reason;
                } else {
                    // 路径规划成功，拼接路径
                    size_t segmentIndex = i + 1;  // 段索引从1开始

                    if (i == 0) {
                        // 第一段：全部加入
                        fullPath = segmentResult.path;
                        pathIndexes.assign(segmentResult.path.size(), segmentIndex);
                    } else {
                        // 后续段：跳过起点（与上一段终点重复）
                        fullPath.insert(fullPath.end(), segmentResult.path.begin() + 1, segmentResult.path.end());
                        pathIndexes.insert(pathIndexes.end(), segmentResult.path.size() - 1, segmentIndex);
                    }

                    // 记录途径点（除了最后一段的终点）
                    if (i < waypoints.size() - 2 && !segmentResult.path.empty()) {
                        waypointCodes.push_back(segmentResult.path.back());
                    }
                }
            }
        } else {
            // ==========================================
            // 有约束条件分支：使用协程版A*（原有逻辑）
            // ==========================================
            LOG_INFO << "[A*] 使用约束条件模式（协程版A*）";

            // 创建网格评估器
            auto gridEvaluator = GridEvaluator::create(ruleOptions);

            // 逐段规划路径：依次连接相邻航点
            for (size_t i = 0; i < waypoints.size() - 1 && pathSuccess; ++i) {
                AStarResult segmentResult = co_await aStarPath(
    waypoints[i], waypoints[i + 1], startTime, planeRadius, options, level, gridEvaluator,
    workLayer   // 新增
);

                if (!segmentResult.success) {
                    // 路径规划失败，记录失败原因
                    pathSuccess = false;
                    failReason = segmentResult.reason;
                } else {
                    // 路径规划成功，拼接路径
                    size_t segmentIndex = i + 1;  // 段索引从1开始

                    if (i == 0) {
                        // 第一段：全部加入
                        fullPath = segmentResult.path;
                        pathIndexes.assign(segmentResult.path.size(), segmentIndex);
                    } else {
                        // 后续段：跳过起点（与上一段终点重复）
                        fullPath.insert(fullPath.end(), segmentResult.path.begin() + 1, segmentResult.path.end());
                        pathIndexes.insert(pathIndexes.end(), segmentResult.path.size() - 1, segmentIndex);
                    }

                    // 记录途径点（除了最后一段的终点）
                    if (i < waypoints.size() - 2 && !segmentResult.path.empty()) {
                        waypointCodes.push_back(segmentResult.path.back());
                    }
                }
            }
        }

        // 构建响应
        Json::Value response;
        if (pathSuccess) {
            Json::Value results;
            results["success"] = true;
            results["path"] = Json::Value(Json::arrayValue);
            results["reason"] = Json::Value::null;

            // 拼接完整路径：竖直段 + 水平段（A*路径）
            vector<string> finalPath;
            vector<int> finalPathIndexes;
            vector<bool> finalIsVertical;

            // 添加竖直段
            finalPath.insert(finalPath.end(), verticalPath.begin(), verticalPath.end());
            finalPathIndexes.insert(finalPathIndexes.end(), verticalPath.size(), 0);  // 竖直段pathIndex为0
            finalIsVertical.insert(finalIsVertical.end(), verticalPath.size(), true);
            // 去掉起飞段的最后一个网格，因为它和下面水平段的第一个网格完全重合
            if (!finalPath.empty() && !fullPath.empty()) {
                finalPath.pop_back();
                finalPathIndexes.pop_back();
                finalIsVertical.pop_back();
            }
            // 添加水平段
            finalPath.insert(finalPath.end(), fullPath.begin(), fullPath.end());
            finalPathIndexes.insert(finalPathIndexes.end(), pathIndexes.begin(), pathIndexes.end());
            finalIsVertical.insert(finalIsVertical.end(), fullPath.size(), false);
            if (!landingPath.empty()) {
                int landingIndex = static_cast<int>(waypoints.size());
                finalPath.insert(finalPath.end(), landingPath.begin(), landingPath.end());
                finalPathIndexes.insert(finalPathIndexes.end(), landingPath.size(), landingIndex);
                finalIsVertical.insert(finalIsVertical.end(), landingPath.size(), true);
            }
            // 将网格路径转换为经纬高坐标，并添加标记
            for (size_t i = 0; i < finalPath.size(); ++i) {
                const auto& code = finalPath[i];
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

                // 添加路��段索引
                gridInfo["pathIndex"] = finalPathIndexes[i];

                // 添加竖直段标记
                if (finalIsVertical[i]) {
                    gridInfo["isVertical"] = true;
                }

                // 添加起点标记
                if (i == 0) {
                    gridInfo["isStart"] = true;
                }

                // 添加终点标记
                if (i == finalPath.size() - 1) {
                    gridInfo["isEnd"] = true;
                }

                // 添加途径点标记
                if (std::find(waypointCodes.begin(), waypointCodes.end(), code) != waypointCodes.end()) {
                    gridInfo["isWaypoint"] = true;
                }

                results["path"].append(gridInfo);
            }
            response["results"] = results;
            callback(HttpResponse::newHttpJsonResponse(response));
        } else {
            // 路径规划失败，返回错误信息
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
        // 捕获未预期的异常
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