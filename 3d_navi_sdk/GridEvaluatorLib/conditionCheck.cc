// 引入条件检查头文件
#include "conditionCheck.h"
// 引入Drogon框架，用于HTTP服务器和Redis客户端
#include <drogon/drogon.h>
// 引入DQG 3D基础库
#include <dqg/DQG3DBasic.h>
// 引入DQG 3D Tile库
#include <dqg/DQG3DTil.h>
// 引入数学库，用于数值计算
#include <cmath>
// 引入算法库，用于std::find_if等
#include <algorithm>
// 引入optional，用于可选值处理
#include <optional>
// 引入字符串流，用于字符串分割
#include <sstream>
// 引入无序映射
#include <unordered_map>
// 引入无序集合
#include <unordered_set>
// 引入有序集合
#include <set>
// 引入有序映射
#include <map>
// 引入数值限制，用于NaN等
#include <limits>
// 引入时间处理，用于获取当前时间
#include <ctime>
// 引入时区设置
#include <cstdlib>
// 引入数组容器
#include <array>
// 引入原子操作，用于线程安全计数
#include <atomic>
// 引入互斥锁，用于线程安全
#include <mutex>

// 使用Drogon的Redis客户端相关类型
using drogon::nosql::RedisClient;
using drogon::nosql::RedisResult;
using drogon::nosql::RedisResultType;

// 计划检查命名空间
namespace plancheck {

// 匿名命名空间，限制符号的作用域在当前文件内
namespace {

// 规则元数据结构体，定义单个检查规则的属性
struct Rule {
    std::string prefix;              // 规则前缀（如hl、fx、gd等）
    std::string type;                // 规则类型（string、set、hash-fields）
    std::string op;                  // 比较操作符（<=、>=、containsAny等）
    std::string jsonPath;            // JSON路径，用于提取字段值
    bool checkValueMustExist{false}; // 检查值必须存在标志
    bool checkValueNotEmpty{false};  // 检查值非空标志
    std::string description;         // 规则描述信息
    // 提取器函数，从options中提取期望值
    std::function<Json::Value(const Json::Value &, const std::string &)> extractor;
};

// 激活后的规则集合结构体，包含某种规则类型下的所有配置
struct ActiveRule {
    std::string type;                              // 规则类型
    int level{0};                                  // 匹配层级，决定网格编码的前几位
    std::vector<const Rule *> rules;               // 指向该类型下所有规则实例的指针集合
    std::vector<std::string> requestedFields;      // 需要从Redis查询的字段列表
};

// 单个网格的时间戳信息结构体，记录无人机在某个网格的时间信息
struct GridStamp {
    std::string code;           // 网格编码
    double arrivalTime{0.0};    // 到达该网格的时间（秒）
    int64_t wdTime{0};          // 天气数据时间戳，用于匹配对应时段的天气数据
    std::string wdRule;         // 天气规则类型（wdh_11为小时级，wdd_11为日级）
};

// 规则表定义，返回系统中所有支持的规则集合
const std::vector<Rule> &ruleSet() {
    // 使用静态变量确保只初始化一次
    static const std::vector<Rule> rules = {
        // 航路校验规则：检查网格是否在航路规划中
        {"hl", "string", "", "", true, false, "航路校验：必须存在于航路规划中", nullptr},

        // [新增] 航路避让规则 (hlz)：检查网格是否为航路
        // 逻辑：checkValueNotEmpty=true (如果Redis中有值，则表示是航路，视为障碍物，不可通行)
        // 注意：实际查询时会映射到 hl 键，复用 hl 的数据源
        {"hlz", "string", "", "", false, true, "航路避让：当前区域存在航路，不可穿越", nullptr},

        // 人口密集区域规则：检查该区域是否有人口密集标记
        {"fx", "string", "", "", false, true, "人口密集区域无法通行", nullptr},
        // 三维实景障碍物规则：检查是否存在实景建模的障碍物
        {"gd", "string", "", "", false, true, "存在三维实景障碍物冲突", nullptr},
        // 无人机实时占用规则：检查是否有其他无人机正在使用该区域
        {"dt", "string", "", "", false, true, "存在无人机实时占用冲突", nullptr},
        // 电子围栏规则：检查是否违反预设的电子围栏限制
        {"dz", "string", "", "", false, true, "存在电子围栏冲突", nullptr},
        // 障碍物规则：检查一般性障碍物冲突
        {"za", "string", "", "", false, true, "存在障碍物冲突", nullptr},
        // 电磁检查规则：检查电磁辐射强度是否超过安全限制
        {"dc", "string", "<=", "", false, false, "电磁辐射强度过大，禁止通行", nullptr},
        // 空域类型占用规则：检查空域类型是否与允许的类型冲突
        {"ad", "set", "containsAny", "", false, false, "空域类型占用冲突",
         // 自定义提取器：从options中提取空域类型列表
         [](const Json::Value &opts, const std::string &key) {
             Json::Value out(Json::arrayValue);
             if (!opts.isMember(key)) return out;
             const auto &val = opts[key];
             // 辅助函数：提取冒号前的部分（如果有）
             auto pushVal = [&out](const std::string &s) {
                 auto pos = s.find(':');
                 out.append(pos == std::string::npos ? s : s.substr(0, pos));
             };
             // 处理字符串格式的输入（逗号分隔）
             if (val.isString()) {
                 std::stringstream ss(val.asString());
                 std::string part;
                 while (std::getline(ss, part, ',')) { if (!part.empty()) pushVal(part); }
             } else if (val.isArray()) {
                 // 处理数组格式的输入
                 for (const auto &v : val) { if (v.isString()) pushVal(v.asString()); }
             }
             return out;
         }},
        // === 日级天气规则 (wdd) ===
        {"wdd", "hash-fields", "<=", "visibility", false, false, "能见度小于标准值，禁止通行", nullptr},
        {"wdd", "hash-fields", ">=", "humidity", false, false, "湿度大于标准值，禁止通行", nullptr},
        {"wdd", "hash-fields", ">=", "tem1", false, false, "最高温度大于标准值，禁止通行", nullptr},
        {"wdd", "hash-fields", "<=", "tem2", false, false, "最低温度小于标准值，禁止通行", nullptr},
        // === 小时级天气规则 (wdh) ===
        {"wdh", "hash-fields", "<=", "visibility", false, false, "能见度小于标准值，禁止通行", nullptr},
        {"wdh", "hash-fields", ">=", "humidity", false, false, "湿度大于标准值，禁止通行", nullptr},
        {"wdh", "hash-fields", "<=", "tem", false, false, "温度小于标准值，禁止通行", nullptr},
        {"wdh", "hash-fields", ">=", "windSpeed", false, false, "风速大于标准值，禁止通行", nullptr},
        {"wdh", "hash-fields", ">", "rainPcpn", false, false, "降雨量大于标准值，禁止通行", nullptr},
    };
    return rules;
}

// 字符串分割函数：将文本按指定分隔符分割成多个部分
// @param text 待分割的字符串
// @param sep 分隔符字符
// @return 分割后的字符串向量
std::vector<std::string> split(const std::string &text, char sep) {
    std::vector<std::string> parts;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, sep)) { parts.push_back(item); }
    return parts;
}

// 将JSON值转换为数值
// @param v JSON值对象
// @return 转换后的双精度浮点数，转换失败返回NaN
double toNumber(const Json::Value &v) {
    // 如果是数值类型，直接返回
    if (v.isNumeric()) return v.asDouble();
    // 如果是字符串类型，尝试转换为数值
    if (v.isString()) {
        try { return std::stod(v.asString()); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
    }
    // 其他类型返回NaN
    return std::numeric_limits<double>::quiet_NaN();
}

// 检查JSON值是否为空
// @param v JSON值对象
// @return 如果值为null、空字符串或空数组返回true，否则返回false
bool isEmptyExpected(const Json::Value &v) {
    return v.isNull() || (v.isString() && v.asString().empty()) || (v.isArray() && v.empty());
}

// 标准化时间戳，将毫秒级时间戳转换为秒级时间戳
// @param ts 原始时间戳
// @return 转换后的秒级时间戳
double normalizeStartSeconds(double ts) {
    return ts > 1e12 ? ts / 1000.0 : ts;
}

// 获取北京时间（UTC+8）的当前时间戳（秒）
// @return 北京时间的Unix时间戳
double getBeijingTime() {
    // 直接获取UTC时间并加上8小时（北京时间 = UTC + 8）
    return static_cast<double>(std::time(nullptr)) + 8.0 * 3600.0;
}

// 从配置选项中提取飞行速度
// @param options 包含speed字段的JSON对象
// @return 飞行速度（单位：米/秒），默认为15.0
double extractSpeed(const Json::Value &options) {
    if (options.isObject() && options.isMember("speed") && options["speed"].isNumeric()) {
        const double s = options["speed"].asDouble();
        return s > 0.0 ? s : 15.0;
    }
    return 15.0;
}

// 创建单个网格的时间戳信息（基于北京时间）
// @param code 网格编码
// @param arrival 到达该网格的时间（秒，北京时间）
// @param nowSec 当前时间（秒，北京时间）
// @return 网格时间戳信息
// 注意：所有时间戳都应该是北京时间（UTC+8）
GridStamp makeStamp(const std::string &code, double arrival, double nowSec) {
    GridStamp s;
    s.code = code;
    s.arrivalTime = arrival;
    // 计算到达时间与当前时间的差值
    const double diff = std::fabs(arrival - nowSec);
    const int64_t UTC_OFFSET = 8 * 3600;

    if (diff > 86400.0) {
        // 时间差超过一天，使用日级天气数据（86400秒=1天）
        // 将时间戳归一化到北京时间该天的0时（北京时间午夜）
        // 步骤：先加8小时转到北京时区 -> 按天取整 -> 再减8小时转回UTC基准
        int64_t arrivalInt = static_cast<int64_t>(arrival);
        s.wdTime = ((arrivalInt + UTC_OFFSET) / 86400) * 86400 - UTC_OFFSET;
        s.wdRule = "wdd_11";
    } else {
        // 时间差在一天内，使用小时级天气数据（3600秒=1小时）
        // 将时间戳归一化到北京时间的小时整点
        // 步骤：先加8小时转到北京时区 -> 按小时取整 -> 再减8小时转回UTC基准
        int64_t arrivalInt = static_cast<int64_t>(arrival);
        s.wdTime = ((arrivalInt + UTC_OFFSET) / 3600) * 3600 - UTC_OFFSET;
        s.wdRule = "wdh_11";
    }
    return s;
}

// 构建路径上所有网格的时间戳信息
// @param codes 网格编码列表（按路径顺序）
// @param startSec 起始时间（秒，北京时间 UTC+8）
// @param speed 飞行速度（米/秒）
// @return 所有网格的时间戳信息列表
std::vector<GridStamp> buildStamps(const std::vector<std::string> &codes, double startSec, double speed) {
    std::vector<GridStamp> out;
    if (codes.empty()) return out;
    // 获取当前时间（确保使用北京时间UTC+8）
    const double nowSec =   getBeijingTime();

    // 将所有网格编码转换为行列高坐标（IJH）
    std::vector<IJH> ijhs;
    ijhs.reserve(codes.size());
    for (const auto &c : codes) ijhs.push_back(getLocalTileRHC(c));

    // 从起始时间开始计算每个网格的到达时间
    double current = startSec;
    out.reserve(codes.size());
    for (size_t i = 0; i < codes.size(); ++i) {
        if (i > 0) {
            // 计算当前网格与前一个网格的距离
            const auto &a = ijhs[i - 1];
            const auto &b = ijhs[i];
            const double dr = static_cast<double>(static_cast<int64_t>(a.row) - static_cast<int64_t>(b.row));
            const double dc = static_cast<double>(static_cast<int64_t>(a.column) - static_cast<int64_t>(b.column));
            const double dh = static_cast<double>(static_cast<int64_t>(a.layer) - static_cast<int64_t>(b.layer));
            // 计算三维欧氏距离，乘以5.0是网格的物理尺寸（米）
            const double stepDistance = std::sqrt(dr * dr + dc * dc + dh * dh) * 5.0;
            // 根据飞行速度计算飞行时间，并累加到当前时间
            if (speed > 0.0) current += std::floor(stepDistance / speed);
        }
        out.push_back(makeStamp(codes[i], current, nowSec));
    }
    return out;
}

// 检查配置选项中是否包含时间冲突检查选项（dp_开头的选项）
// @param options 配置选项JSON对象
// @return 如果包含dp_开头的选项返回true，否则返回false
bool hasDpOption(const Json::Value &options) {
    if (!options.isObject()) return false;
    for (const auto &name : options.getMemberNames()) {
        if (name.rfind("dp_", 0) == 0) return true;
    }
    return false;
}

// 构建激活的规则集合，从配置选项中解析出需要检查的规则
// @param options 配置选项JSON对象，包含类似 "hl_11"、"fx_11" 这样的规则配置
// @return 激活的规则映射表，key为规则前缀，value为激活规则配置
std::unordered_map<std::string, ActiveRule> buildActiveRules(const Json::Value &options) {
    std::unordered_map<std::string, ActiveRule> active;
    if (!options.isObject()) return active;
    // 遍历所有配置选项
    for (const auto &key : options.getMemberNames()) {
        // 查找下划线位置，规则格式为 "前缀_层级"，如 "hl_11"
        const auto pos = key.find('_');
        if (pos == std::string::npos) continue;
        const std::string prefix = key.substr(0, pos);   // 提取前缀（如hl、fx）
        const std::string levelStr = key.substr(pos + 1); // 提取层级（如11）
        if (levelStr.empty()) continue;

        // 检查该前缀是否存在于规则表中
        const auto &rules = ruleSet();
        bool hasRule = false;
        for (const auto &r : rules) { if (r.prefix == prefix) { hasRule = true; break; } }
        if (!hasRule) continue;

        // 获取或创建该前缀的激活规则配置
        auto &conf = active[prefix];
        if (conf.type.empty()) {
            // 首次遇到该前缀，初始化配置
            const auto it = std::find_if(rules.begin(), rules.end(), [&](const Rule &r){return r.prefix==prefix;});
            conf.type = it->type;
            try { conf.level = std::stoi(levelStr); } catch(...) { conf.level = 0; }
        }

        // 对于hash-fields类型，收集需要查询的字段
        if (conf.type == "hash-fields") {
            const auto &obj = options[key];
            if (obj.isObject()) {
                for (const auto &f : obj.getMemberNames()) conf.requestedFields.push_back(f);
            }
        }
        // 将该前缀下的所有规则添加到激活规则列表
        for (const auto &r : rules) {
            if (r.prefix == prefix) conf.rules.push_back(&r);
        }
    }
    return active;
}

// 评估约束条件是否满足
// @param rule 规则定义
// @param expected 期望值
// @param actual 实际值
// @return 如果满足约束条件返回true，否则返回false
bool evaluateConstraint(const Rule &rule, const Json::Value &expected, const Json::Value &actual) {
    // 处理集合类型的containsAny操作
    if (rule.type == "set" && rule.op == "containsAny") {
        std::unordered_set<std::string> act;
        // 将实际值转换为集合（提取冒号前的部分）
        if (actual.isArray()) {
            for (const auto &v : actual) {
                const auto parts = split(v.asString(), ':');
                act.insert(parts.empty() ? v.asString() : parts.front());
            }
        }
        // 检查期望值中是否与实际值有交集
        if (expected.isArray()) {
            for (const auto &e : expected) {
                if (!e.isString()) continue;
                if (act.find(e.asString()) != act.end()) return false;
            }
        }
        return true;
    }

    // 将期望值和实际值都转换为数值
    double actNum = toNumber(actual);
    double expNum = toNumber(expected);
    // 如果转换失败，返回false（注意：GridEvaluator如果是NaN则跳过，这里沿用原逻辑可能比较严格，或者保持一致）
    // 为了与 GridEvaluator 一致性，如果无法转换数值，通常跳过数值比较。
    if (std::isnan(actNum) || std::isnan(expNum)) return true;

    // 特殊处理：能见度的<=操作实际上是>=（能见度越大越好）
    if (rule.jsonPath == "visibility" && rule.op == "<=") return actNum >= expNum;

    // 根据操作符进行比较
    if (rule.op == "<=") return actNum <= expNum;
    if (rule.op == ">=") return actNum >= expNum;
    if (rule.op == "<") return actNum < expNum;
    if (rule.op == ">") return actNum > expNum;
    if (rule.op == "==") return std::fabs(actNum - expNum) < 1e-9;
    if (rule.op == "!=") return std::fabs(actNum - expNum) > 1e-9;
    return true;
}

// === 异步检查上下文 ===
// 用于管理异步Redis查询的状态和回调
struct AsyncContext {
    std::mutex mutex;                                        // 互斥锁，保护共享数据
    std::atomic<int> pendingCount{0};                       // 待完成的请求数量
    std::function<void(ConflictResult)> callback;           // 完成后的回调函数

    // === 输入数据 ===
    std::vector<GridStamp> stamps;                          // 所有网格的时间戳信息
    Json::Value options;                                     // 配置选项
    bool hasDp;                                              // 是否需要检查时间冲突
    std::unordered_map<std::string, ActiveRule> activeRules; // 激活的规则集合

    // === 缓存数据（Redis查询结果） ===
    std::unordered_map<std::string, std::string> stringCache;                         // String类型数据缓存
    std::unordered_map<std::string, std::vector<std::string>> setCache;                // Set类型数据缓存
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> hashCache; // Hash类型数据缓存

    // 检查是否所有请求都完成，如果是则调用finish
    void checkDone() {
        if (--pendingCount <= 0) {
            finish();
        }
    }

    // 完成所有检查，生成最终结果并调用回调
    void finish() {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<ConflictInfo> allConflicts;

        // 遍历所有网格进行检查
        for (size_t idx = 0; idx < stamps.size(); ++idx) {
            const auto &st = stamps[idx];

            // 1. 检查时间冲突（dp选项）
            if (hasDp) {
                const std::string dpKey = "dp_" + st.code;
                if (setCache.count(dpKey)) {
                    const auto &ranges = setCache[dpKey];
                    // 遍历所有时间范围，检查是否重叠
                    for (const auto &r : ranges) {
                        const auto parts = split(r, ':');
                        if (parts.size() < 2) continue;
                        try {
                            double s = std::stod(parts[0]); // 起始时间
                            double e = std::stod(parts[1]); // 结束时间
                            // 标准化为秒级时间戳
                            if (s > 1e12) s /= 1000.0;
                            if (e > 1e12) e /= 1000.0;
                            // 检查到达时间是否在已有时间范围内
                            if (st.arrivalTime >= s && st.arrivalTime <= e) {
                                allConflicts.push_back({.code = st.code, .reason = "时间冲突：飞行时段与已有计划重叠", .index = idx});
                                break; // 找到时间冲突就跳出当前网格的时间范围检查
                            }
                        } catch (...) {}
                    }
                }
            }

            // 2. 检查其他规则
            for (const auto &entry : activeRules) {
                const std::string &prefix = entry.first; // 规则前缀
                const auto &conf = entry.second;           // 规则配置
                const int level = conf.level;              // 匹配层级

                // 针对天气规则的层级匹配 (如 wdh_11 vs wdd_11)
                if (conf.type == "hash-fields") {
                     std::string targetRule = prefix + "_" + std::to_string(level);
                     // 只检查匹配的天气规则类型
                     if (targetRule != st.wdRule) continue;
                }

                // 根据层级截取网格编码前缀
                const auto slice = st.code.substr(0, std::min(static_cast<size_t>(level), st.code.size()));

                // [关键修改] 键名重定向逻辑：如果前缀是 hlz，则查询 hl 的数据
                std::string queryPrefix = prefix;
                if (prefix == "hlz") {
                    queryPrefix = "hl";
                }
                const std::string redisKey = queryPrefix + "_" + slice;      // Redis键名（使用重定向后的前缀）

                const std::string logicalKey = prefix + "_" + std::to_string(level); // 逻辑键名（保持原始前缀用于查找配置）

                // 处理string类型的规则
                if (conf.type == "string") {
                    bool exists = stringCache.count(redisKey);
                    std::string val = exists ? stringCache[redisKey] : "";

                    for (const auto *rule : conf.rules) {
                        // 检查值必须存在
                        if (rule->checkValueMustExist) {
                            if (!exists || val.empty()) {
                                allConflicts.push_back({.code = st.code, .reason = rule->description, .index = idx});
                                break; // 找到冲突就跳出当前网格的当前类型规则检查
                            }
                            continue;
                        }
                        // 检查值必须为空
                        if (rule->checkValueNotEmpty) {
                            if (exists && !val.empty()) {
                                allConflicts.push_back({.code = st.code, .reason = rule->description, .index = idx});
                                break; // 找到冲突就跳出当前网格的当前类型规则检查
                            }
                            continue;
                        }

                        // 获取期望值
                        Json::Value expected;
                        if (rule->extractor) expected = rule->extractor(options, logicalKey);
                        else if (options.isMember(logicalKey)) expected = options[logicalKey];
                        if (isEmptyExpected(expected)) continue;

                        // 评估约束条件
                        if (!evaluateConstraint(*rule, expected, exists ? Json::Value(val) : Json::Value::null)) {
                            allConflicts.push_back({.code = st.code, .reason = rule->description, .index = idx});
                            break; // 找到冲突就跳出当前网格的当前类型规则检查
                        }
                    }
                }
                // 处理set类型的规则
                else if (conf.type == "set") {
                    Json::Value actual(Json::arrayValue);
                    if (setCache.count(redisKey)) {
                        for(const auto& v : setCache[redisKey]) actual.append(v);
                    }
                    for (const auto *rule : conf.rules) {
                        Json::Value expected;
                        if (rule->extractor) expected = rule->extractor(options, logicalKey);
                        else if (options.isMember(logicalKey)) expected = options[logicalKey];
                        if (isEmptyExpected(expected)) continue;
                        if (!evaluateConstraint(*rule, expected, actual)) {
                            allConflicts.push_back({.code = st.code, .reason = rule->description, .index = idx});
                            break; // 找到冲突就跳出当前网格的当前类型规则检查
                        }
                    }
                }
                // 处理hash-fields类型的规则（天气规则）
                else if (conf.type == "hash-fields") {
                     if (!options.isMember(logicalKey) || !options[logicalKey].isObject()) continue;
                     const Json::Value &expectedObj = options[logicalKey];
                     for (const auto *rule : conf.rules) {
                         // 构造字段名：字段名_时间戳
                         const std::string fieldName = rule->jsonPath + "_" + std::to_string(st.wdTime);
                         if (hashCache.count(redisKey) && hashCache[redisKey].count(fieldName)) {
                             std::string valStr = hashCache[redisKey][fieldName];
                             const Json::Value expectedVal = expectedObj.isMember(rule->jsonPath) ? expectedObj[rule->jsonPath] : Json::Value();
                             if (isEmptyExpected(expectedVal)) continue;
                             if (!evaluateConstraint(*rule, expectedVal, valStr)) {
                                 allConflicts.push_back({.code = st.code, .reason = rule->description, .index = idx});
                                 break; // 找到冲突就跳出当前网格的当前类型规则检查
                             }
                         }
                     }
                }
            }
        }

        // 根据是否有冲突决定结果
        if (!allConflicts.empty()) {
            ConflictResult result{
                .pass = false,
                .conflicts = allConflicts,
                .code = allConflicts[0].code,
                .reason = allConflicts[0].reason,
                .index = allConflicts[0].index
            };
            callback(result);
        } else {
            // 所有检查通过
            callback({.pass = true});
        }
    }
};

// === 异步检查上下文（返回第一个冲突版本） ===
// 用于管理异步Redis查询的状态和回调，遇到第一个冲突即返回
struct AsyncContextFirst {
    std::mutex mutex;                                        // 互斥锁，保护共享数据
    std::atomic<int> pendingCount{0};                       // 待完成的请求数量
    std::atomic<bool> conflictFound{false};                 // 是否已找到冲突
    std::function<void(ConflictResult)> callback;           // 完成后的回调函数

    // === 输入数据 ===
    std::vector<GridStamp> stamps;                          // 所有网格的时间戳信息
    Json::Value options;                                     // 配置选项
    bool hasDp;                                              // 是否需要检查时间冲突
    std::unordered_map<std::string, ActiveRule> activeRules; // 激活的规则集合

    // === 缓存数据（Redis查询结果） ===
    std::unordered_map<std::string, std::string> stringCache;                         // String类型数据缓存
    std::unordered_map<std::string, std::vector<std::string>> setCache;                // Set类型数据缓存
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> hashCache; // Hash类型数据缓存

    // 检查是否所有请求都完成，如果是则调用finish
    void checkDone() {
        if (--pendingCount <= 0) {
            finish();
        }
    }

    // 完成所有检查，生成最终结果并调用回调（遇到第一个冲突即返回）
    void finish() {
        // 如果已经找到冲突并返回，则不再处理
        if (conflictFound.load()) return;

        std::lock_guard<std::mutex> lock(mutex);

        // 再次检查，避免竞态条件
        if (conflictFound.load()) return;

        // 遍历所有网格进行检查
        for (size_t idx = 0; idx < stamps.size(); ++idx) {
            const auto &st = stamps[idx];

            // 1. 检查时间冲突（dp选项）
            if (hasDp) {
                const std::string dpKey = "dp_" + st.code;
                if (setCache.count(dpKey)) {
                    const auto &ranges = setCache[dpKey];
                    // 遍历所有时间范围，检查是否重叠
                    for (const auto &r : ranges) {
                        const auto parts = split(r, ':');
                        if (parts.size() < 2) continue;
                        try {
                            double s = std::stod(parts[0]); // 起始时间
                            double e = std::stod(parts[1]); // 结束时间
                            // 标准化为秒级时间戳
                            if (s > 1e12) s /= 1000.0;
                            if (e > 1e12) e /= 1000.0;
                            // 检查到达时间是否在已有时间范围内
                            if (st.arrivalTime >= s && st.arrivalTime <= e) {
                                conflictFound.store(true);
                                ConflictResult result{
                                    .pass = false,
                                    .conflicts = {{.code = st.code, .reason = "时间冲突：飞行时段与已有计划重叠", .index = idx}},
                                    .code = st.code,
                                    .reason = "时间冲突：飞行时段与已有计划重叠",
                                    .index = idx
                                };
                                callback(result);
                                return; // 找到第一个冲突，立即返回
                            }
                        } catch (...) {}
                    }
                }
            }

            // 2. 检查其他规则
            for (const auto &entry : activeRules) {
                const std::string &prefix = entry.first; // 规则前缀
                const auto &conf = entry.second;           // 规则配置
                const int level = conf.level;              // 匹配层级

                // 针对天气规则的层级匹配 (如 wdh_11 vs wdd_11)
                if (conf.type == "hash-fields") {
                     std::string targetRule = prefix + "_" + std::to_string(level);
                     // 只检查匹配的天气规则类型
                     if (targetRule != st.wdRule) continue;
                }

                // 根据层级截取网格编码前缀
                const auto slice = st.code.substr(0, std::min(static_cast<size_t>(level), st.code.size()));

                // [关键修改] 键名重定向逻辑：如果前缀是 hlz，则查询 hl 的数据
                std::string queryPrefix = prefix;
                if (prefix == "hlz") {
                    queryPrefix = "hl";
                }
                const std::string redisKey = queryPrefix + "_" + slice;      // Redis键名（使用重定向后的前缀）

                const std::string logicalKey = prefix + "_" + std::to_string(level); // 逻辑键名（保持原始前缀用于查找配置）

                // 处理string类型的规则
                if (conf.type == "string") {
                    bool exists = stringCache.count(redisKey);
                    std::string val = exists ? stringCache[redisKey] : "";

                    for (const auto *rule : conf.rules) {
                        // 检查值必须存在
                        if (rule->checkValueMustExist) {
                            if (!exists || val.empty()) {
                                conflictFound.store(true);
                                ConflictResult result{
                                    .pass = false,
                                    .conflicts = {{.code = st.code, .reason = rule->description, .index = idx}},
                                    .code = st.code,
                                    .reason = rule->description,
                                    .index = idx
                                };
                                callback(result);
                                return; // 找到第一个冲突，立即返回
                            }
                            continue;
                        }
                        // 检查值必须为空
                        if (rule->checkValueNotEmpty) {
                            if (exists && !val.empty()) {
                                conflictFound.store(true);
                                ConflictResult result{
                                    .pass = false,
                                    .conflicts = {{.code = st.code, .reason = rule->description, .index = idx}},
                                    .code = st.code,
                                    .reason = rule->description,
                                    .index = idx
                                };
                                callback(result);
                                return; // 找到第一个冲突，立即返回
                            }
                            continue;
                        }

                        // 获取期望值
                        Json::Value expected;
                        if (rule->extractor) expected = rule->extractor(options, logicalKey);
                        else if (options.isMember(logicalKey)) expected = options[logicalKey];
                        if (isEmptyExpected(expected)) continue;

                        // 评估约束条件
                        if (!evaluateConstraint(*rule, expected, exists ? Json::Value(val) : Json::Value::null)) {
                            conflictFound.store(true);
                            ConflictResult result{
                                .pass = false,
                                .conflicts = {{.code = st.code, .reason = rule->description, .index = idx}},
                                .code = st.code,
                                .reason = rule->description,
                                .index = idx
                            };
                            callback(result);
                            return; // 找到第一个冲突，立即返回
                        }
                    }
                }
                // 处理set类型的规则
                else if (conf.type == "set") {
                    Json::Value actual(Json::arrayValue);
                    if (setCache.count(redisKey)) {
                        for(const auto& v : setCache[redisKey]) actual.append(v);
                    }
                    for (const auto *rule : conf.rules) {
                        Json::Value expected;
                        if (rule->extractor) expected = rule->extractor(options, logicalKey);
                        else if (options.isMember(logicalKey)) expected = options[logicalKey];
                        if (isEmptyExpected(expected)) continue;
                        if (!evaluateConstraint(*rule, expected, actual)) {
                            conflictFound.store(true);
                            ConflictResult result{
                                .pass = false,
                                .conflicts = {{.code = st.code, .reason = rule->description, .index = idx}},
                                .code = st.code,
                                .reason = rule->description,
                                .index = idx
                            };
                            callback(result);
                            return; // 找到第一个冲突，立即返回
                        }
                    }
                }
                // 处理hash-fields类型的规则（天气规则）
                else if (conf.type == "hash-fields") {
                     if (!options.isMember(logicalKey) || !options[logicalKey].isObject()) continue;
                     const Json::Value &expectedObj = options[logicalKey];
                     for (const auto *rule : conf.rules) {
                         // 构造字段名：字段名_时间戳
                         const std::string fieldName = rule->jsonPath + "_" + std::to_string(st.wdTime);
                         if (hashCache.count(redisKey) && hashCache[redisKey].count(fieldName)) {
                             std::string valStr = hashCache[redisKey][fieldName];
                             const Json::Value expectedVal = expectedObj.isMember(rule->jsonPath) ? expectedObj[rule->jsonPath] : Json::Value();
                             if (isEmptyExpected(expectedVal)) continue;
                             if (!evaluateConstraint(*rule, expectedVal, valStr)) {
                                 conflictFound.store(true);
                                 ConflictResult result{
                                     .pass = false,
                                     .conflicts = {{.code = st.code, .reason = rule->description, .index = idx}},
                                     .code = st.code,
                                     .reason = rule->description,
                                     .index = idx
                                 };
                                 callback(result);
                                 return; // 找到第一个冲突，立即返回
                             }
                         }
                     }
                }
            }
        }

        // 如果遍历完所有网格都没有冲突，返回通过
        callback({.pass = true});
    }
};

} // namespace

// 将多段线路径转换为网格编码列表
// @param points 经纬高坐标点数组，每个点为[经度, 纬度, 高度]
// @param level 网格层级，决定网格的精度
// @param baseTile 基准瓦片信息
// @return 沿路径的网格编码列表
std::vector<std::string> polylineToCodes(const Json::Value &points, int level, const BaseTile &baseTile) {
    std::vector<std::string> codes;
    // 参数校验：points必须是数组且至少包含2个点
    if (!points.isArray() || points.size() < 2) return codes;
    // 将JSON格式的点转换为PointLBHd结构
    std::vector<PointLBHd> linePts;
    linePts.reserve(points.size());
    for (const auto &p : points) {
        if (!p.isArray() || p.size() < 3) continue;
        linePts.push_back({p[0].asDouble(), p[1].asDouble(), p[2].asDouble()});
    }
    if (linePts.size() < 2) return codes;
    // 调用DQG库将线路径转换为网格编码
    std::vector<std::string> lineCodes;
    try {
        lineCodes = lineToLocalCode(linePts, static_cast<uint8_t>(level), baseTile);
    } catch (const std::exception &e) {
        LOG_ERROR << "lineToLocalCode failed: " << e.what();
        return codes;
    }
    return lineCodes;
}

// 检查飞行路径上的网格冲突（核心功能函数）
// @param codes 路径上的网格编码列表
// @param startTimeMsOrSec 起始时间（毫秒或秒，北京时间 UTC+8）
// @param options 配置选项，包含各种检查规则的配置
// @param redis Redis客户端，用于查询网格的状态数据
// @param callback 检查完成后的回调函数，返回ConflictResult结果
void checkLineConflict(
    const std::vector<std::string> &codes,
    double startTimeMsOrSec,
    const Json::Value &options,
    const std::shared_ptr<RedisClient> &redis,
    std::function<void(ConflictResult)> callback)
{
    // === 基础校验 ===
    if (codes.empty()) { callback({.pass = true}); return; }
    if (!redis) { callback({.pass = false, .code = "", .reason = "redis_unavailable"}); return; }

    // 标准化起始时间，提取飞行速度
    const double startSec = normalizeStartSeconds(startTimeMsOrSec);
    const double speed = extractSpeed(options);

    // 创建异步上下文
    auto ctx = std::make_shared<AsyncContext>();
    ctx->callback = callback;
    ctx->options = options;
    ctx->hasDp = hasDpOption(options);
    ctx->activeRules = buildActiveRules(options);

    // 构建所有网格的时间戳信息
    try {
        ctx->stamps = buildStamps(codes, startSec, speed);
    } catch (...) {
        callback({.pass = false, .code = codes.front(), .reason = "invalid_code"});
        return;
    }

    // === 收集需要查询的Redis键 ===
    std::set<std::string> stringKeys;  // String类型键集合
    std::set<std::string> setKeys;     // Set类型键集合
    std::map<std::string, std::set<std::string>> hashKeys; // Hash类型键及字段集合

    // 遍历所有网格，收集需要查询的键
    for (const auto &st : ctx->stamps) {
        // 添加时间冲突检查键
        if (ctx->hasDp) setKeys.insert("dp_" + st.code);

        // 添加规则检查键
        for (const auto &entry : ctx->activeRules) {
            const std::string &prefix = entry.first;
            const auto &conf = entry.second;
            const int level = conf.level;

            // 天气规则的层级匹配
            if (conf.type == "hash-fields") {
                 std::string targetRule = prefix + "_" + std::to_string(level);
                 if (targetRule != st.wdRule) continue;
            }

            // 构造Redis键名
            const auto slice = st.code.substr(0, std::min(static_cast<size_t>(level), st.code.size()));

            // [新增] 键名重定向逻辑：如果前缀是 hlz，则查询 hl 的数据
            std::string queryPrefix = prefix;
            if (prefix == "hlz") {
                queryPrefix = "hl";
            }
            const std::string redisKey = queryPrefix + "_" + slice;

            // 根据规则类型分类收集键
            if (conf.type == "string") stringKeys.insert(redisKey);
            else if (conf.type == "set") setKeys.insert(redisKey);
            else if (conf.type == "hash-fields") {
                // 收集Hash类型的字段名：字段名_时间戳
                for (const auto *rule : conf.rules) {
                    hashKeys[redisKey].insert(rule->jsonPath + "_" + std::to_string(st.wdTime));
                }
            }
        }
    }

    //打印要查询的键
    LOG_INFO << "String Keys:";
    for (const auto &k : stringKeys) LOG_INFO << "  " << k;
    LOG_INFO << "Set Keys:";
    for (const auto &k : setKeys) LOG_INFO << "  " << k;
    LOG_INFO << "Hash Keys:";
    for (const auto &kv : hashKeys) {
        LOG_INFO << "  " << kv.first;
        for (const auto &f : kv.second) LOG_INFO << "    " << f;
    }

    // 计算需要发起的Redis请求数量
    int reqCount = 0;
    if (!stringKeys.empty()) reqCount++;  // MGET算1个请求
    reqCount += setKeys.size();           // 每个SMEMBERS算1个请求
    reqCount += hashKeys.size();          // 每个HMGET算1个请求

    ctx->pendingCount = reqCount;

    // 如果没有需要查询的键，直接完成检查
    if (reqCount == 0) {
        ctx->finish();
        return;
    }

    // === 执行Redis查询 ===

    // 执行MGET命令（批量获取String类型数据）
    if (!stringKeys.empty()) {
        std::string cmd = "MGET";
        std::vector<std::string> keysVec(stringKeys.begin(), stringKeys.end());
        for(const auto& k : keysVec) cmd += " " + k;

        redis->execCommandAsync(
            [ctx, keysVec](const RedisResult& r) {
                // 成功回调：解析查询结果并缓存
                if (r.type() == RedisResultType::kArray) {
                    auto arr = r.asArray();
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    for(size_t i=0; i<arr.size() && i<keysVec.size(); ++i) {
                        if (!arr[i].isNil()) ctx->stringCache[keysVec[i]] = arr[i].asString();
                    }
                }
                ctx->checkDone();
            },
            // 失败回调：仍然调用checkDone继续流程
            [ctx](const std::exception&){ ctx->checkDone(); },
            cmd
        );
    }

    // 执行SMEMBERS命令（获取Set类型数据）
    for(const auto& k : setKeys) {
        redis->execCommandAsync(
            [ctx, k](const RedisResult& r) {
                if (r.type() == RedisResultType::kArray) {
                    auto arr = r.asArray();
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    auto& vec = ctx->setCache[k];
                    for(const auto& item : arr) vec.push_back(item.asString());
                }
                ctx->checkDone();
            },
            [ctx](const std::exception&){ ctx->checkDone(); },
            "SMEMBERS %s", k.c_str()
        );
    }

    // 执行HMGET命令（批量获取Hash类型数据）
    for(const auto& kv : hashKeys) {
        std::string cmd = "HMGET " + kv.first;
        std::vector<std::string> fields(kv.second.begin(), kv.second.end());
        for(const auto& f : fields) cmd += " " + f;

        redis->execCommandAsync(
            [ctx, key = kv.first, fields](const RedisResult& r) {
                if (r.type() == RedisResultType::kArray) {
                    auto arr = r.asArray();
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    for(size_t i=0; i<arr.size() && i<fields.size(); ++i) {
                        if (!arr[i].isNil()) ctx->hashCache[key][fields[i]] = arr[i].asString();
                    }
                }
                ctx->checkDone();
            },
            [ctx](const std::exception&){ ctx->checkDone(); },
            cmd
        );
    }
}

// 检查飞行路径上的网格冲突（遇到第一个冲突即返回版本）
// @param codes 路径上的网格编码列表
// @param startTimeMsOrSec 起始时间（毫秒或秒，北京时间 UTC+8）
// @param options 配置选项
// @param redis Redis客户端
// @param callback 回调函数
void checkLineConflictFirst(
    const std::vector<std::string> &codes,
    double startTimeMsOrSec,
    const Json::Value &options,
    const std::shared_ptr<RedisClient> &redis,
    std::function<void(ConflictResult)> callback)
{
    // === 基础校验 ===
    if (codes.empty()) { callback({.pass = true}); return; }
    if (!redis) { callback({.pass = false, .code = "", .reason = "redis_unavailable"}); return; }

    // 标准化起始时间，提取飞行速度
    const double startSec = normalizeStartSeconds(startTimeMsOrSec);
    const double speed = extractSpeed(options);

    // 创建异步上下文（第一个冲突版本）
    auto ctx = std::make_shared<AsyncContextFirst>();
    ctx->callback = callback;
    ctx->options = options;
    ctx->hasDp = hasDpOption(options);
    ctx->activeRules = buildActiveRules(options);

    // 构建所有网格的时间戳信息
    try {
        ctx->stamps = buildStamps(codes, startSec, speed);
    } catch (...) {
        callback({.pass = false, .code = codes.front(), .reason = "invalid_code"});
        return;
    }

    // === 收集需要查询的Redis键 ===
    std::set<std::string> stringKeys;  // String类型键集合
    std::set<std::string> setKeys;     // Set类型键集合
    std::map<std::string, std::set<std::string>> hashKeys; // Hash类型键及字段集合

    // 遍历所有网格，收集需要查询的键
    for (const auto &st : ctx->stamps) {
        // 添加时间冲突检查键
        if (ctx->hasDp) setKeys.insert("dp_" + st.code);

        // 添加规则检查键
        for (const auto &entry : ctx->activeRules) {
            const std::string &prefix = entry.first;
            const auto &conf = entry.second;
            const int level = conf.level;

            // 天气规则的层级匹配
            if (conf.type == "hash-fields") {
                 std::string targetRule = prefix + "_" + std::to_string(level);
                 if (targetRule != st.wdRule) continue;
            }

            // 构造Redis键名
            const auto slice = st.code.substr(0, std::min(static_cast<size_t>(level), st.code.size()));

            // [新增] 键名重定向逻辑：如果前缀是 hlz，则查询 hl 的数据
            std::string queryPrefix = prefix;
            if (prefix == "hlz") {
                queryPrefix = "hl";
            }
            const std::string redisKey = queryPrefix + "_" + slice;

            // 根据规则类型分类收集键
            if (conf.type == "string") stringKeys.insert(redisKey);
            else if (conf.type == "set") setKeys.insert(redisKey);
            else if (conf.type == "hash-fields") {
                // 收集Hash类型的字段名：字段名_时间戳
                for (const auto *rule : conf.rules) {
                    hashKeys[redisKey].insert(rule->jsonPath + "_" + std::to_string(st.wdTime));
                }
            }
        }
    }

    // 计算需要发起的Redis请求数量
    int reqCount = 0;
    if (!stringKeys.empty()) reqCount++;  // MGET算1个请求
    reqCount += setKeys.size();           // 每个SMEMBERS算1个请求
    reqCount += hashKeys.size();          // 每个HMGET算1个请求

    ctx->pendingCount = reqCount;

    // 如果没有需要查询的键，直接完成检查
    if (reqCount == 0) {
        ctx->finish();
        return;
    }

    // === 执行Redis查询 ===

    // 执行MGET命令（批量获取String类型数据）
    if (!stringKeys.empty()) {
        std::string cmd = "MGET";
        std::vector<std::string> keysVec(stringKeys.begin(), stringKeys.end());
        for(const auto& k : keysVec) cmd += " " + k;

        redis->execCommandAsync(
            [ctx, keysVec](const RedisResult& r) {
                // 如果已找到冲突，忽略后续结果
                if (ctx->conflictFound.load()) return;

                // 成功回调：解析查询结果并缓存
                if (r.type() == RedisResultType::kArray) {
                    auto arr = r.asArray();
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    for(size_t i=0; i<arr.size() && i<keysVec.size(); ++i) {
                        if (!arr[i].isNil()) ctx->stringCache[keysVec[i]] = arr[i].asString();
                    }
                }
                ctx->checkDone();
            },
            // 失败回调：仍然调用checkDone继续流程
            [ctx](const std::exception&){
                if (!ctx->conflictFound.load()) ctx->checkDone();
            },
            cmd
        );
    }

    // 执行SMEMBERS命令（获取Set类型数据）
    for(const auto& k : setKeys) {
        redis->execCommandAsync(
            [ctx, k](const RedisResult& r) {
                // 如果已找到冲突，忽略后续结果
                if (ctx->conflictFound.load()) return;

                if (r.type() == RedisResultType::kArray) {
                    auto arr = r.asArray();
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    auto& vec = ctx->setCache[k];
                    for(const auto& item : arr) vec.push_back(item.asString());
                }
                ctx->checkDone();
            },
            [ctx](const std::exception&){
                if (!ctx->conflictFound.load()) ctx->checkDone();
            },
            "SMEMBERS %s", k.c_str()
        );
    }

    // 执行HMGET命令（批量获取Hash类型数据）
    for(const auto& kv : hashKeys) {
        std::string cmd = "HMGET " + kv.first;
        std::vector<std::string> fields(kv.second.begin(), kv.second.end());
        for(const auto& f : fields) cmd += " " + f;

        redis->execCommandAsync(
            [ctx, key = kv.first, fields](const RedisResult& r) {
                // 如果已找到冲突，忽略后续结果
                if (ctx->conflictFound.load()) return;

                if (r.type() == RedisResultType::kArray) {
                    auto arr = r.asArray();
                    std::lock_guard<std::mutex> lk(ctx->mutex);
                    for(size_t i=0; i<arr.size() && i<fields.size(); ++i) {
                        if (!arr[i].isNil()) ctx->hashCache[key][fields[i]] = arr[i].asString();
                    }
                }
                ctx->checkDone();
            },
            [ctx](const std::exception&){
                if (!ctx->conflictFound.load()) ctx->checkDone();
            },
            cmd
        );
    }
}

} // namespace plancheck