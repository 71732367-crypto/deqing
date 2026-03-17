#include "GridEvaluator.h"
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <algorithm>

using namespace drogon;
using namespace std;

namespace api {
namespace airRoute {

// === 辅助工具函数 ===

/**
 * @brief 将 Json::Value 转换为 double 类型数值
 * @param v Json::Value 对象，可以是数值或字符串类型
 * @return 转换后的 double 值，如果转换失败则返回 NAN
 */
static double toNumber(const Json::Value& v) {
    if (v.isNumeric()) return v.asDouble();
    if (v.isString()) {
        try { return std::stod(v.asString()); } catch(...) { return NAN; }
    }
    return NAN;
}

/**
 * @brief 按指定分隔符分割字符串
 * @param text 要分割的原始字符串
 * @param sep 分隔符
 * @return 分割后的字符串数组，自动过滤空字符串
 */
static std::vector<std::string> split(const std::string &text, char sep) {
    std::vector<std::string> parts;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, sep)) {
        if (!item.empty()) parts.push_back(item);
    }
    return parts;
}

/**
 * @brief 静态规则定义结构体
 * 用于定义校验规则的元数据
 */
struct RuleDef {
    string prefix;                    // 规则前缀，如 "hl"、"wdd"、"wdh" 等
    string type;                      // 规则类型："string"(字符串)、"set"(集合)、"hash-fields"(哈希字段)
    string op;                        // 操作符："<="、">="、"<"、">" 等
    string jsonPath;                  // JSON 路径，用于定位哈希中的字段名（如 "visibility"、"humidity"）
    bool checkValueMustExist;         // 是否必须存在该值（用于字符串类型）
    bool checkValueNotEmpty;          // 是否必须非空（用于字符串类型）
    string description;               // 规则描述，用于显示失败原因
};

/**
 * @brief 获取预定义的规则列表
 * @return 规则定义数组的常量引用
 */
static const vector<RuleDef>& getRuleDefs() {
    static const vector<RuleDef> rules = {
        // === String 类型规则（简单存在性检查）===
        // 格式：{prefix, type, op, jsonPath, checkValueMustExist, checkValueNotEmpty, description}

        // hl - 航路校验：必须存在于航路规划中
        // 逻辑：checkValueMustExist=true (Redis 中必须有值才能通过)
        {"hl", "string", "", "", true, false, "航路校验：必须存在于航路规划中"},

        // [新增] hlz - 航路避让：如果存在数据则表示该区域为航路，不可通行（用于非航路飞行避让）
        // 逻辑：checkValueNotEmpty=true (Redis 中如果有值，则校验失败；无值才通过)
        // 注意：实际查询时，代码会将 hlz 前缀映射为 hl 键进行查询
        {"hlz", "string", "", "", false, true, "航路避让：当前区域存在航路，不可穿越"},

        // fx - 人口密集区域：如果存在数据则表示该区域无法通行
        {"fx", "string", "", "", false, true, "人口密集区域无法通行"},

        // gd - 三维实景障碍物：如果存在数据则表示有障碍物冲突
        {"gd", "string", "", "", false, true, "存在三维实景障碍物冲突"},

        // dt - 无人机实时占用：如果存在数据则表示有实时占用冲突
        {"dt", "string", "", "", false, true, "存在无人机实时占用冲突"},

        // dz - 电子围栏：如果存在数据则表示有电子围栏冲突
        {"dz", "string", "", "", false, true, "存在电子围栏冲突"},

        // za - 障碍物：如果存在数据则表示有障碍物冲突
        {"za", "string", "", "", false, true, "存在障碍物冲突"},

        // === Set 类型规则（空域检查）===
        // ad - 空域类型占用冲突：检查空域是否包含不允许的类型
        {"ad", "set", "containsAny", "", false, false, "空域类型占用冲突"},

        // === Hash 类型规则（天气 - WDD 日级天气数据）===
        // 天气数据存储在 Redis Hash 中，每个字段包含不同时间点的数据
        // 字段格式：{fieldName}_{timestamp}

        // 能见度检查：实际能见度 <= 标准值时违规（能见度低于阈值）
        {"wdd", "hash-fields", "<=", "visibility", false, false, "能见度小于标准值"},

        // 湿度检查：实际湿度 >= 标准值时违规（湿度超过阈值）
        {"wdd", "hash-fields", ">=", "humidity",   false, false, "湿度大于标准值"},

        // 最高温度检查：实际温度 >= 标准值时违规（温度超过阈值）
        {"wdd", "hash-fields", ">=", "tem1",       false, false, "最高温度大于标准值"},

        // 最低温度检查：实际温度 <= 标准值时违规（温度低于阈值）
        {"wdd", "hash-fields", "<=", "tem2",       false, false, "最低温度小于标准值"},

        // 气压检查：实际气压 >= 标准值时违规（气压超过阈值）
        {"wdd", "hash-fields", ">=", "pressure",   false, false, "压力大于标准值"},

        // === Hash 类型规则（天气 - WDH 小时级天气数据）===
        // 小时级天气数据比日级数据精度更高，检查逻辑相同

        // 能见度检查
        {"wdh", "hash-fields", "<=", "visibility", false, false, "能见度小于标准值"},

        // 湿度检查
        {"wdh", "hash-fields", ">=", "humidity",   false, false, "湿度大于标准值"},

        // 温度检查
        {"wdh", "hash-fields", "<=", "tem",        false, false, "温度小于标准值"},

        // 风速检查：实际风速 >= 标准值时违规（风速超过阈值）
        {"wdh", "hash-fields", ">=", "windSpeed",  false, false, "风速大于标准值"},

        // 降雨量检查：实际降雨量 > 标准值时违规（降雨超过阈值）
        {"wdh", "hash-fields", ">",  "rainPcpn",   false, false, "降雨量大于标准值"}
    };
    return rules;
}

/**
 * @brief 创建 GridEvaluator 对象的工厂方法
 * @param options 配置选项，包含要激活的规则及其阈值
 * @return shared_ptr 智能指针指向创建的 GridEvaluator 实例
 */
std::shared_ptr<GridEvaluator> GridEvaluator::create(const Json::Value& options) {
    return std::make_shared<GridEvaluator>(options);
}

/**
 * @brief GridEvaluator 构造函数
 * @param options 配置选项
 */
GridEvaluator::GridEvaluator(const Json::Value& options) {
    // 验证输入参数
    if (!options.isObject()) {
        LOG_WARN << "[GridEvaluator] Options is not an object";
        return;
    }

    LOG_INFO << "[GridEvaluator] Initializing with options: " << options.toStyledString();

    // 遍历配置选项中的所有键
    for (const auto& key : options.getMemberNames()) {
        // 检查是否为 DP（时空冲突）规则
        if (key.substr(0, 3) == "dp_") {
            hasDp_ = true;
            LOG_INFO << "[GridEvaluator] Activated DP rule: " << key;
            continue;
        }

        // 解析键名，格式应为：{prefix}_{level}
        size_t underscore = key.find('_');
        if (underscore == string::npos) continue;

        string prefix = key.substr(0, underscore);      // 提取前缀
        string levelStr = key.substr(underscore + 1);   // 提取级别字符串
        int level = 0;
        try { level = stoi(levelStr); } catch(...) { continue; } // 转换级别为整数

        // 获取或创建该前缀对应的规则组
        auto& group = activeRulesMap_[prefix];
        group.level = level;

        // 标记是否匹配到预定义规则
        bool matched = false;
        for (const auto& def : getRuleDefs()) {
            if (def.prefix != prefix) continue;
            matched = true;

            // 创建规则元数据对象
            RuleMeta meta;
            meta.prefix = def.prefix;
            meta.type = def.type;
            meta.op = def.op;
            meta.jsonPath = def.jsonPath;
            meta.checkValueMustExist = def.checkValueMustExist;
            meta.checkValueNotEmpty = def.checkValueNotEmpty;
            meta.description = def.description;

            // 根据规则类型进行不同的处理
            if (def.type == "hash-fields") {
                if (options[key].isObject() && options[key].isMember(def.jsonPath)) {
                    meta.expectedValue = options[key][def.jsonPath];
                    group.type = "hash-fields";
                    group.rules.push_back(meta);

                    bool exists = false;
                    for(const auto& f : group.requestedFields) if(f == def.jsonPath) exists = true;
                    if(!exists) group.requestedFields.push_back(def.jsonPath);
                }
            } else if (def.type == "set") {
                Json::Value expected(Json::arrayValue);
                if (options.isMember(key)) {
                    auto val = options[key];
                    if (val.isString()) {
                        auto parts = split(val.asString(), ',');
                        for(const auto& p : parts) expected.append(split(p, ':')[0]);
                    } else if (val.isArray()) {
                        for(const auto& v : val) if(v.isString()) expected.append(split(v.asString(), ':')[0]);
                    }
                }
                if (!expected.empty()) {
                    meta.expectedValue = expected;
                    group.type = "set";
                    group.rules.push_back(meta);
                }
            } else {
                // === String 类型（简单存在性检查）===
                group.type = "string";
                group.rules.push_back(meta);
            }
        }

        // 记录激活的规则组信息
        if (matched) {
            LOG_INFO << "[GridEvaluator] Activated Rule Group: prefix=" << prefix
                     << ", level=" << level
                     << ", type=" << group.type
                     << ", rules_count=" << group.rules.size();
        } else {
            LOG_WARN << "[GridEvaluator] No rule definition found for key: " << key << " (prefix: " << prefix << ")";
        }
    }
}

/**
 * @brief 评估单个约束规则
 * @param rule 规则元数据
 * @param actual 实际值
 * @return 返回 true 表示约束满足（通过校验），返回 false 表示违反约束
 */
bool GridEvaluator::evaluateConstraint(const RuleMeta& rule, const Json::Value& actual) {
    // === 1. 存在性/非空检查 ===
    // 如果要求值必须存在，则检查实际值是否非空 (hl 规则使用)
    if (rule.checkValueMustExist) {
        return !(actual.isNull() || (actual.isString() && actual.asString().empty()));
    }
    // 如果要求值为空，则检查实际值是否为空 (hlz, fx, gd 等规则使用)
    // hlz 规则：actual 非空 -> 返回 false (不可通行)
    if (rule.checkValueNotEmpty) {
        return (actual.isNull() || (actual.isString() && actual.asString().empty()));
    }

    // === 2. Set 类型 containsAny 检查 ===
    if (rule.type == "set" && rule.op == "containsAny") {
        std::unordered_set<std::string> actSet;
        if (actual.isArray()) {
            for (const auto& v : actual) {
                string s = v.asString();
                actSet.insert(split(s, ':')[0]);
            }
        }
        if (rule.expectedValue.isArray()) {
            for (const auto& e : rule.expectedValue) {
                if (actSet.count(e.asString())) return false; // 发现冲突，返回 false
            }
        }
        return true; // 无冲突，返回 true
    }

    // === 3. 数值比较 ===
    double actNum = toNumber(actual);
    double expNum = toNumber(rule.expectedValue);

    if (std::isnan(actNum) || std::isnan(expNum)) return true;

    if (rule.jsonPath == "visibility" && rule.op == "<=") {
        return actNum >= expNum;
    }

    if (rule.op == "<=") return actNum <= expNum;
    if (rule.op == ">=") return actNum >= expNum;
    if (rule.op == "<")  return actNum < expNum;
    if (rule.op == ">")  return actNum > expNum;
    if (rule.op == "==") return std::fabs(actNum - expNum) < 1e-9;
    if (rule.op == "!=") return std::fabs(actNum - expNum) > 1e-9;

    return true;
}

/**
 * @brief 异步操作上下文结构体
 */
struct AsyncContext {
    std::mutex mutex;
    std::atomic<int> pendingCount{0};
    bool isDone = false;

    std::vector<std::pair<std::string, Json::Value>> results;

    GridEvaluator::CandidatesCallback callback;
    std::shared_ptr<GridEvaluator> evaluator;
    std::vector<CandidateInfo> candidates;

    void checkDone() {
        int left = --pendingCount;
        if (left <= 0) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (isDone) return;
                isDone = true;
            }
            finish();
        }
    }

    /**
     * @brief 完成所有 Redis 查询后的处理逻辑
     */
    void finish() {
        // === 1. 更新缓存 ===
        {
            std::lock_guard<std::mutex> lock(evaluator->cacheMutex_);
            for (const auto& p : results) {
                evaluator->redisCache_[p.first] = p.second;
            }
        }

        // === 2. 最终评估，生成 Map<Code, CheckResult> ===
        std::unordered_map<std::string, GridEvaluator::CheckResult> finalResults;

        {
            std::lock_guard<std::mutex> lock(evaluator->cacheMutex_);

            // [修复] 如果没有激活任何规则（无约束条件），直接返回所有候选网格为通过状态
            if (evaluator->activeRulesMap_.empty() && !evaluator->hasDp_) {
                for (const auto& cand : candidates) {
                    finalResults[cand.code] = {true, ""};
                }
                callback(finalResults);
                return;
            }

            // 遍历每个候选网格进行评估
            for (const auto& cand : candidates) {
                bool candPass = true;
                std::string failReason;

                // 遍历所有激活的规则组
                for (const auto& [prefix, group] : evaluator->activeRulesMap_) {

                    // [时间规则过滤] 如果不检查时间规则，跳过相关规则
                    if (!cand.checkTimeRules && (prefix == "dt" || prefix == "wdd" || prefix == "wdh")) {
                        continue;
                    }

                    if (group.type == "hash-fields") {
                        string ruleKey = prefix + "_" + to_string(group.level);
                        if (ruleKey != cand.wdRule) continue;
                    }

                    // 截取网格编码
                    string sliceCode = cand.code;
                    if (cand.code.length() > (size_t)group.level) {
                        sliceCode = cand.code.substr(0, group.level);
                    }

                    // [关键修改] 键名重定向逻辑
                    // 如果前缀是 hlz (航路避让)，则查询 hl (航路) 的数据
                    // 这样可以复用 hl 的数据源，但应用 hlz 的校验逻辑 (checkValueNotEmpty)
                    string queryPrefix = prefix;
                    if (prefix == "hlz") {
                        queryPrefix = "hl";
                    }

                    // 构造 Redis 键
                    string redisKeyBase = queryPrefix + "_" + sliceCode;

                    // === 评估逻辑 ===

                    if (group.type == "string") {
                        // === String 类型规则 ===
                        if (evaluator->redisCache_.count(redisKeyBase)) {
                            // 缓存中有值（可能是 null，也可能是字符串）
                            auto val = evaluator->redisCache_[redisKeyBase];
                            for (const auto& rule : group.rules) {
                                // hl 规则 (MustExist): val 非空则 Pass
                                // hlz 规则 (NotEmpty): val 非空则 Fail (障碍)
                                if (!evaluator->evaluateConstraint(rule, val)) {
                                    candPass = false; failReason = rule.description; break;
                                }
                            }
                        } else {
                            // 缓存中没有该键（理论上不会发生，因为 checkCandidates 阶段已经请求过了）
                            // 传入 Json::nullValue 进行检查
                            // hl 规则 (MustExist): 传入 null -> 返回 false (Fail)
                            // hlz 规则 (NotEmpty): 传入 null -> 返回 true (Pass)
                            for (const auto& rule : group.rules) {
                                if (!evaluator->evaluateConstraint(rule, Json::Value::null)) {
                                    candPass = false; failReason = rule.description; break;
                                }
                            }
                        }
                    }
                    else if (group.type == "set") {
                        if (evaluator->redisCache_.count(redisKeyBase)) {
                            auto val = evaluator->redisCache_[redisKeyBase];
                            for (const auto& rule : group.rules) {
                                if (!evaluator->evaluateConstraint(rule, val)) {
                                    candPass = false; failReason = rule.description; break;
                                }
                            }
                        }
                    }
                    else if (group.type == "hash-fields") {
                        if (evaluator->redisCache_.count(redisKeyBase)) {
                            auto hashVal = evaluator->redisCache_[redisKeyBase];
                            if (hashVal.isObject()) {
                                for (const auto& rule : group.rules) {
                                    string fieldName = rule.jsonPath + "_" + to_string(cand.wdTime);
                                    if (hashVal.isMember(fieldName)) {
                                        if (!evaluator->evaluateConstraint(rule, hashVal[fieldName])) {
                                            candPass = false; failReason = rule.description; break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (!candPass) break;
                }

                // === DP（时空冲突）检查 ===
                if (candPass && evaluator->hasDp_ && cand.checkTimeRules) {
                    string dpKey = "dp_" + cand.code;
                    if (evaluator->redisCache_.count(dpKey)) {
                        auto val = evaluator->redisCache_[dpKey];
                        if (val.isArray()) {
                            for (const auto& rangeStr : val) {
                                string s = rangeStr.asString();
                                auto parts = split(s, ':');
                                if (parts.size() >= 2) {
                                    double start = stod(parts[0]);
                                    double end = stod(parts[1]);
                                    if (cand.arrivalTime >= start && cand.arrivalTime <= end) {
                                        candPass = false;
                                        failReason = "存在时空冲突";
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                finalResults[cand.code] = {candPass, failReason};
            }
        }
        callback(finalResults);
    }
};

/**
 * @brief 异步检查多个候选网格的约束条件
 */
void GridEvaluator::checkCandidates(
    const std::vector<CandidateInfo>& candidates,
    CandidatesCallback callback
) {
    if (candidates.empty()) { callback({}); return; }

    auto redis = app().getRedisClient();
    if (!redis) { callback( {} ); return; }

    auto ctx = std::make_shared<AsyncContext>();
    ctx->callback = callback;
    ctx->evaluator = shared_from_this();
    ctx->candidates = candidates;

    vector<string> stringKeys;
    vector<string> setKeys;
    unordered_map<string, vector<string>> hashKeys;

    // === 第一步：收集需要查询的 Redis 键 ===
    {
        lock_guard<mutex> lock(cacheMutex_);
        for (const auto& cand : candidates) {

            // 遍历所有激活的规则组
            for (const auto& [prefix, group] : activeRulesMap_) {

                if (!cand.checkTimeRules && (prefix == "dt" || prefix == "wdd" || prefix == "wdh")) {
                    continue;
                }

                if (group.type == "hash-fields") {
                    string ruleKey = prefix + "_" + to_string(group.level);
                    if (ruleKey != cand.wdRule) continue;
                }

                string sliceCode = cand.code;
                if (cand.code.length() > (size_t)group.level) {
                    sliceCode = cand.code.substr(0, group.level);
                }

                // [关键修改] 生成 Redis 查询键时，拦截 hlz 前缀，重定向到 hl
                // 当规则是 hlz (航路避让) 时，实际上需要查询 hl (航路) 数据
                string queryPrefix = prefix;
                if (prefix == "hlz") {
                    queryPrefix = "hl";
                }

                string redisKey = queryPrefix + "_" + sliceCode;

                if (redisCache_.count(redisKey)) continue;

                if (group.type == "string") {
                    stringKeys.push_back(redisKey);
                } else if (group.type == "set") {
                    setKeys.push_back(redisKey);
                } else if (group.type == "hash-fields") {
                    for (const auto& field : group.requestedFields) {
                        string fullField = field + "_" + to_string(cand.wdTime);
                        hashKeys[redisKey].push_back(fullField);
                    }
                }
            }

            // DP 规则查询
            if (hasDp_ && cand.checkTimeRules) {
                string dpKey = "dp_" + cand.code;
                if (!redisCache_.count(dpKey)) {
                    setKeys.push_back(dpKey);
                }
            }
        }
    }

    // === 第二步：去重处理 ===
    sort(stringKeys.begin(), stringKeys.end());
    stringKeys.erase(unique(stringKeys.begin(), stringKeys.end()), stringKeys.end());
    sort(setKeys.begin(), setKeys.end());
    setKeys.erase(unique(setKeys.begin(), setKeys.end()), setKeys.end());

    for (auto& kv : hashKeys) {
        sort(kv.second.begin(), kv.second.end());
        kv.second.erase(unique(kv.second.begin(), kv.second.end()), kv.second.end());
    }

    // 日志打印部分省略...

    int reqCount = 0;
    if (!stringKeys.empty()) reqCount++;
    reqCount += setKeys.size();
    reqCount += hashKeys.size();

    ctx->pendingCount = reqCount;

    if (reqCount == 0) {
        ctx->finish();
        return;
    }

    // === 第三步：发送 Redis 异步查询命令 ===

    if (!stringKeys.empty()) {
        std::string mgetCmd = "MGET";
        for (const auto& k : stringKeys) {
            mgetCmd.append(" ").append(k);
        }

        redis->execCommandAsync(
            [ctx, stringKeys](const drogon::nosql::RedisResult& r) {
                if (r.type() == drogon::nosql::RedisResultType::kArray) {
                    auto arr = r.asArray();
                    lock_guard<mutex> lk(ctx->mutex);
                    // 保存结果...
                    for (size_t i=0; i<arr.size() && i<stringKeys.size(); ++i) {
                        string val = "(nil)";
                        Json::Value jVal(Json::nullValue);
                        if (!arr[i].isNil()) {
                            val = arr[i].asString();
                            jVal = val;
                        }
                        ctx->results.push_back({stringKeys[i], jVal});
                    }
                }
                ctx->checkDone();
            },
            [ctx](const std::exception& e){
                LOG_ERROR << "[GridEvaluator] MGET Exception: " << e.what();
                ctx->checkDone();
            },
            mgetCmd
        );
    }

    for (const string& key : setKeys) {
        redis->execCommandAsync(
            [ctx, key](const drogon::nosql::RedisResult& r) {
                Json::Value arr(Json::arrayValue);
                if (r.type() == drogon::nosql::RedisResultType::kArray) {
                    for (const auto& item : r.asArray()) {
                        arr.append(item.asString());
                    }
                }
                {
                    lock_guard<mutex> lk(ctx->mutex);
                    ctx->results.push_back({key, arr});
                }
                ctx->checkDone();
            },
            [ctx](const std::exception&){ ctx->checkDone(); },
            "SMEMBERS %s", key.c_str()
        );
    }

    for (const auto& kv : hashKeys) {
        string key = kv.first;
        vector<string> fields = kv.second;
        if (fields.empty()) { ctx->checkDone(); continue; }

        std::string hmgetCmd = "HMGET " + key;
        for (const auto& f : fields) hmgetCmd.append(" ").append(f);

        redis->execCommandAsync(
            [ctx, key, fields](const drogon::nosql::RedisResult& r) {
                Json::Value obj(Json::objectValue);
                if (r.type() == drogon::nosql::RedisResultType::kArray) {
                    auto arr = r.asArray();
                    for(size_t i=0; i<arr.size() && i<fields.size(); ++i) {
                        if (!arr[i].isNil()) obj[fields[i]] = arr[i].asString();
                    }
                }
                {
                    lock_guard<mutex> lk(ctx->mutex);
                    ctx->results.push_back({key, obj});
                }
                ctx->checkDone();
            },
            [ctx](const std::exception&){ ctx->checkDone(); },
            hmgetCmd
        );
    }
}

} // namespace airRoute
} // namespace api