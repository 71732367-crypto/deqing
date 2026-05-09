#include "GridEvaluator.h"
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <algorithm>
#include<dqg/DQG3DBasic.h>
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


    // === 新增：数学区间解析器 ===
    /**
     * @brief 解析类似 "(5,10]" 的区间字符串，并判断数值是否在区间内
     */
    static bool isValueInRange(const std::string& rangeStr, double val) {
    if (rangeStr.empty() || rangeStr.length() < 3) return false;

    // 1. 获取开闭符号
    char leftOp = rangeStr.front(); // '[' 或 '('
    char rightOp = rangeStr.back(); // ']' 或 ')'

    // 2. 提取数字部分
    std::string inner = rangeStr.substr(1, rangeStr.length() - 2);
    size_t commaPos = inner.find(',');
    if (commaPos == std::string::npos) return false;

    // 3. 转换数字
    try {
        double minVal = std::stod(inner.substr(0, commaPos));
        double maxVal = std::stod(inner.substr(commaPos + 1));

        // 4. 逻辑判断
        bool passLeft = (leftOp == '[') ? (val >= minVal) : (val > minVal);
        bool passRight = (rightOp == ']') ? (val <= maxVal) : (val < maxVal);

        return passLeft && passRight;
    } catch (...) {
        LOG_ERROR << "[GridEvaluator] 区间解析失败: " << rangeStr;
        return false; // 解析异常当作不匹配
    }
}

    // === 新增：动态代价提取器 ===
    /**
     * @brief 从前端传入的 JSON 规则配置中，提取当前数值对应的归一化代价 (0.0 ~ 1.0)
     */
    static double extractDynamicCost(const Json::Value& ruleConfig, double actualVal) {
    // 兼容检查：确保前端传的是新版的对象格式
    if (!ruleConfig.isObject() || !ruleConfig.isMember("value")) {
        return 0.0;
    }

    const auto& intervals = ruleConfig["value"];
    if (intervals.isArray()) {
        for (const auto& item : intervals) {
            if (item.isMember("range") && item.isMember("cost")) {
                std::string rangeStr = item["range"].asString();
                if (isValueInRange(rangeStr, actualVal)) {
                    return item["cost"].asDouble(); // 找到匹配区间，返回对应 cost
                }
            }
        }
    }

    // 如果都不匹配，返回配置的 defaultValue，如果没配则默认满额代价 1.0
    return ruleConfig.get("defaultValue", 1.0).asDouble();
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
        // === String 类型规则（简单存在性/非空检查）===
        // 格式：{prefix, type, op, jsonPath, checkValueMustExist, checkValueNotEmpty, description}

        // 航路与禁飞区等刚性拦截规则
        {"hl",  "string", "", "", true,  false, "航路校验：必须存在于航路规划中"},
        {"hlz", "string", "", "", false, true,  "航路避让：当前区域存在航路，不可穿越"},
        {"fx",  "string", "", "", false, true,  "人口密集区域无法通行"},
        {"gd",  "string", "", "", false, true,  "存在三维实景障碍物冲突"},
        {"dt",  "string", "", "", false, true,  "存在无人机实时占用冲突"},
        {"dz",  "string", "", "", false, true,  "存在电子围栏冲突"},
        {"za",  "string", "", "", false, true,  "存在障碍物冲突"},

        // 离散型环境评估（提取代价）
        {"dc",  "string", "", "", false, false, "电磁环境评估"},

        // === Set 类型规则（空域检查）===
        {"ad",  "set", "containsAny", "", false, false, "空域类型占用冲突"},

        // === Hash 类型规则（天气 - WDD 日级天气数据）===
        {"wdd", "hash-fields", "", "visibility", false, false, "日级能见度评估"},
        {"wdd", "hash-fields", "", "humidity",   false, false, "日级湿度评估"},
        {"wdd", "hash-fields", "", "tem1",       false, false, "日级最高温度评估"},
        {"wdd", "hash-fields", "", "tem2",       false, false, "日级最低温度评估"},
        {"wdd", "hash-fields", "", "pressure",   false, false, "日级气压评估"},

        // === Hash 类型规则（天气 - WDH 小时级天气数据）===
        {"wdh", "hash-fields", "", "visibility", false, false, "小时级能见度评估"},
        {"wdh", "hash-fields", "", "humidity",   false, false, "小时级湿度评估"},
        {"wdh", "hash-fields", "", "tem",        false, false, "小时级温度评估"},
        {"wdh", "hash-fields", "", "windSpeed",  false, false, "小时级风速评估"},
        {"wdh", "hash-fields", "", "rainPcpn",   false, false, "小时级降雨量评估"},

  {"dc",  "string", "", "", false, false, "电磁环境评估"},
  {"tx",  "string", "", "", false, false, "通信信号评估"},
  {"dh",  "string", "", "", false, false, "导航信号评估"},
  {"jk",  "string", "", "", false, false, "监视信号评估"},

  // === 离散风险评估与隐私区 ===

  {"fxq", "json-string", "", "workday_low_risk_time",   false, false, "工作日低风险时间"},
  {"fxq", "json-string", "", "workday_mid_risk_time",   false, false, "工作日中风险时间"},
  {"fxq", "json-string", "", "workday_high_risk_time",  false, false, "工作日高风险时间"},
  {"fxq", "json-string", "", "weekend_low_risk_time",   false, false, "周末低风险时间"},
  {"fxq", "json-string", "", "weekend_mid_risk_time",   false, false, "周末中风险时间"},
  {"fxq", "json-string", "", "weekend_high_risk_time",  false, false, "周末高风险时间"},
  {"fxq", "json-string", "", "holiday_low_risk_time",   false, false, "节假日低风险时间"},
  {"fxq", "json-string", "", "holiday_mid_risk_time",   false, false, "节假日中风险时间"},
  {"fxq", "json-string", "", "holiday_high_risk_time",  false, false, "节假日高风险时间"},
  //-------------------暂无数据------------------------
        {"privacy", "hash-fields", "", "residential_area",    false, false, "隐私区域评估"}
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
            if (def.type == "hash-fields" || def.type == "json-string") {
                if (options[key].isObject() && options[key].isMember(def.jsonPath)) {
                    meta.expectedValue = options[key][def.jsonPath];
                    group.type = def.type; // 【关键：动态保留它原本的类型，不要写死】
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
                // ad_ 管制空域：即使前端未传空域类型，也要激活规则（存在数据即拦截）
                meta.expectedValue = expected;  // 可能为空数组，评估时特殊处理
                group.type = "set";
                group.rules.push_back(meta);
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
            // ad_ 管制空域：期望值为空数组时（前端未传空域类型），Redis 有数据即冲突
            if (rule.expectedValue.empty() && rule.prefix == "ad" && !actSet.empty()) {
                return false; // 存在管制空域数据，直接拦截
            }
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
    static double extractDiscreteCost(const Json::Value& ruleConfig, const std::string& actualVal) {
        if (!ruleConfig.isObject() || !ruleConfig.isMember("value")) return 0.0;

        const auto& mappings = ruleConfig["value"];
        if (mappings.isArray()) {
            for (const auto& item : mappings) {
                if (item.isMember("match") && item.isMember("cost")) {
                    if (item["match"].asString() == actualVal) {
                        return item["cost"].asDouble(); // 找到匹配项，返回动态配置的代价
                    }
                }
            }
        }
        return ruleConfig.get("defaultValue", 1.0).asDouble();
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
                    GridEvaluator::CheckResult res;
                    res.pass = true;
                    finalResults[cand.code] = res;
                }
                callback(finalResults);
                return;
            }

            // 遍历每个候选网格进行评估
            for (const auto& cand : candidates) {
                bool candPass = true;
                std::string failReason;

                // 初始化当前网格的归一化惩罚代价（0.0 代表无代价，1.0 代表满额代价）
                double curComm = 0.0, curNav = 0.0, curSurv = 0.0;
                double curWind = 0.0, curRain = 0.0, curVis = 0.0;
                double curTemp = 0.0, curHum = 0.0, curPress = 0.0;
                double curEm = 0.0, curRisk = 0.0, curPrivacy = 0.0;

                // 遍历所有激活的规则组（快递包裹）
                for (const auto& [prefix, group] : evaluator->activeRulesMap_) {

                    // [时间规则过滤] 如果不检查时间规则，跳过相关规则
                    if (!cand.checkTimeRules && (prefix == "dt" || prefix == "wdd" || prefix == "wdh")) continue;

                    if (group.type == "hash-fields") {
                        string ruleKey = prefix + "_" + to_string(group.level);
                        if (ruleKey != cand.wdRule) continue;
                    }

                    // 截取网格编码适配层级
                    string sliceCode = cand.code;
                    if (cand.code.length() > (size_t)group.level) {
                        sliceCode = cand.code.substr(0, group.level);
                    }

                    // 构造当前规则在 Redis 缓存中的键名，拦截 hlz 重定向到 hl
                    string queryPrefix = (prefix == "hlz") ? "hl" : prefix;
                    string redisKeyBase = queryPrefix + "_" + sliceCode;

                    // ==========================================
                    // 核心逻辑合并：红线校验 + 代价提取
                    // ==========================================

                    // 【类型1：处理字符串类型数据（包含存在性校验 和 离散值提取）】
                    if (group.type == "string") {
                        if (evaluator->redisCache_.count(redisKeyBase)) {
                            auto val = evaluator->redisCache_[redisKeyBase];
                            // 1. 红线校验
                            bool passedRedLine = true;
                            for (const auto& rule : group.rules) {
                                if (!evaluator->evaluateConstraint(rule, val)) {
                                    candPass = false; failReason = rule.description;
                                    passedRedLine = false;
                                    break;
                                }
                            }

                            // 2. 离散型代价提取（如：风险区、隐私区、电磁）
                            if (passedRedLine && val.isString()) {
                                std::string sVal = val.asString();

                                if (prefix == "dc") curEm = extractDiscreteCost(group.rules[0].expectedValue, sVal);
                                else if (prefix == "tx") curComm = extractDiscreteCost(group.rules[0].expectedValue, sVal);
                                else if (prefix == "dh") curNav = extractDiscreteCost(group.rules[0].expectedValue, sVal);
                                else if (prefix == "jk") curSurv = extractDiscreteCost(group.rules[0].expectedValue, sVal);
                            }
                        } else {
                            // 缓存没值，处理 hlz, hl 等非空验证逻辑
                            for (const auto& rule : group.rules) {
                                if (!evaluator->evaluateConstraint(rule, Json::Value::null)) {
                                    candPass = false; failReason = rule.description; break;
                                }
                            }
                        }
                    }

                    // 【类型2：处理集合类型数据（空域）】
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

                    // 【类型3：处理哈希字段类型数据（气象 wdh、通导监 cns 等）】
                    else if (group.type == "hash-fields" || group.type == "json-string") {
                        if (evaluator->redisCache_.count(redisKeyBase)) {
                            auto hashVal = evaluator->redisCache_[redisKeyBase];
                            if (hashVal.isObject()) {
                                for (const auto& rule : group.rules) {
                                    // 拼凑字段名（如果是通导监可能不需要时间后缀，这里兼容一下）
                                    string fieldName = rule.jsonPath + "_" + to_string(cand.wdTime);
                                    string actualField = hashVal.isMember(fieldName) ? fieldName : rule.jsonPath;

                                    if (hashVal.isMember(actualField)) {
                                        // 1. 红线校验 (通用)
                                        if (!evaluator->evaluateConstraint(rule, hashVal[actualField])) {
                                            candPass = false; failReason = rule.description; break;
                                        }

                                        // 2. 核心：提取数值并走动态规则引擎
                                        double val = toNumber(hashVal[actualField]);
                                        double dynamicCost = extractDynamicCost(rule.expectedValue, val);

                                        // 3. 根据前缀精准分发代价值
                                        // 3. 根据前缀精准分发代价值
                                        if (prefix == "wdh" || prefix == "wdd") {
                                            if (rule.jsonPath == "windSpeed") curWind = dynamicCost;
                                            else if (rule.jsonPath == "rainPcpn") curRain = dynamicCost;
                                            else if (rule.jsonPath == "visibility") curVis = dynamicCost;
                                            else if (rule.jsonPath == "temperature" || rule.jsonPath == "tem" || rule.jsonPath == "tem1") curTemp = dynamicCost;
                                            else if (rule.jsonPath == "humidity") curHum = dynamicCost;
                                            else if (rule.jsonPath == "pressure") curPress = dynamicCost;
                                        }
                                        else if (prefix == "fxq") {
                                            curRisk = dynamicCost;     // 风险区代价值
                                        }
                                        else if (prefix == "privacy") {
                                            curPrivacy = dynamicCost;  // 隐私区代价值
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // 提前终止：一旦触发红线，后面的规则不用看了
                    if (!candPass) break;
                }

                // === DP（时空冲突）检查 (保留原有逻辑) ===
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
                                        candPass = false; failReason = "存在时空冲突"; break;
                                    }
                                }
                            }
                        }
                    }
                }

                // ==========================================
                // 【修复核心问题】：完整保存网格对象的所有属性
                // ==========================================
                GridEvaluator::CheckResult result;
                result.pass = candPass;
                result.reason = failReason;

                // 只有通过了红线，才保存代价值传给 A*
                if (candPass) {
                    result.commCost = curComm; result.navCost = curNav; result.survCost = curSurv;
                    result.windCost = curWind; result.rainCost = curRain; result.visCost = curVis;
                    result.tempCost = curTemp; result.humCost = curHum; result.pressCost = curPress;
                    result.emCost = curEm;     result.riskCost = curRisk; result.privacyCost = curPrivacy;
                }

                // 将封装好的完整网格结构存入结果集
                finalResults[cand.code] = result;
            }
        }

        // 返回给协程
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

                if (group.type == "string" || group.type == "json-string") {
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
                    // 【修改点 2】: 尝试将拿到的文本解析为 JSON 对象
                       for (size_t i=0; i<arr.size() && i<stringKeys.size(); ++i) {
                           string val = "";
                           Json::Value jVal(Json::nullValue);
                           if (!arr[i].isNil()) {
                               val = arr[i].asString();

                               // 新增 JSON 反序列化逻辑
                               Json::CharReaderBuilder builder;
                               std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                               std::string errs;
                               bool isJson = reader->parse(val.c_str(), val.c_str() + val.length(), &jVal, &errs);

                               // 如果解析失败（比如它是 dc 规则的纯文本），就回退为普通字符串
                               if (!isJson) {
                                   jVal = val;
                               }
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