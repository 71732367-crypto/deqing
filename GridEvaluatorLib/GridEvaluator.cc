#include "GridEvaluator.h"
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <algorithm>
#include<dqg/DQG3DBasic.h>
#include<fstream>
using namespace drogon;
using namespace std;

namespace api {
namespace airRoute {
    Json::Value g_weightConfig;
    bool loadWeightConfig(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            LOG_ERROR << "[GridEvaluator] Failed to open weight config file: " << filepath;
            return false;
        }
        Json::CharReaderBuilder builder;
        std::string errs;
        if (!Json::parseFromStream(builder, file, &g_weightConfig, &errs)) {
            LOG_ERROR << "[GridEvaluator] Failed to parse weight.json: " << errs;
            return false;
        }
        LOG_INFO << "[GridEvaluator] Successfully loaded weight.json";
        return true;
    }

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
    // === 优化版：数学区间解析器（零堆内存分配） ===
    static bool isValueInRange(const std::string& rangeStr, double val) {
    if (rangeStr.length() < 5) return false; // 长度至少形如 (0,1)

    char leftOp = rangeStr.front();
    char rightOp = rangeStr.back();

    // ✅ 核心修改 2：使用 C 底层的 strtod 和原生指针
    // 坚决不用 substr 截取字符串，实现 0 次 Heap 内存分配
    const char* start = rangeStr.c_str() + 1;
    char* end;

    // 极速提取逗号前的第一个浮点数
    double minVal = std::strtod(start, &end);

    // 格式安全保护，确保中间是逗号
    if (*end != ',') return false;

    // 极速提取逗号后的第二个浮点数
    double maxVal = std::strtod(end + 1, nullptr);

    // 逻辑判断
    bool passLeft = (leftOp == '[') ? (val >= minVal) : (val > minVal);
    bool passRight = (rightOp == ']') ? (val <= maxVal) : (val < maxVal);

    return passLeft && passRight;
}



    // === 新增：多条件阈值校验器 ===
    static bool isHitThreshold(const std::string& thresholdStr, double val) {
        if (thresholdStr.empty()) return false;

        // 按照 "||" 分割字符串
        size_t pos = 0;
        std::string s = thresholdStr;
        std::vector<std::string> conditions;
        while ((pos = s.find("||")) != std::string::npos) {
            conditions.push_back(s.substr(0, pos));
            s.erase(0, pos + 2);
        }
        conditions.push_back(s);
        // 逐个判断是否触发阈值（触发代表不可通行）
        for (const auto& cond : conditions) {
            std::string c = cond;
            // 去除空格
            c.erase(std::remove_if(c.begin(), c.end(), ::isspace), c.end());
            if (c.empty()) continue;

            if (c.substr(0, 2) == ">=") { if (val >= std::stod(c.substr(2))) return true; }
            else if (c.substr(0, 2) == "<=") { if (val <= std::stod(c.substr(2))) return true; }
            else if (c[0] == '>') { if (val > std::stod(c.substr(1))) return true; }
            else if (c[0] == '<') { if (val < std::stod(c.substr(1))) return true; }
            else if (c.substr(0, 2) == "==") { if (std::fabs(val - std::stod(c.substr(2))) < 1e-9) return true; }
        }
        return false;
    }
    // === 新增：动态代价提取器 ===
    /**
     * @brief 从前端传入的 JSON 规则配置中，提取当前数值对应的归一化代价 (0.0 ~ 1.0)
     */
    static double extractDynamicCost(const Json::Value& ruleConfig, double actualVal) {
    // 兼容检查：确保前端传的是新版的对象格式
        if (ruleConfig.isMember("cost")) {
            double c = ruleConfig["cost"].asDouble();
            // 【加一行日志】
  //          LOG_INFO << "命中静态代价 -> 实际值: " << actualVal << " | 提取代价: " << c;
            return c;
        }

        if (!ruleConfig.isMember("value")) {
            return 0.0;
        }

        const auto& intervals = ruleConfig["value"];
        if (intervals.isArray()) {
            for (const auto& item : intervals) {
                if (item.isMember("range") && item.isMember("cost")) {
                    std::string rangeStr = item["range"].asString();
                    if (isValueInRange(rangeStr, actualVal)) {
                        double c = item["cost"].asDouble();
                        // 【加一行日志】
        //                LOG_INFO << "命中区间代价 -> 区间: " << rangeStr << " | 实际值: " << actualVal << " | 提取代价: " << c;
                        return c;
                    }
                }
            }
        }

    // 如果都不匹配，返回 0 (无额外代价，阈值机制已接管不可通行判断)
    return 0.0;
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
    // 辅助函数：将 "HH:MM" 转换为当天的分钟数 (例如 "10:30" 转换为 630)
    static int parseTimeStr(const std::string& timeStr) {
        if (timeStr.length() < 5) return -1;
        int h = 0, m = 0;
        try {
            h = std::stoi(timeStr.substr(0, 2));
            m = std::stoi(timeStr.substr(3, 2));
        } catch(...) { return -1; }
        return h * 60 + m;
    }
    // 核心判断函数：校验无人机预计到达时间(arrivalTime)是否符合指定的 risk 时段和日期要求
    static bool isTimeInRanges(int arrivalTime, const std::string& fieldName, const std::string& rangesStr) {
        // 前端传入的是 Unix 时间戳（UTC 绝对秒数），转换为北京时间需要 + 8 小时 (8 * 3600 秒)
        time_t t = arrivalTime + 8 * 3600;
        struct tm tm_time;
        // 使用线程安全的 gmtime_r 取出对应的小时和星期
        gmtime_r(&t, &tm_time);

        int currentMinutes = tm_time.tm_hour * 60 + tm_time.tm_min;
        bool isWeekend = (tm_time.tm_wday == 0 || tm_time.tm_wday == 6);

        // 校验日类型：根据规则字段名，过滤掉不符合当日类型的判定
        if (fieldName.find("weekend") == 0 && !isWeekend) return false;
        if (fieldName.find("workday") == 0 && isWeekend) return false;
        if (fieldName.find("holiday") == 0 && !isWeekend) return false; // 简化处理：将周末视为节假日
        // 校验具体时分段（支持如 "00:00-10:00,22:00-24:00" 的多个时段）
        auto timeParts = split(rangesStr, ',');
        for (const auto& part : timeParts) {
            auto bounds = split(part, '-');
            if (bounds.size() == 2) {
                int startMin = parseTimeStr(bounds[0]);
                int endMin = parseTimeStr(bounds[1]);
                if (endMin == 0 && bounds[1].substr(0,2) == "24") endMin = 1440; // 兼容 24:00

                if (currentMinutes >= startMin && currentMinutes <= endMin) {
                    return true; // 匹配成功
                }
            }
        }
        return false;
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
                meta.expectedValue = options[key];  // ← 加这行，保存完整的配置对象
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
        return 0.0;
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

                    // 先截取网格编码适配层级，保证层级匹配
                    string sliceCode = cand.code;
                    if (sliceCode.length() > (size_t)group.level) {
                        sliceCode = sliceCode.substr(0, group.level);
                    }

                    // 【修复核心】：在适配后的安全层级上进行地面投影，实现绝对拦截
                    if (prefix == "dz" || prefix == "ad") {
                        IJH ijh = getLocalTileRHC(sliceCode);
                        ijh.layer = 0;
                        sliceCode = rchToCode(ijh, static_cast<uint8_t>(sliceCode.length()));
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
                                //======电子围栏时间校验=======
                                                              // === 新增：临时空域(电子围栏)时间校验 (兼容数组和单个对象，无Lambda版) ===
                                if (rule.prefix == "dz" && cand.checkTimeRules) {
                                    bool timeConflict = false;

                                    if (val.isArray()) {
                                        // 遍历数组
                                        for (unsigned int i = 0; i < val.size(); ++i) {
                                            const Json::Value& timeRange = val[i];
                                            if (timeRange.isObject() && timeRange.isMember("start_time") && timeRange.isMember("end_time")) {
                                                std::string stStr = timeRange["start_time"].asString();
                                                std::string etStr = timeRange["end_time"].asString();

                                                struct tm tm_st = {0};
                                                struct tm tm_et = {0};
                                                int y, M, d, h, m, s;

                                                if (sscanf(stStr.c_str(), "%d-%d-%d %d:%d:%d", &y, &M, &d, &h, &m, &s) == 6) {
                                                    tm_st.tm_year = y - 1900; tm_st.tm_mon = M - 1; tm_st.tm_mday = d;
                                                    tm_st.tm_hour = h; tm_st.tm_min = m; tm_st.tm_sec = s;
                                                }
                                                if (sscanf(etStr.c_str(), "%d-%d-%d %d:%d:%d", &y, &M, &d, &h, &m, &s) == 6) {
                                                    tm_et.tm_year = y - 1900; tm_et.tm_mon = M - 1; tm_et.tm_mday = d;
                                                    tm_et.tm_hour = h; tm_et.tm_min = m; tm_et.tm_sec = s;
                                                }

                                                int st = static_cast<int>(timegm(&tm_st));
                                                int et = static_cast<int>(timegm(&tm_et));

                                                if (cand.arrivalTime >= st && cand.arrivalTime <= et) {
                                                    timeConflict = true;
                                                    break; // 只要有一个时间段冲突，就拦截
                                                }
                                            }
                                        }
                                    }
                                    else if (val.isObject()) {
                                        // 单个对象直接校验
                                        if (val.isMember("start_time") && val.isMember("end_time")) {
                                            std::string stStr = val["start_time"].asString();
                                            std::string etStr = val["end_time"].asString();

                                            struct tm tm_st = {0};
                                            struct tm tm_et = {0};
                                            int y, M, d, h, m, s;

                                            if (sscanf(stStr.c_str(), "%d-%d-%d %d:%d:%d", &y, &M, &d, &h, &m, &s) == 6) {
                                                tm_st.tm_year = y - 1900; tm_st.tm_mon = M - 1; tm_st.tm_mday = d;
                                                tm_st.tm_hour = h; tm_st.tm_min = m; tm_st.tm_sec = s;
                                            }
                                            if (sscanf(etStr.c_str(), "%d-%d-%d %d:%d:%d", &y, &M, &d, &h, &m, &s) == 6) {
                                                tm_et.tm_year = y - 1900; tm_et.tm_mon = M - 1; tm_et.tm_mday = d;
                                                tm_et.tm_hour = h; tm_et.tm_min = m; tm_et.tm_sec = s;
                                            }

                                            int st = static_cast<int>(timegm(&tm_st));
                                            int et = static_cast<int>(timegm(&tm_et));

                                            if (cand.arrivalTime >= st && cand.arrivalTime <= et) {
                                                timeConflict = true;
                                            }
                                        }
                                    }

                                    // 算路到达该网格的时间不在禁飞时间段内，或者是无法解析的脏数据，则放行
                                    if (!timeConflict && (val.isArray() || val.isObject())) {
                                        continue;
                                    }
                                }
                                // ========================================

                                if (!evaluator->evaluateConstraint(rule, val)) {
                                    candPass = false; failReason = rule.description;
                                    passedRedLine = false;
                                    break;
                                }
                            }

                            // 2. 离散型代价提取（如：风险区、隐私区、电磁）
                            if (passedRedLine && (val.isString() || val.isNumeric())) {
                                std::string sVal = val.asString();


                                if (prefix == "tx") {
                                    LOG_INFO << "[DEBUG TX] Found grid: " << redisKeyBase << ", raw value: " << sVal;
                                }

                                if (prefix == "dc") {
                                    // 加上异常捕获，防止 redis 里存的不是纯数字导致程序崩溃
                                    try {
                                        curEm = extractDynamicCost(group.rules[0].expectedValue, std::stod(sVal));
                                    } catch (...) {
                                        curEm = 0; // 如果转换失败，默认给最高代价
                                    }
                                }
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

                                        double dynamicCost = 0.0;
                                       if (prefix == "fxq") {
                                           // 把 Redis 里的那串时间字符串安全地取出来（比如 "00:00-10:00..."）
                                           std::string rangeStr = hashVal[actualField].isString() ? hashVal[actualField].asString() : "";
                                           if (!rangeStr.empty() && isTimeInRanges(cand.arrivalTime, rule.jsonPath, rangeStr)) {
                                               dynamicCost = extractDynamicCost(rule.expectedValue, 0.0);
                                           }else {
                                               continue;
                                           }
                                       }else {
                                           // 2. 核心：提取数值并走动态规则引擎
                                           double val = toNumber(hashVal[actualField]);
                                           if (rule.expectedValue.isObject() && rule.expectedValue.isMember("threshold")) {
                                               if (isHitThreshold(rule.expectedValue["threshold"].asString(), val)) {
                                                   candPass = false;
                                                   failReason = rule.description + " 触及极端阈值限制(threshold)";
                                                   break;
                                               }
                                           }
                                           dynamicCost = extractDynamicCost(rule.expectedValue, val);
                                       }
                                        // 3. 根据前缀精准分发代价值
                                        if (prefix == "wdh" || prefix == "wdd") {
                                            if (rule.jsonPath == "windSpeed") curWind = dynamicCost;
                                            else if (rule.jsonPath == "rainPcpn") curRain = dynamicCost;
                                            else if (rule.jsonPath == "visibility") curVis = dynamicCost;
                                            else if (rule.jsonPath == "temperature" || rule.jsonPath == "tem" || rule.jsonPath == "tem1") curTemp = dynamicCost;
                                            else if (rule.jsonPath == "humidity") curHum = dynamicCost;
                                            else if (rule.jsonPath == "pressure") curPress = dynamicCost;
                                        }else if (prefix == "fxq") {
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

                // 先截取网格编码适配层级，保证层级匹配
                string sliceCode = cand.code;
                if (sliceCode.length() > (size_t)group.level) {
                    sliceCode = sliceCode.substr(0, group.level);
                }

                // 【修复核心】：在适配后的安全层级上进行地面投影，实现绝对拦截
                if (prefix == "dz" || prefix == "ad") {
                    IJH ijh = getLocalTileRHC(sliceCode);
                    ijh.layer = 0;
                    sliceCode = rchToCode(ijh, static_cast<uint8_t>(sliceCode.length()));
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

                    // ✅ 核心修改 1：将 JSON 解析器的创建移到 for 循环外部！
                    // 整个批次回调只创建一次，彻底消灭海量堆内存分配
                    Json::CharReaderBuilder builder;
                    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                    std::string errs; // 错误信息也放外面复用

                    for (size_t i=0; i<arr.size() && i<stringKeys.size(); ++i) {
                        string val = "";
                        Json::Value jVal(Json::nullValue);
                        if (!arr[i].isNil()) {
                            val = arr[i].asString();

                            // ✅ 复用外面的 reader 进行极速解析
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