#pragma once

#include <drogon/drogon.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <functional>
#include <atomic>
#include <optional>
#include <json/json.h>

namespace api {
    namespace airRoute {
        // === 1. 权重配置结构体 (映射 weight_config.csv) ===
        // 用于存储某一特定模式（如“最短路径”）下的所有因子权重
        extern Json::Value g_weightConfig;
        bool loadWeightConfig(const std::string& filepath);
        struct RouteWeights {
            double distance;      // 飞行距离权重 (对应 flight_distance，如 0.5)
            double efficiency;    // 效率/高度变化权重 (对应 height_change，如 0.15)

            // 安全子因子 (总计占 15%)
            double comm;        // 通信权重 (对应 communication，如 0.0338)
            double nav;          // 导航权重 (对应 navigation，如 0.0263)
            double surv;        // 监视权重 (对应 surveillance，如 0.015)
            double wind;        // 风速权重 (0.0203)
            double rain;        // 降水权重 (0.0081)
            double vis;        // 能见度权重 (0.0054)
            double temp;        // 温度权重 (0.0045)
            double hum;           // 湿度权重 (0.0036)
            double press;         // 气压权重 (0.0032)
            double em;            // 电磁权重 (0.03)

            // 风险与隐私
            double riskArea;    // 风险区权重 (对应 risk_area，如 0.10)
            double privacy;      // 隐私/居民区权重 (对应 residential_area，如 0.10)
        };

        // === 2. 枚举类：定义寻路模式 ===
        // enum class 是 C++11 的强类型枚举，比传统的 enum 更安全，防止命名冲突
        enum class RouteMode {
            ORIGINAL = 0, // 原始 A* (无算路因子影响)
            SHORTEST=1,   // 最短路径 (对应 weight_value1)
            SAFEST=2,     // 风险最低 (对应 weight_value2)
            BALANCED=3    // 综合最优 (对应 weight_value3)
        };
        //获取对应模式的权重配置
        static RouteWeights getWeightsByMode(RouteMode mode) {
            std::string modeStr = "original";
            if (mode == RouteMode::SHORTEST) modeStr = "shortest";
            else if (mode == RouteMode::SAFEST) modeStr = "safest";
            else if (mode == RouteMode::BALANCED) modeStr = "balanced";

            // 1. 如果配置加载成功，从 g_weightConfig["modes"] 中动态读取权重
            if (g_weightConfig.isObject() && g_weightConfig.isMember("modes") && g_weightConfig["modes"].isMember(modeStr)) {
                const auto& jw = g_weightConfig["modes"][modeStr];
                return {
                    jw.get("distance", 1.0).asDouble(),
                    jw.get("efficiency", 0.0).asDouble(),
                    jw.get("comm", 0.0).asDouble(),
                    jw.get("nav", 0.0).asDouble(),
                    jw.get("surv", 0.0).asDouble(),
                    jw.get("wind", 0.0).asDouble(),
                    jw.get("rain", 0.0).asDouble(),
                    jw.get("vis", 0.0).asDouble(),
                    jw.get("temp", 0.0).asDouble(),
                    jw.get("hum", 0.0).asDouble(),
                    jw.get("press", 0.0).asDouble(),
                    jw.get("em", 0.0).asDouble(),
                    jw.get("riskArea", 0.0).asDouble(),
                    jw.get("privacy", 0.0).asDouble()
                };
            }


    if (mode == RouteMode::SHORTEST) {
        // 【最短模式】距离锚定 1.0。极度头铁，最多只愿意为了极高风险区和强风绕路 20%
        // 顺序: dist, eff, comm, nav, surv, wind, rain, vis, temp, hum, press, em, riskArea, privacy
        return {1.0, 0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0};

    } else if (mode == RouteMode::SAFEST) {
        // 【最安全模式】距离锚定 1.0。极度惜命，宁愿多飞 1.4 倍的距离(最大额外惩罚 1.45)也要绕开恶劣环境
        return {1.0, 0.15, 0.1, 0.1, 0.05, 0.2, 0.1, 0.05, 0.05, 0.0, 0.0, 0.1, 0.4, 0.15};

    } else if (mode == RouteMode::BALANCED) {
        // 【最均衡模式】距离锚定 1.0。日常巡检最佳，遇到高风险最多愿意绕路 70% (最大额外惩罚 0.70)
        return {1.0, 0.1, 0.05, 0.05, 0.0, 0.1, 0.05, 0.0, 0.0, 0.0, 0.0, 0.05, 0.2, 0.1};

    } else {
        // 【默认 ORIGINAL】 100% 依赖物理距离，完全无视所有附加代价
        return {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
        }
        struct CandidateInfo {

            std::string code;                  // 网格编码
            int arrivalTime;                   // 到达时间（秒，北京时间 UTC+8）
            int wdTime;                        // 规范化后的天气数据时间戳（北京时间，按日或按小时对齐）
            std::string wdRule;                // 天气规则标识（"wdd_11"表示天级别，"wdh_11"表示小时级别）
            bool checkTimeRules = true;        // 控制是否检查时间依赖的约束(dp, dt, wdd, wdh)
        };

        struct AsyncContext;

        class GridEvaluator : public std::enable_shared_from_this<GridEvaluator> {
        public:
            struct CheckResult {
                bool pass;
                std::string reason;
                // --- 以下为归一化后的代价值 (0.0 表示无影响，1.0 表示满额代价) 默认为0---
                double commCost = 0.0;   // 通信代价
                double navCost = 0.0;    // 导航代价
                double survCost = 0.0;   // 监视代价
                double windCost = 0.0;   // 风速代价
                double rainCost = 0.0;   // 降水代价
                double visCost = 0.0;    // 能见度代价
                double tempCost = 0.0;   // 温度代价
                double humCost = 0.0;    // 湿度代价
                double pressCost = 0.0;  // 气压代价
                double emCost = 0.0;     // 电磁代价
                double riskCost = 0.0;   // 风险区代价
                double privacyCost = 0.0;// 隐私区代价
            };

            using CandidatesCallback = std::function<void(const std::unordered_map<std::string, CheckResult>&)>;

            struct RuleMeta {
                std::string prefix;
                std::string type;
                std::string op;
                std::string jsonPath;
                bool checkValueMustExist = false;
                bool checkValueNotEmpty = false;
                std::string description;

                Json::Value expectedValue;
            };

            struct ActiveRuleGroup {
                std::string type;
                int level;
                std::vector<RuleMeta> rules;
                std::vector<std::string> requestedFields;
            };

            static std::shared_ptr<GridEvaluator> create(const Json::Value& options);
            GridEvaluator(const Json::Value& options);

            void checkCandidates(
                const std::vector<CandidateInfo>& candidates,
                CandidatesCallback callback
            );

            friend struct AsyncContext;

        private:
            bool evaluateConstraint(const RuleMeta& rule, const Json::Value& actual);

        private:
            bool hasDp_ = false;
            std::unordered_map<std::string, ActiveRuleGroup> activeRulesMap_;
            std::unordered_map<std::string, Json::Value> redisCache_;
            std::mutex cacheMutex_;
        };

    } // namespace airRoute
} // namespace api