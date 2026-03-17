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