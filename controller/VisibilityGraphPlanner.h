#pragma once

#include <drogon/orm/DbClient.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace api::airRoute {

struct VisibilityPoint {
    double longitude = 0.0;
    double latitude = 0.0;
};

struct VisibilityPlanResult {
    bool success = false;
    bool usedGraph = false;
    std::vector<VisibilityPoint> path;
    std::string reason;
};

/// 从数据库活动图中查询二维绕行路径。可见性回调只负责校验起终点临时连边。
class VisibilityGraphPlanner {
public:
    using VisibilityCheck = std::function<bool(const VisibilityPoint&, const VisibilityPoint&)>;

    static VisibilityGraphPlanner& instance();

    VisibilityPlanResult plan(
        const drogon::orm::DbClientPtr& db,
        const VisibilityPoint& start,
        const VisibilityPoint& goal,
        const std::unordered_set<std::string>& blockedRiskIds,
        const std::unordered_set<std::string>& blockedTemporaryFenceIds,
        const VisibilityCheck& visibilityCheck);

private:
    struct GraphData;

    VisibilityGraphPlanner() = default;
    std::shared_ptr<const GraphData> loadActiveGraph(const drogon::orm::DbClientPtr& db);
};

}  // namespace api::airRoute
