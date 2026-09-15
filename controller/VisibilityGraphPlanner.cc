#include "VisibilityGraphPlanner.h"

#include <drogon/drogon.h>
#include <json/json.h>
#include <proj.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace api::airRoute {
namespace {

struct CandidateNode {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
};

struct GraphEdge {
    int target = -1;
    double distance = 0.0;
    std::vector<std::string> riskIds;
    std::vector<std::string> temporaryFenceIds;
};

struct QueueNode {
    int id = -1;
    double f = 0.0;
    double g = 0.0;
};

struct QueueNodeGreater {
    bool operator()(const QueueNode& lhs, const QueueNode& rhs) const {
        if (lhs.f != rhs.f) {
            return lhs.f > rhs.f;
        }
        return lhs.g > rhs.g;
    }
};

std::vector<std::string> parseMetadataIds(const std::string& text) {
    std::vector<std::string> result;
    if (text.empty() || text == "[]") {
        return result;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    std::istringstream input(text);
    if (!Json::parseFromStream(builder, input, &root, &errors) || !root.isArray()) {
        throw std::runtime_error("可见边动态区域元数据不是有效JSON数组");
    }

    result.reserve(root.size());
    for (const auto& item : root) {
        if (!item.isObject() || !item.isMember("id")) {
            continue;
        }
        if (item["id"].isString() || item["id"].isIntegral()) {
            result.push_back(item["id"].asString());
        }
    }
    return result;
}

bool containsBlockedId(
    const std::vector<std::string>& ids,
    const std::unordered_set<std::string>& blockedIds) {
    for (const auto& id : ids) {
        if (blockedIds.count(id) != 0) {
            return true;
        }
    }
    return false;
}

class CoordinateTransformer {
public:
    explicit CoordinateTransformer(int targetSrid) {
        context_ = proj_context_create();
        if (!context_) {
            throw std::runtime_error("无法创建PROJ上下文");
        }

        const std::string target = "EPSG:" + std::to_string(targetSrid);
        PJ* raw = proj_create_crs_to_crs(context_, "EPSG:4326", target.c_str(), nullptr);
        if (!raw) {
            throw std::runtime_error("无法创建WGS84到活动图坐标系的转换");
        }

        transform_ = proj_normalize_for_visualization(context_, raw);
        proj_destroy(raw);
        if (!transform_) {
            throw std::runtime_error("无法规范化活动图坐标转换");
        }
    }

    ~CoordinateTransformer() {
        proj_destroy(transform_);
        proj_context_destroy(context_);
    }

    CoordinateTransformer(const CoordinateTransformer&) = delete;
    CoordinateTransformer& operator=(const CoordinateTransformer&) = delete;

    std::pair<double, double> forward(const VisibilityPoint& point) const {
        const PJ_COORD input = proj_coord(point.longitude, point.latitude, 0.0, 0.0);
        const PJ_COORD output = proj_trans(transform_, PJ_FWD, input);
        if (!std::isfinite(output.xy.x) || !std::isfinite(output.xy.y)) {
            throw std::runtime_error("起终点坐标转换失败");
        }
        return {output.xy.x, output.xy.y};
    }

private:
    PJ_CONTEXT* context_ = nullptr;
    PJ* transform_ = nullptr;
};

double distance(double ax, double ay, double bx, double by) {
    return std::hypot(bx - ax, by - ay);
}

}  // namespace

struct VisibilityGraphPlanner::GraphData {
    long long graphId = 0;
    long long graphVersion = 0;
    int srid = 4326;
    std::vector<CandidateNode> candidates;
    std::vector<std::vector<GraphEdge>> adjacency;
};

VisibilityGraphPlanner& VisibilityGraphPlanner::instance() {
    static VisibilityGraphPlanner planner;
    return planner;
}

std::shared_ptr<const VisibilityGraphPlanner::GraphData>
VisibilityGraphPlanner::loadActiveGraph(const drogon::orm::DbClientPtr& db) {
    if (!db) {
        throw std::runtime_error("数据库客户端不可用");
    }

    static std::mutex cacheMutex;
    static std::shared_ptr<const GraphData> cache;
    std::lock_guard<std::mutex> lock(cacheMutex);

    const auto graphRows = db->execSqlSync(
        "SELECT id, graph_version, srid FROM air_space_graph "
        "WHERE is_active = TRUE "
        "ORDER BY graph_version DESC LIMIT 1");
    if (graphRows.empty()) {
        throw std::runtime_error("未找到已激活可见图");
    }

    const long long graphId = graphRows[0]["id"].as<long long>();
    if (cache && cache->graphId == graphId) {
        return cache;
    }

    auto graph = std::make_shared<GraphData>();
    graph->graphId = graphId;
    graph->graphVersion = graphRows[0]["graph_version"].as<long long>();
    graph->srid = graphRows[0]["srid"].as<int>();

    const auto candidateRows = db->execSqlSync(
        "SELECT candidate_id, x, y, longitude, latitude "
        "FROM air_space_graph_candidate WHERE graph_id = $1 "
        "ORDER BY candidate_id",
        graphId);
    if (candidateRows.empty()) {
        throw std::runtime_error("活动图没有候选点");
    }

    graph->candidates.reserve(candidateRows.size());
    for (const auto& row : candidateRows) {
        CandidateNode candidate;
        candidate.id = row["candidate_id"].as<int>();
        candidate.x = row["x"].as<double>();
        candidate.y = row["y"].as<double>();
        candidate.longitude = row["longitude"].as<double>();
        candidate.latitude = row["latitude"].as<double>();
        if (candidate.id != static_cast<int>(graph->candidates.size())) {
            throw std::runtime_error("活动图候选点ID不连续");
        }
        graph->candidates.push_back(candidate);
    }

    graph->adjacency.resize(graph->candidates.size());
    const auto edgeRows = db->execSqlSync(
        "SELECT source_id, target_id, distance_m, risk_areas::text, "
        "temporary_fences::text FROM air_space_graph_edge "
        "WHERE graph_id = $1 ORDER BY source_id, target_id",
        graphId);
    if (edgeRows.empty()) {
        throw std::runtime_error("活动图没有可见边");
    }

    for (const auto& row : edgeRows) {
        const int source = row["source_id"].as<int>();
        const int target = row["target_id"].as<int>();
        if (source < 0 || target < 0 ||
            source >= static_cast<int>(graph->candidates.size()) ||
            target >= static_cast<int>(graph->candidates.size()) || source == target) {
            throw std::runtime_error("活动图包含无效边端点");
        }

        GraphEdge forward;
        forward.target = target;
        forward.distance = row["distance_m"].as<double>();
        forward.riskIds = parseMetadataIds(row["risk_areas"].as<std::string>());
        forward.temporaryFenceIds = parseMetadataIds(
            row["temporary_fences"].as<std::string>());

        GraphEdge reverse = forward;
        reverse.target = source;
        graph->adjacency[source].push_back(std::move(forward));
        graph->adjacency[target].push_back(std::move(reverse));
    }

    cache = graph;
    LOG_INFO << "[VisibilityGraph] 已加载活动图 id=" << graph->graphId
             << ", version=" << graph->graphVersion
             << ", candidates=" << graph->candidates.size()
             << ", edges=" << edgeRows.size();
    return cache;
}

VisibilityPlanResult VisibilityGraphPlanner::plan(
    const drogon::orm::DbClientPtr& db,
    const VisibilityPoint& start,
    const VisibilityPoint& goal,
    const std::unordered_set<std::string>& blockedRiskIds,
    const std::unordered_set<std::string>& blockedTemporaryFenceIds,
    const VisibilityCheck& visibilityCheck) {
    VisibilityPlanResult result;

    try {
        if (visibilityCheck(start, goal)) {
            result.success = true;
            result.path = {start, goal};
            return result;
        }

        const auto graph = loadActiveGraph(db);
        CoordinateTransformer transformer(graph->srid);
        const auto [startX, startY] = transformer.forward(start);
        const auto [goalX, goalY] = transformer.forward(goal);
        const int candidateCount = static_cast<int>(graph->candidates.size());
        const int startId = candidateCount;
        const int goalId = candidateCount + 1;

        std::vector<int> startNearest(candidateCount);
        std::vector<int> goalNearest(candidateCount);
        for (int i = 0; i < candidateCount; ++i) {
            startNearest[i] = i;
            goalNearest[i] = i;
        }
        std::sort(startNearest.begin(), startNearest.end(), [&](int lhs, int rhs) {
            const auto& a = graph->candidates[lhs];
            const auto& b = graph->candidates[rhs];
            return distance(startX, startY, a.x, a.y) <
                   distance(startX, startY, b.x, b.y);
        });
        std::sort(goalNearest.begin(), goalNearest.end(), [&](int lhs, int rhs) {
            const auto& a = graph->candidates[lhs];
            const auto& b = graph->candidates[rhs];
            return distance(goalX, goalY, a.x, a.y) <
                   distance(goalX, goalY, b.x, b.y);
        });

        std::vector<signed char> startVisibility(candidateCount, -1);
        std::vector<signed char> goalVisibility(candidateCount, -1);
        std::vector<double> startDistance(candidateCount, std::numeric_limits<double>::infinity());
        std::vector<double> goalDistance(candidateCount, std::numeric_limits<double>::infinity());

        std::vector<int> stages = {32, 64, 128, candidateCount};
        stages.erase(std::unique(stages.begin(), stages.end()), stages.end());

        for (int stageSize : stages) {
            stageSize = std::min(stageSize, candidateCount);
            for (int index = 0; index < stageSize; ++index) {
                const int startCandidateId = startNearest[index];
                const auto& startCandidate = graph->candidates[startCandidateId];
                if (startVisibility[startCandidateId] < 0) {
                    const VisibilityPoint point{
                        startCandidate.longitude, startCandidate.latitude};
                    startVisibility[startCandidateId] = visibilityCheck(start, point) ? 1 : 0;
                    if (startVisibility[startCandidateId] != 0) {
                        startDistance[startCandidateId] = distance(
                            startX, startY, startCandidate.x, startCandidate.y);
                    }
                }

                const int goalCandidateId = goalNearest[index];
                const auto& goalCandidate = graph->candidates[goalCandidateId];
                if (goalVisibility[goalCandidateId] < 0) {
                    const VisibilityPoint point{
                        goalCandidate.longitude, goalCandidate.latitude};
                    goalVisibility[goalCandidateId] = visibilityCheck(point, goal) ? 1 : 0;
                    if (goalVisibility[goalCandidateId] != 0) {
                        goalDistance[goalCandidateId] = distance(
                            goalCandidate.x, goalCandidate.y, goalX, goalY);
                    }
                }
            }

            std::vector<double> best(candidateCount + 2, std::numeric_limits<double>::infinity());
            std::vector<int> parent(candidateCount + 2, -1);
            std::vector<unsigned char> closed(candidateCount + 2, 0);
            std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeGreater> open;

            auto heuristic = [&](int id) {
                if (id == goalId) {
                    return 0.0;
                }
                if (id == startId) {
                    return distance(startX, startY, goalX, goalY);
                }
                const auto& candidate = graph->candidates[id];
                return distance(candidate.x, candidate.y, goalX, goalY);
            };

            best[startId] = 0.0;
            open.push({startId, heuristic(startId), 0.0});

            auto relax = [&](int from, int to, double edgeDistance) {
                const double newCost = best[from] + edgeDistance;
                if (newCost + 1e-9 >= best[to]) {
                    return;
                }
                best[to] = newCost;
                parent[to] = from;
                open.push({to, newCost + heuristic(to), newCost});
            };

            while (!open.empty()) {
                const QueueNode current = open.top();
                open.pop();
                if (closed[current.id] != 0 || current.g > best[current.id] + 1e-9) {
                    continue;
                }
                closed[current.id] = 1;
                if (current.id == goalId) {
                    break;
                }

                if (current.id == startId) {
                    for (int index = 0; index < stageSize; ++index) {
                        const int id = startNearest[index];
                        if (startVisibility[id] != 0) {
                            relax(startId, id, startDistance[id]);
                        }
                    }
                    continue;
                }

                for (const auto& edge : graph->adjacency[current.id]) {
                    if (containsBlockedId(edge.riskIds, blockedRiskIds) ||
                        containsBlockedId(edge.temporaryFenceIds, blockedTemporaryFenceIds)) {
                        continue;
                    }
                    relax(current.id, edge.target, edge.distance);
                }

                if (goalVisibility[current.id] != 0) {
                    relax(current.id, goalId, goalDistance[current.id]);
                }
            }

            if (!std::isfinite(best[goalId])) {
                continue;
            }

            std::vector<int> nodePath;
            for (int id = goalId; id >= 0; id = parent[id]) {
                nodePath.push_back(id);
                if (id == startId) {
                    break;
                }
            }
            if (nodePath.empty() || nodePath.back() != startId) {
                continue;
            }
            std::reverse(nodePath.begin(), nodePath.end());

            result.path.reserve(nodePath.size());
            for (const int id : nodePath) {
                if (id == startId) {
                    result.path.push_back(start);
                } else if (id == goalId) {
                    result.path.push_back(goal);
                } else {
                    const auto& candidate = graph->candidates[id];
                    result.path.push_back({candidate.longitude, candidate.latitude});
                }
            }
            result.success = true;
            result.usedGraph = true;
            return result;
        }

        result.reason = "活动可见图中未找到可用绕行路径";
    } catch (const std::exception& error) {
        result.reason = error.what();
    }
    return result;
}

}  // namespace api::airRoute
