#include "api_airRoute_Astar.h"
#include "GridEvaluator.h"
#include "VisibilityGraphPlanner.h"
#include <drogon/drogon.h>
#include <trantor/net/EventLoop.h>
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
#include <cstdio>
#include <iomanip>
#include <optional>
#include <sstream>
#include <utility>
#include <memory>
#include <limits>
#include <cstdlib>
#include <chrono>
#include <mutex>
#include <map>
#include "TIFF.h"
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/io/WKTReader.h>
#include <drogon/orm/DbClient.h>
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
        auto sync_flag = std::make_shared<bool>(true);
        evaluator->checkCandidates(candidates, [this, h, sync_flag](const std::unordered_map<std::string, GridEvaluator::CheckResult>& res) mutable {
            this->result = std::make_shared<std::unordered_map<std::string, GridEvaluator::CheckResult>>(res);
            if (*sync_flag) {
                auto loop = trantor::EventLoop::getEventLoopOfCurrentThread();
                if (!loop) loop = drogon::app().getLoop();
                loop->queueInLoop([h]() mutable { h.resume(); });
            } else {
                h.resume();
            }
        });
        *sync_flag = false;
    }

    std::shared_ptr<std::unordered_map<std::string, GridEvaluator::CheckResult>> await_resume() { return result; }
};

  namespace {

      struct RouteWaypoint {
          std::array<int, 3> grid;
          bool mustPass = false;
      };

      struct SegmentCheckResult {
          bool safe = false;
          double distance = 0.0;
          double cost = 0.0;
          std::string reason;
      };

      // 每次请求共享原始搜索与抽稀的规则、地形和有向线段校验缓存。
      struct RouteValidationContext {
          std::unordered_map<std::string, GridEvaluator::CheckResult> ruleCache;
          std::unordered_set<std::string> checkedRuleKeys;
          std::unordered_map<std::string, bool> heightSafeCache;
          std::unordered_map<std::string, SegmentCheckResult> segmentCache;
          std::vector<std::array<int, 3>> sphericalMask;
          int maskLevel = -1;
          double maskRadius = -1.0;
          int referenceTime = getBeijingTime();
      };

      struct VectorObstacle {
          std::string id;
          std::vector<std::pair<double, double>> vertices;
      };

      struct CachedFenceObstacle {
          std::string id;
          std::string timeRestrictions;
          VectorObstacle geometry;
      };

      struct CachedRiskObstacle {
          std::string id;
          std::string emergencyDate;
          std::string emergencyLowRiskTime;
          std::string emergencyMidRiskTime;
          std::string emergencyHighRiskTime;
          std::string workdayLowRiskTime;
          std::string workdayMidRiskTime;
          std::string workdayHighRiskTime;
          std::string weekendLowRiskTime;
          std::string weekendMidRiskTime;
          std::string weekendHighRiskTime;
          VectorObstacle geometry;
      };

      struct DatabaseObstacleCache {
          long long graphId = -1;
          long long graphVersion = -1;
          std::chrono::steady_clock::time_point loadedAt{};
          std::unordered_set<std::string> graphTemporaryFenceIds;
          std::vector<VectorObstacle> airSpace;
          std::vector<CachedFenceObstacle> fences;
          std::vector<CachedRiskObstacle> risks;
          bool risksLoaded = false;
      };

      std::mutex databaseObstacleCacheMutex;
      std::shared_ptr<const DatabaseObstacleCache> databaseObstacleCache;
      constexpr std::chrono::seconds databaseObstacleCacheTtl(30);

/// 2D 线段求交（带 ε 容差，防止顶点穿模）
/// 返回值：是否相交；若相交，hitLon/hitLat 填入交点
static bool segmentIntersect2D(
    double ax, double ay, double bx, double by,   // 线段 AB
    double cx, double cy, double dx, double dy,   // 线段 CD
    double& hitX, double& hitY,
    double eps = 0.001)
{
    double denom = (dy - cy) * (bx - ax) - (dx - cx) * (by - ay);
    if (std::abs(denom) < 1e-12) return false;
    double t = ((dx - cx) * (ay - cy) - (dy - cy) * (ax - cx)) / denom;
    double u = ((bx - ax) * (ay - cy) - (by - ay) * (ax - cx)) / denom;
    if (t > eps && t < (1.0 - eps) && u > eps && u < (1.0 - eps)) {
        hitX = ax + t * (bx - ax);
        hitY = ay + t * (by - ay);
        return true;
    }
    return false;
}
/// 判断二维线段是否穿过任一障碍物边界。
static bool segmentHitsAnyObstacle2D(
    double fromLon, double fromLat,
    double toLon, double toLat,
    const std::vector<VectorObstacle>& obstacles)
{
    for (const auto& obstacle : obstacles) {
        const auto& verts = obstacle.vertices;
        if (verts.size() < 3) continue;
        for (size_t i = 0; i < verts.size(); ++i) {
            size_t j = (i + 1) % verts.size();
            double hx, hy;
            if (segmentIntersect2D(fromLon, fromLat, toLon, toLat,
                                   verts[i].first, verts[i].second,
                                   verts[j].first, verts[j].second, hx, hy)) {
                return true;
            }
        }
    }
    return false;
}
/// 点-in-多边形 检测 (射线法, 2D)
static bool pointInAnyObstacle2D(double lon, double lat,
                                  const std::vector<VectorObstacle>& obstacles)
{
    for (const auto& obs : obstacles) {
        const auto& poly = obs.vertices;
        bool inside = false;
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
            if (((poly[i].second > lat) != (poly[j].second > lat)) &&
                (lon < (poly[j].first - poly[i].first) * (lat - poly[i].second) /
                        (poly[j].second - poly[i].second) + poly[i].first))
            {
                inside = !inside;
            }
        }
        if (inside) return true;
    }
    return false;
}
      struct BeijingTimeParts {
          std::tm value{};
          int minutes = 0;
          int dateKey = 0;
          std::string dateTime;
      };

      BeijingTimeParts getBeijingTimeParts(int timestamp) {
          const std::time_t beijingTimestamp = static_cast<std::time_t>(timestamp) + 8 * 3600;
          BeijingTimeParts result;
          gmtime_r(&beijingTimestamp, &result.value);
          result.minutes = result.value.tm_hour * 60 + result.value.tm_min;
          result.dateKey = (result.value.tm_year + 1900) * 10000 +
                           (result.value.tm_mon + 1) * 100 + result.value.tm_mday;
          std::ostringstream stream;
          stream << std::put_time(&result.value, "%Y-%m-%d %H:%M:%S");
          result.dateTime = stream.str();
          return result;
      }

      std::vector<std::string> splitText(const std::string& text, char separator) {
          std::vector<std::string> values;
          std::stringstream stream(text);
          std::string value;
          while (std::getline(stream, value, separator)) {
              values.push_back(value);
          }
          return values;
      }

      bool parseHourMinute(const std::string& text, int& minutes) {
          int hour = 0;
          int minute = 0;
          if (std::sscanf(text.c_str(), "%d:%d", &hour, &minute) != 2 ||
              hour < 0 || hour > 24 || minute < 0 || minute > 59 ||
              (hour == 24 && minute != 0)) {
              return false;
          }
          minutes = hour * 60 + minute;
          return true;
      }

      bool isMinuteInRanges(int currentMinutes, const std::string& ranges) {
          for (const auto& rawRange : splitText(ranges, ',')) {
              const size_t separator = rawRange.find('-');
              if (separator == std::string::npos) {
                  continue;
              }
              int startMinutes = 0;
              int endMinutes = 0;
              if (!parseHourMinute(rawRange.substr(0, separator), startMinutes) ||
                  !parseHourMinute(rawRange.substr(separator + 1), endMinutes)) {
                  continue;
              }
              if (startMinutes <= endMinutes) {
                  if (currentMinutes >= startMinutes &&
                      (currentMinutes < endMinutes || endMinutes == 1440)) {
                      return true;
                  }
              } else if (currentMinutes >= startMinutes || currentMinutes < endMinutes) {
                  return true;
              }
          }
          return false;
      }

      bool parseDateKey(const std::string& text, int& dateKey) {
          int year = 0;
          int month = 0;
          int day = 0;
          if (std::sscanf(text.c_str(), "%d.%d.%d", &year, &month, &day) != 3 &&
              std::sscanf(text.c_str(), "%d-%d-%d", &year, &month, &day) != 3) {
              return false;
          }
          if (month < 1 || month > 12 || day < 1 || day > 31) {
              return false;
          }
          dateKey = year * 10000 + month * 100 + day;
          return true;
      }

      std::optional<size_t> matchingEmergencyRule(
          int dateKey,
          const std::string& emergencyDates) {
          const auto entries = splitText(emergencyDates, ';');
          for (size_t index = 0; index < entries.size(); ++index) {
              const std::string& entry = entries[index];
              int startDate = 0;
              int endDate = 0;
              if (entry.size() >= 21) {
                  const size_t separator = entry.find('-', 10);
                  if (separator != std::string::npos &&
                      parseDateKey(entry.substr(0, separator), startDate) &&
                      parseDateKey(entry.substr(separator + 1), endDate) &&
                      dateKey >= startDate && dateKey <= endDate) {
                      return index;
                  }
              } else if (parseDateKey(entry, startDate) && dateKey == startDate) {
                  return index;
              }
          }
          return std::nullopt;
      }

      std::string emergencyTimeAt(const std::string& value, size_t index) {
          const auto entries = splitText(value, ';');
          return index < entries.size() ? entries[index] : std::string();
      }

      std::string rowText(const drogon::orm::Row& row, const std::string& field) {
          return row[field].isNull() ? std::string() : row[field].as<std::string>();
      }

      int riskLevelAtTime(
          const std::string& emergencyDate,
          const std::string& emergencyLowRiskTime,
          const std::string& emergencyMidRiskTime,
          const std::string& emergencyHighRiskTime,
          const std::string& workdayLowRiskTime,
          const std::string& workdayMidRiskTime,
          const std::string& workdayHighRiskTime,
          const std::string& weekendLowRiskTime,
          const std::string& weekendMidRiskTime,
          const std::string& weekendHighRiskTime,
          const BeijingTimeParts& time) {
          std::string low;
          std::string mid;
          std::string high;
          const auto emergencyIndex = matchingEmergencyRule(time.dateKey, emergencyDate);
          if (emergencyIndex.has_value()) {
              low = emergencyTimeAt(emergencyLowRiskTime, *emergencyIndex);
              mid = emergencyTimeAt(emergencyMidRiskTime, *emergencyIndex);
              high = emergencyTimeAt(emergencyHighRiskTime, *emergencyIndex);
          } else {
              const bool weekend = time.value.tm_wday == 0 || time.value.tm_wday == 6;
              if (weekend) {
                  low = weekendLowRiskTime;
                  mid = weekendMidRiskTime;
                  high = weekendHighRiskTime;
              } else {
                  low = workdayLowRiskTime;
                  mid = workdayMidRiskTime;
                  high = workdayHighRiskTime;
              }
          }

          if (isMinuteInRanges(time.minutes, high)) return 3;
          if (isMinuteInRanges(time.minutes, mid)) return 2;
          if (isMinuteInRanges(time.minutes, low)) return 1;
          return 0;
      }

      int riskLevelAtTime(const drogon::orm::Row& row, const BeijingTimeParts& time) {
          return riskLevelAtTime(
              rowText(row, "emergency_date"),
              rowText(row, "emergency_low_risk_time"),
              rowText(row, "emergency_mid_risk_time"),
              rowText(row, "emergency_high_risk_time"),
              rowText(row, "workday_low_risk_time"),
              rowText(row, "workday_mid_risk_time"),
              rowText(row, "workday_high_risk_time"),
              rowText(row, "weekend_low_risk_time"),
              rowText(row, "weekend_mid_risk_time"),
              rowText(row, "weekend_high_risk_time"),
              time);
      }

      bool modeBlocksRisk(RouteMode mode, int riskLevel) {
          if (mode == RouteMode::SAFEST) return riskLevel >= 1;
          if (mode == RouteMode::BALANCED) return riskLevel >= 2;
          return false;
      }

      bool isTemporaryFenceActive(
          const std::string& restrictionsText,
          const BeijingTimeParts& time) {
          if (restrictionsText.empty()) return true;
          Json::Value restrictions;
          Json::Reader reader;
          if (!reader.parse(restrictionsText, restrictions) || !restrictions.isObject()) {
              return true;
          }

          const auto& dateRange = restrictions["timeRange"];
          if (dateRange.isArray() && dateRange.size() >= 2 &&
              dateRange[0].isString() && dateRange[1].isString() &&
              !dateRange[0].asString().empty() && !dateRange[1].asString().empty()) {
              if (time.dateTime < dateRange[0].asString() ||
                  time.dateTime > dateRange[1].asString()) {
                  return false;
              }
          }

          const auto& days = restrictions["daysActive"];
          if (days.isArray() && !days.empty()) {
              static const std::array<const char*, 7> names = {
                  "Sunday", "Monday", "Tuesday", "Wednesday",
                  "Thursday", "Friday", "Saturday"};
              bool matched = false;
              for (const auto& day : days) {
                  if (day.isString() && day.asString() == names[time.value.tm_wday]) {
                      matched = true;
                      break;
                  }
              }
              if (!matched) return false;
          }

          const auto& dayRange = restrictions["dayTimeRange"];
          if (dayRange.isArray() && dayRange.size() >= 2 &&
              dayRange[0].isString() && dayRange[1].isString() &&
              !dayRange[0].asString().empty() && !dayRange[1].asString().empty()) {
              return isMinuteInRanges(
                  time.minutes, dayRange[0].asString() + "-" + dayRange[1].asString());
          }
          return true;
      }

      bool jsonNumber(const Json::Value& value, double& output) {
          try {
              if (value.isNumeric()) {
                  output = value.asDouble();
                  return std::isfinite(output);
              }
              if (value.isString()) {
                  output = std::stod(value.asString());
                  return std::isfinite(output);
              }
          } catch (...) {
          }
          return false;
      }

      void appendPolygonObstacle(
          const std::string& id,
          const Json::Value& points,
          std::vector<VectorObstacle>& obstacles) {
          if (!points.isArray() || points.size() < 3) return;
          VectorObstacle obstacle;
          obstacle.id = id;
          for (const auto& point : points) {
              if (!point.isObject()) return;
              double longitude = 0.0;
              double latitude = 0.0;
              if (!jsonNumber(point["longitude"], longitude) ||
                  !jsonNumber(point["latitude"], latitude)) {
                  return;
              }
              obstacle.vertices.emplace_back(longitude, latitude);
          }
          obstacles.push_back(std::move(obstacle));
      }

      void appendCircleObstacle(
          const std::string& id,
          const Json::Value& value,
          std::vector<VectorObstacle>& obstacles) {
          if (!value.isArray() || value.empty() || !value[0].isObject()) return;
          double longitude = 0.0;
          double latitude = 0.0;
          double radius = 0.0;
          if (!jsonNumber(value[0]["longitude"], longitude) ||
              !jsonNumber(value[0]["latitude"], latitude) ||
              !jsonNumber(value[0]["radius"], radius) || radius <= 0.0) {
              return;
          }
          Json::Value points(Json::arrayValue);
          constexpr int segments = 64;
          const double longitudeMeters = 111320.0 * std::cos(latitude * M_PI / 180.0);
          for (int index = 0; index < segments; ++index) {
              const double angle = 2.0 * M_PI * index / segments;
              Json::Value point;
              point["longitude"] = longitude + radius * std::cos(angle) / longitudeMeters;
              point["latitude"] = latitude + radius * std::sin(angle) / 111320.0;
              points.append(point);
          }
          appendPolygonObstacle(id, points, obstacles);
      }

      void appendWktObstacle(
          const std::string& id,
          const std::string& wkt,
          geos::io::WKTReader& reader,
          std::vector<VectorObstacle>& obstacles) {
          auto geometry = reader.read(wkt);
          if (!geometry || geometry->isEmpty()) return;
          auto coordinates = geometry->getCoordinates();
          if (!coordinates || coordinates->getSize() < 3) return;

          VectorObstacle obstacle;
          obstacle.id = id;
          obstacle.vertices.reserve(coordinates->getSize());
          for (size_t index = 0; index < coordinates->getSize(); ++index) {
              const auto& coordinate = coordinates->getAt(index);
              obstacle.vertices.emplace_back(coordinate.x, coordinate.y);
          }
          obstacles.push_back(std::move(obstacle));
      }

      std::shared_ptr<const DatabaseObstacleCache> getDatabaseObstacleCache(
          const drogon::orm::DbClientPtr& db,
          bool loadRisks) {
          const auto graphRows = db->execSqlSync(
              "SELECT id, graph_version FROM air_space_graph "
              "WHERE graph_name = 'deqing' AND is_active = TRUE "
              "ORDER BY graph_version DESC LIMIT 1");
          const long long graphId = graphRows.empty() ? -1 : graphRows[0]["id"].as<long long>();
          const long long graphVersion = graphRows.empty()
              ? -1 : graphRows[0]["graph_version"].as<long long>();
          const auto now = std::chrono::steady_clock::now();

          std::lock_guard<std::mutex> lock(databaseObstacleCacheMutex);
          const bool staticCacheFresh =
              databaseObstacleCache &&
              databaseObstacleCache->graphId == graphId &&
              databaseObstacleCache->graphVersion == graphVersion &&
              now - databaseObstacleCache->loadedAt < databaseObstacleCacheTtl;
          const bool riskCacheReady = !loadRisks ||
              (staticCacheFresh && databaseObstacleCache->risksLoaded);
          if (staticCacheFresh && riskCacheReady) {
              return databaseObstacleCache;
          }

          auto refreshed = staticCacheFresh
              ? std::make_shared<DatabaseObstacleCache>(*databaseObstacleCache)
              : std::make_shared<DatabaseObstacleCache>();

          if (!staticCacheFresh) {
              refreshed->graphId = graphId;
              refreshed->graphVersion = graphVersion;
              refreshed->loadedAt = now;
              refreshed->graphTemporaryFenceIds.clear();
              refreshed->airSpace.clear();
              refreshed->fences.clear();
              refreshed->risks.clear();
              refreshed->risksLoaded = false;

              if (graphId >= 0) {
                  const auto temporaryRows = db->execSqlSync(
                      "SELECT DISTINCT item->>'id' AS id "
                      "FROM air_space_graph_edge e "
                      "CROSS JOIN LATERAL jsonb_array_elements(e.temporary_fences) item "
                      "WHERE e.graph_id = $1",
                      graphId);
                  for (const auto& row : temporaryRows) {
                      if (!row["id"].isNull()) {
                          refreshed->graphTemporaryFenceIds.insert(row["id"].as<std::string>());
                      }
                  }
              }

              const auto airSpaceRows = db->execSqlSync(
                  "SELECT id, shape, boundary_data::text AS boundary_data "
                  "FROM air_space WHERE space_type = 'WG'");
              for (const auto& row : airSpaceRows) {
                  Json::Value boundary;
                  Json::Reader reader;
                  if (!reader.parse(rowText(row, "boundary_data"), boundary)) continue;

                  std::vector<VectorObstacle> parsed;
                  const std::string id = row["id"].as<std::string>();
                  if (rowText(row, "shape") == "2") {
                      appendCircleObstacle(id, boundary, parsed);
                  } else {
                      appendPolygonObstacle(id, boundary, parsed);
                  }
                  if (!parsed.empty()) {
                      refreshed->airSpace.push_back(std::move(parsed.front()));
                  }
              }

              const auto fenceRows = db->execSqlSync(
                  "SELECT id, time_restrictions::text AS time_restrictions, "
                  "boundary::text AS boundary FROM fence");
              for (const auto& row : fenceRows) {
                  Json::Value boundary;
                  Json::Reader reader;
                  if (!reader.parse(rowText(row, "boundary"), boundary) ||
                      !boundary.isObject()) {
                      continue;
                  }

                  std::vector<VectorObstacle> parsed;
                  const std::string id = row["id"].as<std::string>();
                  const Json::Value& points = boundary["boundaryData"];
                  if (boundary["shape"].asString() == "2") {
                      appendCircleObstacle(id, points, parsed);
                  } else {
                      appendPolygonObstacle(id, points, parsed);
                  }
                  CachedFenceObstacle fence;
                  fence.id = id;
                  fence.timeRestrictions = rowText(row, "time_restrictions");
                  if (!parsed.empty()) fence.geometry = std::move(parsed.front());
                  // 即使几何无效，也保留记录，保证临时围栏仍能阻断可见图边。
                  refreshed->fences.push_back(std::move(fence));
              }
          }

          if (loadRisks && !refreshed->risksLoaded) {
              const auto riskRows = db->execSqlSync(
                  "SELECT a.id, a.emergency_date, a.emergency_low_risk_time, "
                  "a.emergency_mid_risk_time, a.emergency_high_risk_time, "
                  "r.workday_low_risk_time, r.workday_mid_risk_time, "
                  "r.workday_high_risk_time, r.weekend_low_risk_time, "
                  "r.weekend_mid_risk_time, r.weekend_high_risk_time, "
                  "ST_AsText(parts.geom) AS geom_wkt "
                  "FROM risk_area a JOIN risk_area_rule r ON r.type = a.type "
                  "CROSS JOIN LATERAL ST_Dump(ST_CollectionExtract("
                  "ST_MakeValid(a.geom), 3)) parts");
              auto factory = geos::geom::GeometryFactory::create();
              geos::io::WKTReader wktReader(factory.get());
              refreshed->risks.clear();
              for (const auto& row : riskRows) {
                  CachedRiskObstacle risk;
                  risk.id = row["id"].as<std::string>();
                  risk.emergencyDate = rowText(row, "emergency_date");
                  risk.emergencyLowRiskTime = rowText(row, "emergency_low_risk_time");
                  risk.emergencyMidRiskTime = rowText(row, "emergency_mid_risk_time");
                  risk.emergencyHighRiskTime = rowText(row, "emergency_high_risk_time");
                  risk.workdayLowRiskTime = rowText(row, "workday_low_risk_time");
                  risk.workdayMidRiskTime = rowText(row, "workday_mid_risk_time");
                  risk.workdayHighRiskTime = rowText(row, "workday_high_risk_time");
                  risk.weekendLowRiskTime = rowText(row, "weekend_low_risk_time");
                  risk.weekendMidRiskTime = rowText(row, "weekend_mid_risk_time");
                  risk.weekendHighRiskTime = rowText(row, "weekend_high_risk_time");
                  std::vector<VectorObstacle> parsed;
                  appendWktObstacle(risk.id, rowText(row, "geom_wkt"), wktReader, parsed);
                  if (!parsed.empty()) {
                      risk.geometry = std::move(parsed.front());
                  }
                  // 风险ID需要参与可见边过滤，不能因几何解析失败而丢失。
                  refreshed->risks.push_back(std::move(risk));
              }
              refreshed->risksLoaded = true;
          }

          databaseObstacleCache = refreshed;
          LOG_INFO << "[VisibilityGraph] 数据库障碍物缓存刷新，图版本="
                   << refreshed->graphVersion
                   << "，空域=" << refreshed->airSpace.size()
                   << "，围栏=" << refreshed->fences.size()
                   << "，风险区=" << refreshed->risks.size();
          return databaseObstacleCache;
      }

      void insertDatabaseVisibilityWaypoints(
          std::vector<RouteWaypoint>& waypoints,
          int level,
          const BaseTile& baseTile,
          RouteMode mode,
          int startTime,
          std::vector<VectorObstacle>& thinningObstacles) {
          if (waypoints.size() < 2) return;

          const auto db = drogon::app().getDbClient("default");
          if (!db) {
              LOG_WARN << "[VisibilityGraph] 数据库客户端不可用，保留原始途径点";
              return;
          }

          const BeijingTimeParts time = getBeijingTimeParts(startTime);
          std::unordered_set<std::string> blockedRiskIds;
          std::unordered_set<std::string> graphTemporaryFenceIds;
          std::unordered_set<std::string> blockedTemporaryFenceIds;
          std::vector<VectorObstacle> obstacles;

          try {
              const bool loadRisks = mode == RouteMode::SAFEST || mode == RouteMode::BALANCED;
              const auto cached = getDatabaseObstacleCache(db, loadRisks);
              graphTemporaryFenceIds = cached->graphTemporaryFenceIds;
              obstacles = cached->airSpace;

              for (const auto& fence : cached->fences) {
                  const bool temporary = graphTemporaryFenceIds.count(fence.id) != 0;
                  if (temporary && !isTemporaryFenceActive(fence.timeRestrictions, time)) {
                      continue;
                  }
                  if (temporary) blockedTemporaryFenceIds.insert(fence.id);
                  if (!fence.geometry.vertices.empty()) {
                      obstacles.push_back(fence.geometry);
                  }
              }

              if (loadRisks) {
                  for (const auto& risk : cached->risks) {
                      const int riskLevel = riskLevelAtTime(
                          risk.emergencyDate,
                          risk.emergencyLowRiskTime,
                          risk.emergencyMidRiskTime,
                          risk.emergencyHighRiskTime,
                          risk.workdayLowRiskTime,
                          risk.workdayMidRiskTime,
                          risk.workdayHighRiskTime,
                          risk.weekendLowRiskTime,
                          risk.weekendMidRiskTime,
                          risk.weekendHighRiskTime,
                          time);
                      if (!modeBlocksRisk(mode, riskLevel)) continue;
                      blockedRiskIds.insert(risk.id);
                      if (!risk.geometry.vertices.empty()) {
                          obstacles.push_back(risk.geometry);
                      }
                  }
              }
          } catch (const std::exception& error) {
              LOG_WARN << "[VisibilityGraph] 障碍物规则读取失败，保留原始途径点: "
                       << error.what();
              return;
          }

          // 本次请求的障碍集合在所有航段和后续抽稀中保持不变。
          thinningObstacles = obstacles;
          const auto visible = [&obstacles](
              const VisibilityPoint& from,
              const VisibilityPoint& to) {
              if (pointInAnyObstacle2D(from.longitude, from.latitude, obstacles) ||
                  pointInAnyObstacle2D(to.longitude, to.latitude, obstacles)) {
                  return false;
              }
              return !segmentHitsAnyObstacle2D(
                  from.longitude, from.latitude, to.longitude, to.latitude, obstacles);
          };

          std::vector<RouteWaypoint> result;
          auto appendWaypoint = [&result](const RouteWaypoint& waypoint) {
              if (!result.empty() && result.back().grid == waypoint.grid) {
                  // 同一网格只要有一个是前端点，就必须保留。
                  result.back().mustPass =
                      result.back().mustPass || waypoint.mustPass;
                  return;
              }

              result.push_back(waypoint);
          };
          appendWaypoint(waypoints.front());
          for (size_t segment = 0; segment + 1 < waypoints.size(); ++segment) {
              const auto& startWaypoint = waypoints[segment];
              const auto& goalWaypoint = waypoints[segment + 1];
              const auto& startGrid = startWaypoint.grid;
              const auto& goalGrid = goalWaypoint.grid;
              const IJH startIjh = {
                  static_cast<uint32_t>(startGrid[1]),
                  static_cast<uint32_t>(startGrid[0]),
                  static_cast<uint32_t>(startGrid[2])};
              const IJH goalIjh = {
                  static_cast<uint32_t>(goalGrid[1]),
                  static_cast<uint32_t>(goalGrid[0]),
                  static_cast<uint32_t>(goalGrid[2])};
              const LatLonHei startCoordinate = getLocalTileLatLon(
                  rchToCode(startIjh, static_cast<uint8_t>(level)), baseTile);
              const LatLonHei goalCoordinate = getLocalTileLatLon(
                  rchToCode(goalIjh, static_cast<uint8_t>(level)), baseTile);

              const VisibilityPlanResult plan = VisibilityGraphPlanner::instance().plan(
                  db,
                  {startCoordinate.longitude, startCoordinate.latitude},
                  {goalCoordinate.longitude, goalCoordinate.latitude},
                  blockedRiskIds,
                  blockedTemporaryFenceIds,
                  visible);
              if (!plan.success) {
                  LOG_WARN << "[VisibilityGraph] 航段 " << segment
                           << " 未生成绕行引导，交由原A*处理: " << plan.reason;
                  appendWaypoint(goalWaypoint);
                  continue;
              }

              // 可见候选点只有经纬度，沿用当前航段作业层，不查询或写入地形高度。
              for (size_t index = 1; index + 1 < plan.path.size(); ++index) {
                  const auto& candidate = plan.path[index];
                  const IJH candidateIjh = localRowColHeiNumber(
                      static_cast<uint8_t>(level),
                      candidate.longitude,
                      candidate.latitude,
                      startCoordinate.height,
                      baseTile);
                  const std::array<int, 3> candidateGrid = {
                      static_cast<int>(candidateIjh.column),
                      static_cast<int>(candidateIjh.row),
                      startGrid[2]};
                  if (candidateGrid != goalGrid) {
                      appendWaypoint(RouteWaypoint{candidateGrid, false});
                  }
              }
              appendWaypoint(goalWaypoint);
          }
          waypoints = std::move(result);
          LOG_INFO << "[VisibilityGraph] 预处理完成，风险阻挡=" << blockedRiskIds.size()
                   << "，临时禁飞阻挡=" << blockedTemporaryFenceIds.size()
                   << "，途径点=" << waypoints.size();
      }
  }
namespace {

struct SegmentRequest {
    std::string from;
    std::string to;
    double startTime = 0.0;
    bool skipBufferRuleCheck = false;
};

double gridDistanceMeters(const std::string& fromCode, const std::string& toCode, int level) {
    const IJH from = getLocalTileRHC(fromCode);
    const IJH to = getLocalTileRHC(toCode);
    const double dx = static_cast<double>(to.column) - from.column;
    const double dy = static_cast<double>(to.row) - from.row;
    const double dz = static_cast<double>(to.layer) - from.layer;
    return std::sqrt(dx * dx + dy * dy + dz * dz) * getGridSize(level);
}

std::string routeRuleKey(const std::string& code, int time) {
    return code + "|" + std::to_string(time);
}

std::string routeSegmentKey(const SegmentRequest& segment) {
    // 时间相关校验有方向；保留亚秒起始时间，不能复用反向线段结果。
    std::ostringstream key;
    key << segment.from << ">" << segment.to << "|"
        << std::setprecision(17) << segment.startTime << "|"
        << (segment.skipBufferRuleCheck ? "skip-buffer" : "full-buffer");
    return key.str();
}

bool routeHeightSafe(const std::string& code, const BaseTile& baseTile,
                     RouteValidationContext& context) {
    const auto cached = context.heightSafeCache.find(code);
    if (cached != context.heightSafeCache.end()) return cached->second;
    const LatLonHei point = getLocalTileLatLon(code, baseTile);
    const double ground = TiffReader::getInstance().getElevation(point.longitude, point.latitude);
    // 保留无地形数据时的降级行为，不把无效高程当作零海拔。
    const bool unavailable = !std::isfinite(ground) || ground == TiffReader::INVALID_ELEVATION;
    const double height = point.height - ground;
    const bool safe = unavailable || (std::isfinite(height) && height >= 15.0 && height <= 120.0);
    context.heightSafeCache.emplace(code, safe);
    return safe;
}

double routePenalty(
    const GridEvaluator::CheckResult& result,
    const RouteWeights& weights) {
    return weights.comm * result.commCost +
           weights.nav * result.navCost +
           weights.surv * result.survCost +
           weights.wind * result.windCost +
           weights.rain * result.rainCost +
           weights.vis * result.visCost +
           weights.temp * result.tempCost +
           weights.hum * result.humCost +
           weights.press * result.pressCost +
           weights.em * result.emCost +
           weights.riskArea * result.riskCost +
           weights.privacy * result.privacyCost;
}

void ensureSphericalMask(
    RouteValidationContext& context,
    uint8_t level,
    double planeRadius) {
    if (context.maskLevel == level &&
        std::abs(context.maskRadius - planeRadius) < 1e-9 &&
        !context.sphericalMask.empty()) {
        return;
    }

    context.sphericalMask.clear();
    context.maskLevel = level;
    context.maskRadius = planeRadius;

    const double gridSize = getGridSize(level);
    if (planeRadius > 0.0) {
        const int extendCell = static_cast<int>(std::ceil(planeRadius / gridSize));
        for (int dx = -extendCell; dx <= extendCell; ++dx) {
            for (int dy = -extendCell; dy <= extendCell; ++dy) {
                for (int dz = -extendCell; dz <= extendCell; ++dz) {
                    const double distance =
                        std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize;
                    if (distance <= planeRadius) {
                        context.sphericalMask.push_back({dx, dy, dz});
                    }
                }
            }
        }
    }

    if (context.sphericalMask.empty()) {
        context.sphericalMask.push_back({0, 0, 0});
    }

    // 与普通A*保持一致：设置半径时至少扩展一格缓冲区。
    if (planeRadius > 0.0 && context.sphericalMask.size() == 1) {
        context.sphericalMask.clear();
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    context.sphericalMask.push_back({dx, dy, dz});
                }
            }
        }
    }
}


struct PreparedSegment {
    SegmentRequest request;
    SegmentCheckResult result;
    std::vector<std::string> ruleKeys;
    std::vector<std::pair<std::string, double>> costSamples;
};

/// 网格中心连线在指定网格内的参数区间，用于实际经过时间和距离积分。
std::pair<double, double> cellParameterInterval(
    const IJH& from, const IJH& to, const IJH& cell) {
    const std::array<double, 3> a{
        static_cast<double>(from.column), static_cast<double>(from.row), static_cast<double>(from.layer)};
    const std::array<double, 3> b{
        static_cast<double>(to.column), static_cast<double>(to.row), static_cast<double>(to.layer)};
    const std::array<double, 3> c{
        static_cast<double>(cell.column), static_cast<double>(cell.row), static_cast<double>(cell.layer)};
    double enter = 0.0;
    double leave = 1.0;
    for (size_t axis = 0; axis < 3; ++axis) {
        const double delta = b[axis] - a[axis];
        if (delta == 0.0) {
            if (std::abs(a[axis] - c[axis]) > 0.5) return {1.0, 0.0};
            continue;
        }
        double low = (c[axis] - 0.5 - a[axis]) / delta;
        double high = (c[axis] + 0.5 - a[axis]) / delta;
        if (low > high) std::swap(low, high);
        enter = std::max(enter, low);
        leave = std::min(leave, high);
    }
    return {enter, leave};
}

/// A* 扩展和抽稀共用：线段障碍、中心真高、扫过缓冲区及逐秒规则。
Task<std::vector<SegmentCheckResult>> checkRouteSegments(
    const std::vector<SegmentRequest>& requests,
    const std::shared_ptr<GridEvaluator>& evaluator,
    uint8_t level,
    double planeRadius,
    double speed,
    bool enableTrueHeightCheck,
    RouteMode mode,
    const std::vector<VectorObstacle>& obstacles,
    RouteValidationContext& context) {
    const BaseTile& baseTile = ::getProjectBaseTile();
    ensureSphericalMask(context, level, planeRadius);
    // 长距离搜索只限制校验缓存，不丢弃搜索节点或改变通行结论。
    if (context.checkedRuleKeys.size() > 100000) {
        context.checkedRuleKeys.clear();
        context.ruleCache.clear();
    }
    if (context.segmentCache.size() > 20000) context.segmentCache.clear();
    const RouteWeights weights = getWeightsByMode(evaluator ? mode : RouteMode::ORIGINAL);
    // GridEvaluator 的返回键只有 code，因此不同时间必须分批，不能互相覆盖。
    std::map<int, std::unordered_map<std::string, CandidateInfo>> pending;
    std::vector<PreparedSegment> prepared;
    prepared.reserve(requests.size());

    for (const auto& request : requests) {
        PreparedSegment segment;
        segment.request = request;
        const auto cached = context.segmentCache.find(routeSegmentKey(request));
        if (cached != context.segmentCache.end()) {
            segment.result = cached->second;
            prepared.push_back(std::move(segment));
            continue;
        }
        const LatLonHei from = getLocalTileLatLon(request.from, baseTile);
        const LatLonHei to = getLocalTileLatLon(request.to, baseTile);
        if (pointInAnyObstacle2D(from.longitude, from.latitude, obstacles) ||
            pointInAnyObstacle2D(to.longitude, to.latitude, obstacles) ||
            segmentHitsAnyObstacle2D(from.longitude, from.latitude, to.longitude, to.latitude, obstacles)) {
            segment.result.reason = "矢量障碍物阻挡线段";
            prepared.push_back(std::move(segment));
            continue;
        }
        segment.result.distance = gridDistanceMeters(request.from, request.to, level);
        const double duration = segment.result.distance / speed;
        if (!std::isfinite(request.startTime) || !std::isfinite(duration) || duration < 0.0 ||
            request.startTime < std::numeric_limits<int>::min() ||
            request.startTime + duration >= std::numeric_limits<int>::max()) {
            segment.result.reason = "航段时间超出规则支持范围";
            prepared.push_back(std::move(segment));
            continue;
        }
        const IJH fromGrid = getLocalTileRHC(request.from);
        const IJH toGrid = getLocalTileRHC(request.to);
        const auto cells = singleLineToGrids2({
            {from.longitude, from.latitude, from.height},
            {to.longitude, to.latitude, to.height}}, level, baseTile);
        if (cells.empty() || cells.front() != request.from || cells.back() != request.to) {
            segment.result.reason = "线段网格化未覆盖起终点";
            prepared.push_back(std::move(segment));
            continue;
        }
        segment.result.safe = true;
        for (const auto& code : cells) {
            const IJH center = getLocalTileRHC(code);
            const auto [enter, leave] = cellParameterInterval(fromGrid, toGrid, center);
            if (enter > leave + 1e-9) {
                segment.result.safe = false;
                segment.result.reason = "线段网格化产生不相交网格: " + code;
                break;
            }
            // 真高限制针对航迹中心；缓冲区仅参与障碍和规则检查，二者不混用。
            if (enableTrueHeightCheck && !routeHeightSafe(code, baseTile, context)) {
                segment.result.safe = false;
                segment.result.reason = "真高不在15至120米范围: " + code;
                break;
            }
            if (!evaluator) continue;
            const double enterTime = request.startTime + enter * duration;
            const double leaveTime = request.startTime + std::max(enter, leave) * duration;
            const int firstSecond = static_cast<int>(std::floor(enterTime));
            const int lastSecond = static_cast<int>(std::floor(leaveTime));
            // 在规则的秒精度内覆盖整个占用区间，防止长快捷线跨过时段边界。
            for (int second = firstSecond; second <= lastSecond; ++second) {
                const auto normalized = normalizeGridTime(second, context.referenceTime);
                for (const auto& offset : context.sphericalMask) {
                    const int64_t x = static_cast<int64_t>(center.column) + offset[0];
                    const int64_t y = static_cast<int64_t>(center.row) + offset[1];
                    const int64_t z = static_cast<int64_t>(center.layer) + offset[2];
                    // 沿用既有负坐标裁剪语义，避免生成无效编码。
                    if (x < 0 || y < 0 || z < 0) continue;
                    const uint64_t maxCoord = level >= 21
                        ? std::numeric_limits<uint64_t>::max() : (1ull << (3u * level));
                    if (static_cast<uint64_t>(x) >= maxCoord ||
                        static_cast<uint64_t>(y) >= maxCoord ||
                        static_cast<uint64_t>(z) >= maxCoord) continue;
                    const std::string expanded = rchToCode(
                        {static_cast<uint32_t>(y), static_cast<uint32_t>(x), static_cast<uint32_t>(z)}, level);
                    const std::string key = routeRuleKey(expanded, second);
                    segment.ruleKeys.push_back(key);
                    if (context.checkedRuleKeys.count(key) == 0) {
                        pending[second].emplace(expanded, CandidateInfo{
                            expanded, second, normalized.wdTime, normalized.wdRule, true});
                    }
                }
                // 软代价按中心航迹在当前秒内的距离积分，不参与硬性禁行判定。
                const double secondsInCell = std::max(0.0,
                    std::min(leaveTime, static_cast<double>(second) + 1.0) -
                    std::max(enterTime, static_cast<double>(second)));
                segment.costSamples.emplace_back(routeRuleKey(code, second), secondsInCell * speed);
            }
        }
        segment.result.cost = segment.result.distance *
            (weights.distance + (fromGrid.layer != toGrid.layer ? weights.efficiency : 0.0));
        prepared.push_back(std::move(segment));
    }

    constexpr size_t batchSize = 2048;
    for (const auto& [second, byCode] : pending) {
        std::vector<CandidateInfo> batch;
        batch.reserve(std::min(batchSize, byCode.size()));
        auto flush = [&]() -> Task<void> {
            const auto results = co_await GridCheckAwaiter{evaluator, batch};
            for (const auto& candidate : batch) {
                const std::string key = routeRuleKey(candidate.code, second);
                context.checkedRuleKeys.insert(key);
                if (results) {
                    const auto found = results->find(candidate.code);
                    if (found != results->end()) context.ruleCache[key] = found->second;
                }
            }
            batch.clear();
            co_return;
        };
        for (const auto& [code, candidate] : byCode) {
            batch.push_back(candidate);
            if (batch.size() == batchSize) co_await flush();
        }
        if (!batch.empty()) co_await flush();
    }

    std::vector<SegmentCheckResult> results;
    results.reserve(prepared.size());
    for (auto& segment : prepared) {
        if (segment.result.safe && !segment.request.skipBufferRuleCheck) {
            for (const auto& key : segment.ruleKeys) {
                const auto rule = context.ruleCache.find(key);
                if (rule == context.ruleCache.end() || !rule->second.pass) {
                    segment.result.safe = false;
                    segment.result.reason = rule == context.ruleCache.end()
                        ? "规则检查缺少结果: " + key
                        : "规则禁止通行: " + key + " " + rule->second.reason;
                    break;
                }
            }
        }
        if (segment.result.safe) {
            for (const auto& [key, length] : segment.costSamples) {
                const auto rule = context.ruleCache.find(key);
                if (rule == context.ruleCache.end()) {
                    segment.result.safe = false;
                    segment.result.reason = "规则检查缺少结果: " + key;
                    break;
                }
                segment.result.cost += length * routePenalty(rule->second, weights);
            }
        }
        context.segmentCache[routeSegmentKey(segment.request)] = segment.result;
        results.push_back(segment.result);
    }
    co_return results;
}

}  // namespace
// === A* 核心逻辑 (协程版 - 有约束条件) ===
Task<AStarResult> aStarPath(
    array<int, 3> start,
    array<int, 3> end,
    double startTime,
    double planeRadius,
    const AStarOptions& options,
    int level,
    std::shared_ptr<GridEvaluator> evaluator,
    int workLayer,
    RouteMode routeMode,
    bool enableTrueHeightCheck,
    const std::vector<VectorObstacle>& obstacles,
    RouteValidationContext& validation
) {
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

    // --- 边界保护：防止 rchToCode 因坐标越界导致内存分配崩溃 ---
    int gridMax = (1 << level) - 1;
    if (sx > gridMax || sy > gridMax || ex > gridMax || ey > gridMax) {
        co_return {false, {}, "坐标超出网格边界(gridMax=" + std::to_string(gridMax) + ")"};
    }
    if (sz < 0 || ez < 0) {
        co_return {false, {}, "高度层不能为负数"};
    }
    // ----------------------------------------------------------

    RouteWeights weights = getWeightsByMode(evaluator ? routeMode : RouteMode::ORIGINAL);

    IJH startIJH = {(uint32_t)sy, (uint32_t)sx, (uint32_t)sz};
    std::string startCode = rchToCode(startIJH, static_cast<uint8_t>(level));
    IJH endIJH = {(uint32_t)ey, (uint32_t)ex, (uint32_t)ez};
    std::string endCode = rchToCode(endIJH, static_cast<uint8_t>(level));

    // 保持原分支语义：起点只做真高校验，首轮扩展不做球形缓冲区硬拦截。
    if (enableTrueHeightCheck && !routeHeightSafe(startCode, baseTile, validation)) {
        co_return AStarResult{false, {}, "起点真高不在15至120米范围"};
    }
    if (enableTrueHeightCheck && !routeHeightSafe(endCode, baseTile, validation)) {
        co_return AStarResult{false, {}, "终点真高不在15至120米范围"};
    }

    // ==========================================
    // 2. A* 主循环 (引入运动学约束与终点特权)
    // ==========================================
    double dx2 = sx - ex, dy2 = sy - ey, dz2 = sz - ez;
    double lineLength = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

    // [核心修改 1]：动态启发式权重 (Weighted A*)
    // 默认 1.3 用于打破平衡；如果是 safest 模式，大幅提高权重以抵消巨大的 g(n) 惩罚

    double hWeight = evaluator ? 1.5 : 1.3;
    if (evaluator && routeMode == RouteMode::SAFEST) {
        hWeight = 1.5;
    } else if (evaluator && routeMode == RouteMode::BALANCED) {
        hWeight = 1.5;
    } else if (evaluator && routeMode == RouteMode::SHORTEST) {
        hWeight =1.5;
    }

    auto heuristic = [&](int x, int y, int z) {
        double dx = x - ex, dy = y - ey, dz = z - ez;
        return std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize * hWeight;
    };

    struct Node {
        int x, y, z;
        double g, h, f;
        double arrivalTime;
        GridKey key;



        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, double at_)
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

        struct NeighborMeta {
            int x, y, z;
            std::string code;
        };
        std::vector<NeighborMeta> neighbors;
        std::vector<SegmentRequest> requests;
        neighbors.reserve(26);
        requests.reserve(26);
        const std::string currentCode = rchToCode(
            {static_cast<uint32_t>(cur.y), static_cast<uint32_t>(cur.x),
             static_cast<uint32_t>(cur.z)}, static_cast<uint8_t>(level));
        const uint64_t maxCoord = level >= 21
            ? std::numeric_limits<uint64_t>::max() : (1ull << (3u * level));
        for (const auto& direction : DIRECTIONS) {
            const int nx = cur.x + direction[0];
            const int ny = cur.y + direction[1];
            const int nz = cur.z + direction[2];
            if (nx < 0 || ny < 0 || nz < 0) continue;
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;
            if (closedSet.count({nx, ny, nz})) continue;
            const std::string code = rchToCode(
                {static_cast<uint32_t>(ny), static_cast<uint32_t>(nx),
                 static_cast<uint32_t>(nz)}, static_cast<uint8_t>(level));
            neighbors.push_back({nx, ny, nz, code});
            requests.push_back({
                currentCode, code, cur.arrivalTime,
                cur.key == startKey});
        }
        const auto checks = co_await checkRouteSegments(
            requests, evaluator, static_cast<uint8_t>(level), planeRadius,
            options.speed, enableTrueHeightCheck, routeMode, obstacles, validation);
        for (size_t index = 0; index < neighbors.size(); ++index) {
            const auto& check = checks[index];
            if (!check.safe) {
                lastFailReason = check.reason;
                continue;
            }
            const auto& nb = neighbors[index];
            const double newG = cur.g + check.cost;
            const GridKey nKey{nb.x, nb.y, nb.z};
            const auto existing = openMap.find(nKey);
            if (existing != openMap.end() && newG >= existing->second.g) continue;

            const double newH = heuristic(nb.x, nb.y, nb.z) * weights.distance;
            // 全程保留亚秒精度，只在规则查询和响应输出时转换成整数秒。
            Node next(nb.x, nb.y, nb.z, newG, newH,
                      cur.arrivalTime + check.distance / options.speed);
            openSet.push(next);
            openMap[nKey] = next;
            parent[nKey] = cur.key;
        }
    }

    co_return {false, {}, lastFailReason};
}
//----------------------------抽稀函数--------------------------------------
namespace {

Task<AStarResult> thinPathGreedy(
    const std::vector<std::string>& originalPath,
    const std::shared_ptr<GridEvaluator>& evaluator,
    double startTime,
    uint8_t level,
    double planeRadius,
    double speed,
    bool enableTrueHeightCheck,
    RouteMode currentMode,
    const std::vector<VectorObstacle>& obstacles,
    RouteValidationContext& context) {
    if (originalPath.empty()) {
        co_return AStarResult{false, {}, "抽稀输入路径为空"};
    }

    auto runPass = [&](const std::vector<std::string>& path, bool halve) -> Task<AStarResult> {
        std::vector<std::string> output{path.front()};
        size_t currentIndex = 0;
        double currentTime = startTime;
        while (currentIndex + 1 < path.size()) {
            // 使用同一出发时间比较快捷线与被替换子路径的积分代价。
            std::vector<SegmentRequest> baselineRequests;
            double baselineTime = currentTime;
            for (size_t index = currentIndex; index + 1 < path.size(); ++index) {
                baselineRequests.push_back({
                    path[index], path[index + 1], baselineTime,
                    currentIndex == 0 && index == 0});
                baselineTime += gridDistanceMeters(path[index], path[index + 1], level) / speed;
            }
            const auto baseline = co_await checkRouteSegments(
                baselineRequests, evaluator, level, planeRadius, speed,
                enableTrueHeightCheck, currentMode, obstacles, context);
            std::vector<double> prefixCost(baseline.size() + 1, 0.0);
            std::vector<bool> prefixSafe(baseline.size() + 1, true);
            for (size_t index = 0; index < baseline.size(); ++index) {
                prefixCost[index + 1] = prefixCost[index] + baseline[index].cost;
                prefixSafe[index + 1] = prefixSafe[index] && baseline[index].safe;
            }

            size_t targetIndex = path.size() - 1;
            bool found = false;
            std::string lastReason;
            while (targetIndex > currentIndex) {
                const std::vector<SegmentRequest> shortcutRequests{
                    {path[currentIndex], path[targetIndex], currentTime,
                     currentIndex == 0 && targetIndex == 1}};
                const auto checks = co_await checkRouteSegments(
                    shortcutRequests, evaluator, level, planeRadius, speed,
                    enableTrueHeightCheck, currentMode, obstacles, context);
                const auto& shortcut = checks.front();
                const size_t replacedCount = targetIndex - currentIndex;
                // 不把软代价当作禁止通行；仅拒绝比可行原子路径更差的替换。
                const bool acceptableCost = !prefixSafe[replacedCount] ||
                    shortcut.cost <= prefixCost[replacedCount] + 1e-5;
                if (shortcut.safe && acceptableCost) {
                    found = true;
                    currentTime += shortcut.distance / speed;
                    break;
                }
                lastReason = shortcut.safe ? "快捷线累计代价高于原子路径" : shortcut.reason;
                if (targetIndex == currentIndex + 1) break;
                targetIndex = halve
                    ? currentIndex + std::max<size_t>(1, (targetIndex - currentIndex) / 2)
                    : targetIndex - 1;
            }
            if (!found) {
                co_return AStarResult{false, {}, "抽稀连接失败: " + lastReason};
            }
            output.push_back(path[targetIndex]);
            currentIndex = targetIndex;
        }
        co_return AStarResult{true, std::move(output), ""};
    };

    auto first = co_await runPass(originalPath, true);
    if (!first.success) {
        // 快捷线改变后续到达时间后，原后缀可能不可行；回退必须重新验证。
        std::vector<SegmentRequest> requests;
        double time = startTime;
        for (size_t index = 0; index + 1 < originalPath.size(); ++index) {
            requests.push_back({
                originalPath[index], originalPath[index + 1], time,
                index == 0});
            time += gridDistanceMeters(originalPath[index], originalPath[index + 1], level) / speed;
        }
        if (requests.empty()) requests.push_back({originalPath.front(), originalPath.front(), time});
        const auto checks = co_await checkRouteSegments(
            requests, evaluator, level, planeRadius, speed,
            enableTrueHeightCheck, currentMode, obstacles, context);
        for (const auto& check : checks) {
            if (!check.safe) {
                LOG_WARN << "[A*] " << first.reason << "；原路径复检失败: " << check.reason;
                co_return AStarResult{false, {}, "原路径复检失败: " + check.reason};
            }
        }
        LOG_WARN << "[A*] " << first.reason << "；保留已通过统一校验的原路径";
        co_return AStarResult{true, originalPath, ""};
    }
    if (first.path.size() <= 2) co_return first;
    auto refined = co_await runPass(first.path, false);
    // 第二轮可能改变时间分布；失败时第一轮结果仍按其实际时间通过了校验。
    if (!refined.success) {
        LOG_WARN << "[A*] 抽稀精查失败，保留已验证第一轮结果: " << refined.reason;
        co_return first;
    }
    co_return refined;
}

}  // namespace
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
        vector<RouteWaypoint> waypoints;

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
            waypoints.push_back(RouteWaypoint{
                {
                    static_cast<int>(ijh.column),
                    static_cast<int>(ijh.row),
                    layer
                },
                true
            });
        }

        long long rawStartTime = (*jsonBody).get("startTime", static_cast<Json::Int64>(getBeijingTime())).asInt64();
        int startTime = (rawStartTime > 9999999999LL) ? static_cast<int>(rawStartTime / 1000) : rawStartTime;

        if (jsonBody->isMember("planeRadius") && !(*jsonBody)["planeRadius"].isNumeric()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "planeRadius 必须是数值";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            co_return;
        }
        const double planeRadius = (*jsonBody).get("planeRadius", 0.75).asDouble();
        if (!std::isfinite(planeRadius) || planeRadius < 0.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "planeRadius 必须是非负有限数值";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            co_return;
        }

        // speed 统一表示飞行速度，单位为米/秒。
        if (jsonBody->isMember("speed") && !(*jsonBody)["speed"].isNumeric()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "speed 必须是数值";

            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            co_return;
        }

        const double speed = (*jsonBody).get("speed", 15.0).asDouble();
        if (!std::isfinite(speed) || speed <= 0.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "speed 必须是大于0的有限数值";

            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            co_return;
        }

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
            waypoints[i].grid[2] = static_cast<int>(wpIJH.layer);
        }

        // 为了兼容后续调用 A* 时传入的 workLayer 参数，将其指向起点的作业层
        int workLayer = startWorkLayer;

        AStarOptions options;
        options.speed = speed;

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
        vector<size_t> mustPassIndices;
        bool pathSuccess = true;
        string failReason;

        std::shared_ptr<GridEvaluator> gridEvaluator = nullptr;
        if (!isUnconstrained) {
            gridEvaluator = GridEvaluator::create(ruleOptions);
        }
        LOG_INFO << "[A*] 开始数据库可见图绕行预处理";
        std::vector<VectorObstacle> thinningObstacles;
        insertDatabaseVisibilityWaypoints(
            waypoints, level, baseTile, currentMode, startTime,
            thinningObstacles);
        RouteValidationContext validation;
        double cruiseStartTime = startTime;
        for (size_t index = 0; index + 1 < verticalPath.size(); ++index) {
            cruiseStartTime += gridDistanceMeters(verticalPath[index], verticalPath[index + 1], level) / speed;
        }
        double currentSegmentStartTime = cruiseStartTime;

        for (size_t i = 0; i < waypoints.size() - 1 && pathSuccess; ++i) {
            LOG_INFO << "[A*] 航段 " << i + 1 << " 使用"
                     << (isUnconstrained ? "无约束模式" : "约束模式")
                     << "（协程版A*）";
            AStarResult segmentResult = co_await aStarPath(
                waypoints[i].grid, waypoints[i + 1].grid,
                currentSegmentStartTime, planeRadius, options, level, gridEvaluator,
                workLayer, currentMode, enableTrueHeightCheck,
                thinningObstacles, validation);

            if (!segmentResult.success) {
                pathSuccess = false;
                failReason = segmentResult.reason;
                break;
            }
            if (segmentResult.path.empty()) {
                pathSuccess = false;
                failReason = "A*返回了空路径";
                break;
            }

            for (size_t j = 0; j + 1 < segmentResult.path.size(); ++j) {
                currentSegmentStartTime += gridDistanceMeters(
                    segmentResult.path[j], segmentResult.path[j + 1], level) / options.speed;
            }

            if (i == 0) {
                fullPath = segmentResult.path;
                if (waypoints.front().mustPass) {
                    mustPassIndices.push_back(0);
                }
            } else {
                fullPath.insert(
                    fullPath.end(),
                    segmentResult.path.begin() + 1,
                    segmentResult.path.end());
            }

            if (waypoints[i + 1].mustPass) {
                const size_t index = fullPath.size() - 1;
                if (mustPassIndices.empty() || mustPassIndices.back() != index) {
                    mustPassIndices.push_back(index);
                }
            }
        }


        if (pathSuccess && applySmoothing && fullPath.size() > 2) {
            LOG_INFO << "[A*] 开始对完整分段航线按前端必经点整体抽稀";
            std::vector<std::string> smoothedPath;
            std::vector<int> smoothedIndexes;
            std::vector<std::string> frontendWaypointCodes;
            double sectionStartTime = cruiseStartTime;

            for (size_t sectionIndex = 0;
                 sectionIndex + 1 < mustPassIndices.size();
                 ++sectionIndex) {
                const size_t begin = mustPassIndices[sectionIndex];
                const size_t end = mustPassIndices[sectionIndex + 1];
                std::vector<std::string> section(
                    fullPath.begin() + begin,
                    fullPath.begin() + end + 1);

                // 有向时间规则按实际飞行方向计算，不能倒转航线后复用结果。
                auto smoothed = co_await thinPathGreedy(
                    section, gridEvaluator, sectionStartTime,
                    static_cast<uint8_t>(level), planeRadius, options.speed,
                    enableTrueHeightCheck, currentMode,
                    thinningObstacles, validation);
                if (!smoothed.success) {
                    pathSuccess = false;
                    failReason = smoothed.reason;
                    break;
                }
                const auto& selectedPath = smoothed.path;

                const size_t appendBegin = smoothedPath.empty() ? 0 : 1;
                for (size_t index = appendBegin; index < selectedPath.size(); ++index) {
                    smoothedPath.push_back(selectedPath[index]);
                    smoothedIndexes.push_back(static_cast<int>(sectionIndex + 1));
                }

                if (sectionIndex + 1 < mustPassIndices.size() - 1 &&
                    !selectedPath.empty()) {
                    frontendWaypointCodes.push_back(selectedPath.back());
                }

                for (size_t index = 0; index + 1 < selectedPath.size(); ++index) {
                    sectionStartTime += gridDistanceMeters(
                        selectedPath[index], selectedPath[index + 1], level) / options.speed;
                }
            }

            fullPath = std::move(smoothedPath);
            pathIndexes = std::move(smoothedIndexes);
            waypointCodes = std::move(frontendWaypointCodes);
        } else if (pathSuccess) {
            pathIndexes.assign(fullPath.size(), 1);
            waypointCodes.clear();

            for (size_t sectionIndex = 0;
                 sectionIndex + 1 < mustPassIndices.size();
                 ++sectionIndex) {
                size_t begin = mustPassIndices[sectionIndex];
                const size_t end = mustPassIndices[sectionIndex + 1];
                if (sectionIndex > 0) {
                    ++begin;
                }
                for (size_t index = begin;
                     index <= end && index < pathIndexes.size();
                     ++index) {
                    pathIndexes[index] = static_cast<int>(sectionIndex + 1);
                }
            }

            for (size_t index = 1; index + 1 < mustPassIndices.size(); ++index) {
                waypointCodes.push_back(fullPath[mustPassIndices[index]]);
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
                int landingIndex = static_cast<int>(pointsArr.size());
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
