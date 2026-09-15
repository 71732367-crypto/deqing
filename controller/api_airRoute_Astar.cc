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
#include <cstdlib>
#include <chrono>
#include <mutex>
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
/*
// 牛顿迭代法求平方根，用于计算欧几里得距离
static double newton(double num, int iters = 5) {
    if (num <= 0) return 0.0;
    double x = num / 2.0;
    for (int i = 0; i < iters; ++i) x = 0.5 * (x + num / x);
    return x;
}
*/

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
          std::vector<std::array<int, 3>>& waypoints,
          int level,
          const BaseTile& baseTile,
          RouteMode mode,
          int startTime) {
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

          // 本次请求的障碍集合在所有航段中保持不变。
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

          std::vector<std::array<int, 3>> result;
          result.push_back(waypoints.front());
          for (size_t segment = 0; segment + 1 < waypoints.size(); ++segment) {
              const auto& startGrid = waypoints[segment];
              const auto& goalGrid = waypoints[segment + 1];
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
                  result.push_back(goalGrid);
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
                  if (candidateGrid != result.back() && candidateGrid != goalGrid) {
                      result.push_back(candidateGrid);
                  }
              }
              result.push_back(goalGrid);
          }
          waypoints = std::move(result);
          LOG_INFO << "[VisibilityGraph] 预处理完成，风险阻挡=" << blockedRiskIds.size()
                   << "，临时禁飞阻挡=" << blockedTemporaryFenceIds.size()
                   << "，途径点=" << waypoints.size();
      }
  }
// === A* 核心逻辑 (简化版 - 无约束条件) ===
AStarResult aStarPathSimple(
    array<int, 3> start,
    array<int, 3> end,
    const AStarOptions& options,
    int level,
    int workLayer,
    bool enableTrueHeightCheck
) {
    double gridSize;
    try {
        gridSize = getGridSize(level);
    } catch (const exception& e) {
        return {false, {}, string("不支持的 level: ") + to_string(level)};
    }
    // === [新增] 获取全局基础瓦片 ===
    const BaseTile& baseTile = ::getProjectBaseTile();
    int sx = start[0], sy = start[1], sz = start[2];
    int ex = end[0], ey = end[1], ez = end[2];

    if (sx < 0 || sy < 0 ||  ex < 0 || ey < 0 ) {
        return {false, {}, "行列坐标不能为负数"};
    }
    // --- 边界保护：防止 rchToCode 因坐标越界导致内存分配崩溃 ---
    int gridMax = (1 << level) - 1;
    if (sx > gridMax || sy > gridMax || ex > gridMax || ey > gridMax) {
        return {false, {}, "坐标超出网格边界(gridMax=" + std::to_string(gridMax) + ")"};
    }
    if (sz < 0 || ez < 0) {
        return {false, {}, "高度层不能为负数"};
    }
    // ----------------------------------------------------------
    // ==========================================
    // [新增] 起点/终点 120米真高前置校验
    // ==========================================
    auto checkNodeTrueHeight = [&](int x, int y, int z, const string& pointName) -> string {
        if (!enableTrueHeightCheck) return "";
        IJH ijh = {(uint32_t)y, (uint32_t)x, (uint32_t)z};
        string code = rchToCode(ijh, static_cast<uint8_t>(level));
        LatLonHei boundary = getLocalTileLatLon(code, baseTile);
        float ground = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
        float tHeight = boundary.height - ground;

        // 注意：真高计算 (boundary.height - ground) 天然兼容负数海拔(如水下/低洼区)
        if (tHeight > 120.0f) return pointName + "超出空域限制，超出120米真高适飞空域";
        if (tHeight < 15.0f) return pointName + "低于15米安全真高，存在撞地危险";
        return "";
    };

    string startErr = checkNodeTrueHeight(sx, sy, sz, "起点");
    if (!startErr.empty()) return {false, {}, startErr};

    string endErr = checkNodeTrueHeight(ex, ey, ez, "终点");
    if (!endErr.empty()) return {false, {}, endErr};
    // ==========================================
    double dx2 = sx - ex, dy2 = sy - ey, dz2 = sz - ez;
    double lineLength = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
    // 定义 A* 算法的启发式函数 (使用原生 std::sqrt 并加入 Tie-Breaker)
    auto heuristic = [&](int x, int y, int z) {
        double dx = x - ex, dy = y - ey, dz = z - ez;
        // 使用 std::sqrt 保证精度，乘以 1.3 的权重打破平衡，防止 A* 在空旷区盲目扩散
        return std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize * 1.3;
    };
    // A*节点结构
    struct Node {
        int x, y, z;
        double g, h, f;
        GridKey key;



        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, size_t s_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), key({x_, y_, z_}) {}

        bool operator>(const Node& o) const {
            return f > o.f; // 只需要比较 f 值，性能更好
        }
    };

    static size_t globalSeqSimple = 0;
    GridKey startKey = {sx, sy, sz}; // 起点无方向

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
    string failReason = "未找到路径"; // [新增] 追踪失败原因
    while (!openSet.empty()) {
        if (++searchSteps > MAX_SEARCH_STEPS) {
            return {false, {}, "路径计算超时: 搜索范围过大"};
        }

        Node cur = openSet.top();
        openSet.pop();

        if (closedSet.count(cur.key)) continue;

        auto it = openMap.find(cur.key);
        if (it == openMap.end() || abs(it->second.g - cur.g) > 1e-6) continue;

        // 到达终点判定：通过坐标对比，不对比方向
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
            return {true, path, ""};
        }

        closedSet.insert(cur.key);
        openMap.erase(cur.key);

        // 扩展 26 个邻居
        for (size_t i = 0; i < DIRECTIONS.size(); ++i) {
            const auto& d = DIRECTIONS[i];
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            int nz = cur.z + d[2];

            if (nx < 0 || ny < 0 || nz < 0) continue;
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;

            GridKey nKey = {nx, ny, nz};
            if (closedSet.count(nKey)) continue;
            // ==========================================
            // [新增] 简化版的物理真高安全限制
            // ==========================================
            IJH nextIJH = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
            string code = rchToCode(nextIJH, static_cast<uint8_t>(level));
            LatLonHei boundary = getLocalTileLatLon(code, baseTile);

            if (enableTrueHeightCheck) {
                float groundElevation = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
                float trueHeight = boundary.height - groundElevation;

                if (trueHeight > 120.0f) {
                    failReason = "超出空域限制，超出120米真高适飞空域";
                    continue;
                }
                if (trueHeight < 15.0f) {
                    failReason = "低于15米安全真高，存在撞地危险";
                    continue;
                }
            }
            // ==========================================

            double moveDist = DIRECTION_DISTANCES[i] * gridSize;



            double newG = cur.g + moveDist ;

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
Task<AStarResult> aStarPath(
    array<int, 3> start,
    array<int, 3> end,
    int startTime,
    double planeRadius,
    const AStarOptions& options,
    int level,
    std::shared_ptr<GridEvaluator> evaluator,
    int workLayer,
    RouteMode routeMode,
    bool enableTrueHeightCheck
) {
    int currentTime = getBeijingTime();
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

    RouteWeights weights = getWeightsByMode(routeMode);

    IJH startIJH = {(uint32_t)sy, (uint32_t)sx, (uint32_t)sz};
    std::string startCode = rchToCode(startIJH, static_cast<uint8_t>(level));
    IJH endIJH = {(uint32_t)ey, (uint32_t)ex, (uint32_t)ez};
    std::string endCode = rchToCode(endIJH, static_cast<uint8_t>(level));

    // ==========================================
    // 1. 起点/终点 基础有效性检查
    // ==========================================
    {
        // [新增] 独立真高校验，防止起终点直接违规
        auto checkNodeTrueHeight = [&](const string& code, const string& pointName) -> string {
            if (!enableTrueHeightCheck) return "";
            LatLonHei boundary = getLocalTileLatLon(code, baseTile);
            float ground = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
            float tHeight = boundary.height - ground;
            if (tHeight > 120.0f) return pointName + "超出空域限制，超出120米真高适飞空域";
            if (tHeight < 15.0f) return pointName + "低于15米安全真高，存在撞地危险";
            return "";
        };

        string startHeightErr = checkNodeTrueHeight(startCode, "起点");
        if (!startHeightErr.empty()) co_return {false, {}, startHeightErr};

        string endHeightErr = checkNodeTrueHeight(endCode, "终点");
        if (!endHeightErr.empty()) co_return {false, {}, endHeightErr};
        auto startNorm = normalizeGridTime(startTime, currentTime);
        CandidateInfo startCand = { startCode, startTime, startNorm.wdTime, startNorm.wdRule, true };
        CandidateInfo endCand = { endCode, 0, 0, "", false };

        std::vector<CandidateInfo> preCheckCands = {startCand, endCand};
        auto preCheckResultsPtr = co_await GridCheckAwaiter{evaluator, preCheckCands};

        if (preCheckResultsPtr->count(startCode)) {
            const auto& res = preCheckResultsPtr->at(startCode);
            if (!res.pass) {
                co_return {false, {}, "起点不可通行: " + res.reason};
            }
        }

        if (preCheckResultsPtr->count(endCode)) {
            const auto& res = preCheckResultsPtr->at(endCode);
            if (!res.pass) {
                co_return {false, {}, "终点不可通行: " + res.reason};
            }
        }
    }

    // ==========================================
    // 2. A* 主循环 (引入运动学约束与终点特权)
    // ==========================================
    double dx2 = sx - ex, dy2 = sy - ey, dz2 = sz - ez;
    double lineLength = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

    // [核心修改 1]：动态启发式权重 (Weighted A*)
    // 默认 1.3 用于打破平衡；如果是 safest 模式，大幅提高权重以抵消巨大的 g(n) 惩罚

    double hWeight = 1.5;
    if (routeMode == RouteMode::SAFEST) {
        hWeight = 1.5;
    } else if (routeMode == RouteMode::BALANCED) {
        hWeight = 1.5;
    } else if (routeMode == RouteMode::SHORTEST) {
        hWeight =1.5;
    }

    auto heuristic = [&](int x, int y, int z) {
        double dx = x - ex, dy = y - ey, dz = z - ez;
        return std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize * hWeight;
    };

    struct Node {
        int x, y, z;
        double g, h, f;
        int arrivalTime;
        GridKey key;



        Node() = default;
        Node(int x_, int y_, int z_, double g_, double h_, int at_)
            : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_+h_), arrivalTime(at_), key({x_, y_, z_}) {}

        bool operator>(const Node& o) const {
            return f > o.f; // 只需要比较 f 值，性能更好
        }
    };
//==================================================================================
      //球行包围避障逻辑计算
//==================================================================================
      std::vector<std::array<int,3>>sphericalMask;
      int exendCell=0;
      if (planeRadius>0) {
          exendCell =std::ceil(planeRadius/gridSize);
          for (int dx =-exendCell; dx <= exendCell; ++dx) {
              for (int dy =-exendCell; dy <= exendCell; ++dy) {
                  for (int dz = -exendCell; dz <= exendCell; ++dz) {
                      double dist=std::sqrt(dx*dx+dy*dy+dz*dz)*gridSize;
                      if (dist<=planeRadius) {
                          sphericalMask.push_back({dx,dy,dz});
                      }
                  }
              }
          }
      }
      if (sphericalMask.empty()) {
          sphericalMask.push_back({0, 0, 0});
      }
      //最少一格缓冲区
      if (planeRadius > 0 && sphericalMask.size() == 1) {
          sphericalMask.clear(); // 清空原本只有 {0,0,0} 的数组
          for (int dx = -1; dx <= 1; ++dx) {
              for (int dy = -1; dy <= 1; ++dy) {
                  for (int dz = -1; dz <= 1; ++dz) {
                      sphericalMask.push_back({dx, dy, dz});
                  }
              }
          }
      }



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

        struct NeighborMeta { int x, y, z; string code; double moveCost; int arrival; };
        vector<NeighborMeta> validNeighbors;
        vector<CandidateInfo> candidateListForChecker;
        validNeighbors.reserve(26);
        candidateListForChecker.reserve(26);

        uint64_t maxCoord = (1ULL << (3 * level));
        std::unordered_set<std::string> addedCodes;
        // 遍历 26 个方向
        for (size_t i = 0; i < DIRECTIONS.size(); ++i) {
            const auto& d = DIRECTIONS[i];
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];
            int nz = cur.z + d[2];

            if (nx < 0 || ny < 0 || nz < 0) continue;
            if (static_cast<uint64_t>(nx) >= maxCoord ||
                static_cast<uint64_t>(ny) >= maxCoord ||
                static_cast<uint64_t>(nz) >= maxCoord) continue;
            GridKey nKey = {nx, ny, nz};
            if (closedSet.count(nKey)) continue;

            double moveDist = DIRECTION_DISTANCES[i] * gridSize;
            int stepTime = static_cast<int>(moveDist / options.speed);
            int arrival = cur.arrivalTime + stepTime;

            auto norm = normalizeGridTime(arrival, currentTime);
            IJH nextIJH = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
            string code = rchToCode(nextIJH, static_cast<uint8_t>(level));

            // ==========================================
            // [新增] 120米适飞区与防撞地真高校验
            // ==========================================

            // 1. 将邻居网格编码转换为实际的经纬度和绝对高度
            LatLonHei boundary = getLocalTileLatLon(code, baseTile);

            if (enableTrueHeightCheck) {
                // 2. 从 TiffReader 获取此经纬度下的真实地面高程
                float groundElevation = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);

                // 3. 计算相对高差(真高)。
                // 算法天然兼容负数高程（如水下测绘或低洼盆地），
                // 假设无人机网格绝对高度 20m，地面海拔 -50m，真高为 20 - (-50) = 70m。
                float trueHeight = boundary.height - groundElevation;
                // 4. 适飞区判定限制：最高 120 米，最低安全距离 5 米
                float maxFlyableTrueHeight = 120.0f;
                float minSafeTrueHeight = 15.0f;

                if (trueHeight > maxFlyableTrueHeight) {
                    lastFailReason = "路径受阻: 前方超出空域限制，超出120米真高适飞空域";
                    continue;
                }
                if (trueHeight < minSafeTrueHeight) {
                    lastFailReason = "路径受阻: 前方低于15米安全真高，存在撞地危险";
                    continue;
                }
            }
            // ==========================================
            validNeighbors.push_back({nx, ny, nz, code, moveDist, arrival});





            // ==========================================
            // [修改] 叠加球形 Mask：将无人机占据的所有网格送入 Redis 检查
            // ==========================================
            for (const auto& offset: sphericalMask) {
                int mx=nx+offset[0];
                int my=ny+offset[1];
                int mz=nz+offset[2];
                if (mx<0||my<0||mz<0) continue;
                if (static_cast<uint64_t>(mx) >= maxCoord ||
                    static_cast<uint64_t>(my) >= maxCoord ||
                    static_cast<uint64_t>(mz) >= maxCoord) continue;
                IJH maskIJH = {(uint32_t)my, (uint32_t)mx, (uint32_t)mz};
                std::string maskCode = rchToCode(maskIJH, static_cast<uint8_t>(level));

                if (addedCodes.find(maskCode) == addedCodes.end()) {
                    addedCodes.insert(maskCode);
                    candidateListForChecker.push_back({maskCode, arrival, norm.wdTime, norm.wdRule, true});
                }
            }
        }

        std::shared_ptr<std::unordered_map<string, GridEvaluator::CheckResult>> checkResultsPtr;
        if (!candidateListForChecker.empty()) {
            checkResultsPtr = co_await GridCheckAwaiter{evaluator, candidateListForChecker};
        }

        // 邻居缓冲区检查：包围球有一个网格碰撞整个节点就丢弃
        bool allNeighborsPassable = true;
        bool skipNeighborBufferCheck = (cur.key == startKey);

        std::vector<NeighborMeta>passedNeighbors;

       if (!skipNeighborBufferCheck) {
           for (const auto& nb:validNeighbors) {
               bool neighborIsSafe = true;
               for (const auto&offset: sphericalMask) {
                   int mx = nb.x+offset[0],my = nb.y+offset[1],mz = nb.z+offset[2];
                   if (mx < 0 || my < 0 || mz < 0 || mx >= maxCoord || my >= maxCoord || mz >= maxCoord) continue;
                   IJH mIJH = {(uint32_t)my, (uint32_t)mx, (uint32_t)mz};
                   std::string mCode = rchToCode(mIJH, static_cast<uint8_t>(level));
                   if (checkResultsPtr && checkResultsPtr->count(mCode)) {
                       if (!checkResultsPtr->at(mCode).pass) {
                           neighborIsSafe = false;
                           lastFailReason = "碰撞: 网格 " + mCode;
                           break;
                       }
                   }else {
                       neighborIsSafe = false; // 无数据也视为不安全
                       break;
                   }
               }
               if (neighborIsSafe) {
                   passedNeighbors.push_back(nb); // 该邻居整体安全，允许飞行
               }
           }

       } else {
           passedNeighbors = validNeighbors; // 起点不校验
       }
        if (passedNeighbors.empty()) {
            continue; // 所有邻居都撞了，才会抛弃当前扩展
        }

        // ==========================================
        // 计算邻居节点的累积代价值 (融合转向惩罚)
        // ==========================================
        for (const auto& nb : passedNeighbors) {
            // 【新增】防御性编程：防止起点周围的网格因缺失数据导致 at() 抛出异常崩溃
            if (!checkResultsPtr || !checkResultsPtr->count(nb.code)) continue;
            const auto& res = checkResultsPtr->at(nb.code);

            double distanceCost = nb.moveCost;
            double altitudeChange = std::abs(nb.z - cur.z) * gridSize;
            double efficiencyCost = (altitudeChange > 0) ? 1.0 : 0.0;

            double baseG = weights.distance * distanceCost;
            double effPenalty = weights.efficiency * efficiencyCost;

            double safetyPenalty =
                (weights.comm  * res.commCost) +
                (weights.nav   * res.navCost)  +
                (weights.surv  * res.survCost) +
                (weights.wind  * res.windCost) +
                (weights.rain  * res.rainCost) +
                (weights.vis   * res.visCost)  +
                (weights.temp  * res.tempCost) +
                (weights.hum   * res.humCost)  +
                (weights.press * res.pressCost)+
                (weights.em    * res.emCost);

            double riskPenalty = weights.riskArea * res.riskCost;
            double privacyPenalty = weights.privacy * res.privacyCost;

            double totalExtraFactor = effPenalty + safetyPenalty + riskPenalty + privacyPenalty;
            double extraCost = distanceCost * totalExtraFactor;




            // 总代价值汇总
            double newG = cur.g + baseG + extraCost ;

            GridKey nKey = {nb.x, nb.y, nb.z};
            auto existing = openMap.find(nKey);
            if (existing != openMap.end() && newG >= existing->second.g) continue;

            double newH = heuristic(nb.x, nb.y, nb.z) * weights.distance;
            Node next(nb.x, nb.y, nb.z, newG, newH, nb.arrival);
            openSet.push(next);
            openMap[nKey] = next;
            parent[nKey] = cur.key;
        }
    }

    co_return {false, {}, lastFailReason};
}
//----------------------------抽稀函数--------------------------------------
    Task<vector<string>> thinPathGreedy(
        const vector<string>& originalPath, //存取抽稀前路径
        std::shared_ptr<GridEvaluator> evaluator, // 修复：统一变量名为 evaluator
        int startTime,
        uint8_t level,
        double planeRadius,
        bool enableTrueHeightCheck,
        RouteMode currentMode
    )
    {
        if (originalPath.size() <= 2) co_return originalPath;
        vector<string> smoothPath; //存储平滑后的结果
        smoothPath.push_back(originalPath[0]); //将起点存入平滑路径
        size_t currentIndex = 0; //当前节点
        size_t targetIndex = 2; //相隔一个网格的节点
        const BaseTile& baseTile = ::getProjectBaseTile(); // 获取基准瓦片范围，用于坐标转换
        uint64_t maxCoord = (1ull << (3 * level)); //用于边界检测
        int currentTime = getBeijingTime(); //用于时间规则统一

        // 使用与普通 A* 相同的无人机球形缓冲区规则
        double gridSize = getGridSize(level);
        std::vector<std::array<int, 3>> sphericalMask;
        int extendCell = 0;
        if (planeRadius > 0) {
            extendCell = static_cast<int>(std::ceil(planeRadius / gridSize));
            for (int dx = -extendCell; dx <= extendCell; ++dx) {
                for (int dy = -extendCell; dy <= extendCell; ++dy) {
                    for (int dz = -extendCell; dz <= extendCell; ++dz) {
                        double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize;
                        if (dist <= planeRadius) {
                            sphericalMask.push_back({dx, dy, dz});
                        }
                    }
                }
            }
        }
        if (sphericalMask.empty()) {
            sphericalMask.push_back({0, 0, 0});
        }
        // 与普通 A* 保持一致：设置了半径时至少扩展一格缓冲区
        if (planeRadius > 0 && sphericalMask.size() == 1) {
            sphericalMask.clear();
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        sphericalMask.push_back({dx, dy, dz});
                    }
                }
            }
        }

        RouteWeights weights = getWeightsByMode(currentMode);
        auto calcPenalty = [&](const GridEvaluator::CheckResult& res) {
            double safetyPenalty =
                (weights.comm  * res.commCost) +
                (weights.nav   * res.navCost)  +
                (weights.surv  * res.survCost) +
                (weights.wind  * res.windCost) +
                (weights.rain  * res.rainCost) +
                (weights.vis   * res.visCost)  +
                (weights.temp  * res.tempCost) +
                (weights.hum   * res.humCost)  +
                (weights.press * res.pressCost)+
                (weights.em    * res.emCost);
            double riskPenalty = weights.riskArea * res.riskCost;
            double privacyPenalty = weights.privacy * res.privacyCost;
            return safetyPenalty + riskPenalty + privacyPenalty;
        };

        while (targetIndex < originalPath.size())
        {
            //获取物理坐标并调用DDA射线算法
            LatLonHei p1 = getLocalTileLatLon(originalPath[currentIndex], baseTile); //获取当前点坐标
            LatLonHei p2 = getLocalTileLatLon(originalPath[targetIndex], baseTile); //获取目标点的坐标
            std::vector<std::array<double, 3>> lineReq{  //记录p1和p2的经纬高
                {p1.longitude, p1.latitude, p1.height},
                {p2.longitude, p2.latitude, p2.height}
            };
            std::vector<std::string> lineGrids = singleLineToGrids2(lineReq, level, baseTile); //拉取直线
            // 按无人机半径建立球形缓冲区
            std::unordered_set<std::string> expandedGridSet;
            for (const auto& code : lineGrids)
            {
                IJH centerIJH = getLocalTileRHC(code);
                int cx = centerIJH.column;
                int cy = centerIJH.row;
                int cz = centerIJH.layer;
                for (const auto& offset : sphericalMask)
                {
                    int nx = cx + offset[0];
                    int ny = cy + offset[1];
                    int nz = cz + offset[2];
                    // 边界保护：兼容负高度和坐标越界
                    if (nx < 0 || ny < 0 || nz < 0) continue;
                    if (static_cast<uint64_t>(nx) >= maxCoord || static_cast<uint64_t>(ny) >= maxCoord || static_cast<uint64_t>(nz) >= maxCoord) continue;
                    IJH nIjh = {(uint32_t)ny, (uint32_t)nx, (uint32_t)nz};
                    expandedGridSet.insert(rchToCode(nIjh, level));
                }
            }

            // ================= 修复核心：循环分离 =================
            // 1. 先把所有要校验的网格塞进数组
            vector<CandidateInfo> checkCands;
            auto norm = normalizeGridTime(startTime, currentTime);
            for (const auto& code : expandedGridSet)
            {
                checkCands.push_back({code, startTime, norm.wdTime, norm.wdRule, true});
            }
            // 注意：装填数据的 for 循环在这里结束了！

            // 2. 挂起协程，统一等待 Redis 批量校验完成
            auto checkResultsPtr = co_await GridCheckAwaiter{evaluator, checkCands};

            // 3. 计算抽稀起止点的最大允许代价 (动态权重判断核心)
            double maxAllowedPenalty = 0.0;
            if (checkResultsPtr->count(originalPath[currentIndex])) {
                maxAllowedPenalty = std::max(maxAllowedPenalty, calcPenalty(checkResultsPtr->at(originalPath[currentIndex])));
            }
            if (checkResultsPtr->count(originalPath[targetIndex])) {
                maxAllowedPenalty = std::max(maxAllowedPenalty, calcPenalty(checkResultsPtr->at(originalPath[targetIndex])));
            }
            maxAllowedPenalty += 1e-5; // 容差，防止浮点数精度误差

            // 4. 双指针判定与滑动机制
            bool isLineSafe = true;
            for (const auto& code : expandedGridSet) {
                // 【新增】真高安全校验：拉直的视线绝不能越过 120m 适飞区或 15m 撞地红线
                if (enableTrueHeightCheck) {
                    LatLonHei boundary = getLocalTileLatLon(code, baseTile);
                    float ground = TiffReader::getInstance().getElevation(boundary.longitude, boundary.latitude);
                    float tHeight = boundary.height - ground;

                    if (tHeight > 120.0f || tHeight < 15.0f) {
                        isLineSafe = false;
                        break;
                    }
                }

                // 规则及代价校验
                if (checkResultsPtr->count(code))
                {
                    const auto& res = checkResultsPtr->at(code);
                    if (!res.pass)
                    {
                        isLineSafe = false;
                        break;
                    }
                    if (calcPenalty(res) > maxAllowedPenalty)
                    {
                        isLineSafe = false; // 直线穿过了比原A*节点代价更高的区域，否决抽稀
                        break;
                    }
                }
            }

            if (isLineSafe)
            {
                targetIndex++;
            }
            else
            {
                // 不安全撞墙了，退回到上一个确认安全的点作为必经拐点
                smoothPath.push_back(originalPath[targetIndex - 1]);
                //将退回的节点当作下一次探索的起点
                currentIndex = targetIndex - 1;
                targetIndex = currentIndex + 2;
            }
        }
        smoothPath.push_back(originalPath.back());

        std::vector<std::array<int, 3>> tempWaypoints;
        for (const auto& code : smoothPath) {
            IJH p = getLocalTileRHC(code);
            tempWaypoints.push_back({(int)p.column, (int)p.row, (int)p.layer});
        }
        vector<string> finalSmoothPath;
        for (const auto& wp : tempWaypoints) {
            IJH p = {(uint32_t)wp[1], (uint32_t)wp[0], (uint32_t)wp[2]};
            finalSmoothPath.push_back(rchToCode(p, level));
        }
        co_return finalSmoothPath;
    }
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
        vector<array<int, 3>> waypoints;

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
            waypoints.push_back({static_cast<int>(ijh.column), static_cast<int>(ijh.row), layer});
        }

        long long rawStartTime = (*jsonBody).get("startTime", static_cast<Json::Int64>(getBeijingTime())).asInt64();
        int startTime = (rawStartTime > 9999999999LL) ? static_cast<int>(rawStartTime / 1000) : rawStartTime;

        double planeRadius = (*jsonBody).get("planeRadius", 0.75).asDouble();

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
            waypoints[i][2] = static_cast<int>(wpIJH.layer);
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
        bool pathSuccess = true;
        string failReason;

        std::shared_ptr<GridEvaluator> gridEvaluator = nullptr;
        if (!isUnconstrained) {
            gridEvaluator = GridEvaluator::create(ruleOptions);
        }
        LOG_INFO << "[A*] 开始数据库可见图绕行预处理";
        insertDatabaseVisibilityWaypoints(
            waypoints, level, baseTile, currentMode, startTime);
        int currentSegmentStartTime = startTime;

        for (size_t i = 0; i < waypoints.size() - 1 && pathSuccess; ++i) {
            AStarResult segmentResult;

            if (isUnconstrained) {
                LOG_INFO << "[A*] 航段 " << i+1 << " 使用无约束模式（简化版A*）";
                segmentResult = aStarPathSimple(waypoints[i], waypoints[i + 1], options, level, workLayer, enableTrueHeightCheck);
            } else {
                LOG_INFO << "[A*] 航段 " << i+1 << " 使用约束模式（协程版A*）";
                segmentResult = co_await aStarPath(
                    waypoints[i], waypoints[i + 1], currentSegmentStartTime, planeRadius, options, level, gridEvaluator,
                    workLayer, currentMode, enableTrueHeightCheck
                );
            }

            if (!segmentResult.success) {
                pathSuccess = false;
                failReason = segmentResult.reason;
                break;
            }
            // 调用平滑函数 (根据 applySmoothing 标志决定是否执行)
            if (applySmoothing && !isUnconstrained && !segmentResult.path.empty() && gridEvaluator) {
                LOG_INFO << "[A*] 航段 " << i+1 << " 开始执行A*航线抽稀...";
                segmentResult.path = co_await thinPathGreedy(segmentResult.path, gridEvaluator, currentSegmentStartTime, level, planeRadius, enableTrueHeightCheck, currentMode);
            }
            if (!segmentResult.path.empty()) {
                double stepGridSize = getGridSize(level);
                int segmentDuration = 0;
                for(size_t j = 0; j < segmentResult.path.size() - 1; ++j) {
                    IJH p1 = getLocalTileRHC(segmentResult.path[j]);
                    IJH p2 = getLocalTileRHC(segmentResult.path[j+1]);
                    double dx = (int)p2.column - (int)p1.column;
                    double dy = (int)p2.row - (int)p1.row;
                    double dz = (int)p2.layer - (int)p1.layer;
                    // 精确计算 26方向 实际发生的欧几里得距离，累加时间
                    segmentDuration += static_cast<int>(std::sqrt(dx*dx + dy*dy + dz*dz) * stepGridSize / options.speed);
                }
                currentSegmentStartTime += segmentDuration;
            }

            size_t segmentIndex = i + 1;
            if (i == 0) {
                fullPath = segmentResult.path;
                pathIndexes.assign(segmentResult.path.size(), segmentIndex);
            } else {
                fullPath.insert(fullPath.end(), segmentResult.path.begin() + 1, segmentResult.path.end());
                pathIndexes.insert(pathIndexes.end(), segmentResult.path.size() - 1, segmentIndex);
            }

            if (i < waypoints.size() - 2 && !segmentResult.path.empty()) {
                waypointCodes.push_back(segmentResult.path.back());
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
                int landingIndex = static_cast<int>(waypoints.size());
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
