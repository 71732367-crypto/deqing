#include "api_airRoute_Astar.h"
#include "GridEvaluator.h"
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
#include <utility>
#include <memory>
#include <cstdlib>
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
          std::unique_ptr<geos::geom::Geometry> geom;
          std::vector<std::pair<double, double>> vertices;
          double alt_min;
          double alt_max;
      };

      // 新增：判断时间戳是否落在逗号分隔的风险时间段内 (例如: "00:00-09:00,17:00-24:00")
      bool isTimeInRiskRanges(int timestamp, const std::string& timeRanges) {
          if (timeRanges.empty()) return false;

          std::time_t t = timestamp;
          std::tm* tm_info = std::localtime(&t);
          int currentMinutes = tm_info->tm_hour * 60 + tm_info->tm_min;

          std::stringstream ss(timeRanges);
          std::string range;
          while (std::getline(ss, range, ',')) {
              if (range.length() >= 11) {
                  int startMin = std::stoi(range.substr(0, 2)) * 60 + std::stoi(range.substr(3, 2));
                  int endMin = std::stoi(range.substr(6, 2)) * 60 + std::stoi(range.substr(9, 2));
                  if (currentMinutes >= startMin && currentMinutes <= endMin) {
                      return true;
                  }
              }
          }
          return false;
      }


      //todo: ============== 增量可见性图 辅助结构 ============

     /// 射线与障碍物多边形边的碰撞结果
struct RayHitInfo {
    const VectorObstacle* obstacle;   // 被击中的障碍物指针
    size_t obstacleIdx;               // 在 obstacles 数组中的索引
    size_t edgeI1, edgeI2;            // 被击中的边的两个顶点索引 (在 vertices 中)
    double hitLon, hitLat;            // 交点经纬度
    double distFromOrigin;            // 交点到射线起点的距离
};
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
/// 射线探测：从 fromLon/fromLat 到 toLon/toLat，遍历所有障碍物多边形，
/// 返回距起点最近的碰撞信息（仅做2D平面碰撞，高度交由主循环外部判断）
static std::optional<RayHitInfo> raycastObstacles2D(
    double fromLon, double fromLat,
    double toLon, double toLat,
    const std::vector<VectorObstacle>& obstacles)
{
    std::optional<RayHitInfo> closest;
    double minDist = std::numeric_limits<double>::max();
    for (size_t oi = 0; oi < obstacles.size(); ++oi) {
        const auto& verts = obstacles[oi].vertices;
        if (verts.size() < 3) continue;
        for (size_t i = 0; i < verts.size(); ++i) {
            size_t j = (i + 1) % verts.size();
            double hx, hy;
            if (segmentIntersect2D(fromLon, fromLat, toLon, toLat,
                                   verts[i].first, verts[i].second,
                                   verts[j].first, verts[j].second, hx, hy))
            {
                double dist = std::hypot(hx - fromLon, hy - fromLat);
                if (dist < minDist) {
                    minDist = dist;
                    closest = RayHitInfo{&obstacles[oi], oi, i, j, hx, hy, dist};
                }
            }
        }
    }
    return closest;
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
/// 局部拐点生成：对碰撞边 (i1, i2) 及其一阶邻接顶点，
/// 用外法线安全膨胀算子生成候选绕行点
static std::vector<std::pair<double, double>> generateLocalCandidates2D(
    const RayHitInfo& hit,
    double currentLon, double currentLat,
    double goalLon, double goalLat,
    double bufferDeg,                              // 安全缓冲距离（经纬度度数）
    const std::vector<VectorObstacle>& obstacles,
    const std::unordered_set<size_t>& closedHashes, // 容差去重集合
    int MAX_K = 4)
{
    const auto& poly = hit.obstacle->vertices;
    size_t len = poly.size();
    // 提取碰撞边两端点，并加入基于当前视角的宏观左右切点
    std::set<size_t> targetIndices = {
        hit.edgeI1,
        hit.edgeI2
    };

    if (len > 0) {
        double baseAngle = std::atan2(poly[0].second - currentLat, poly[0].first - currentLon);
        double maxDiff = -1e9, minDiff = 1e9;
        size_t leftIdx = 0, rightIdx = 0;
        
        for (size_t i = 0; i < len; ++i) {
            double a = std::atan2(poly[i].second - currentLat, poly[i].first - currentLon);
            double diff = a - baseAngle;
            while(diff > M_PI) diff -= 2 * M_PI;
            while(diff < -M_PI) diff += 2 * M_PI;
            
            if (diff > maxDiff) { maxDiff = diff; leftIdx = i; }
            if (diff < minDiff) { minDiff = diff; rightIdx = i; }
        }
        targetIndices.insert(leftIdx);
        targetIndices.insert(rightIdx);

    }
    std::vector<std::pair<double, double>> candidates;
    for (size_t idx : targetIndices) {
        size_t iPrev = (idx + len - 1) % len;
        size_t iNext = (idx + 1) % len;
        double px = poly[iPrev].first,  py = poly[iPrev].second;
        double cx = poly[idx].first,    cy = poly[idx].second;
        double nx = poly[iNext].first,  ny = poly[iNext].second;
        // 入边向量 v1 = curr - prev，出边向量 v2 = next - curr
        double v1x = cx - px, v1y = cy - py;
        double v2x = nx - cx, v2y = ny - cy;
        double l1 = std::hypot(v1x, v1y);
        double l2 = std::hypot(v2x, v2y);
        if (l1 < 1e-12 || l2 < 1e-12) continue;
        // 入边法向量 n1 = (v1y, -v1x) / l1
        double n1x = v1y / l1, n1y = -v1x / l1;
        // 出边法向量 n2 = (v2y, -v2x) / l2
        double n2x = v2y / l2, n2y = -v2x / l2;
        // 合成外法线方向
        double outX = n1x + n2x, outY = n1y + n2y;
        double outLen = std::hypot(outX, outY);
        if (outLen < 0.01) { outX = n1x; outY = n1y; }
        else { outX /= outLen; outY /= outLen; }
        // 候选点 = 顶点 + 外法线 × 缓冲距离
        double cpLon = cx + outX * bufferDeg;
        double cpLat = cy + outY * bufferDeg;
        // 如果候选点落入障碍物内部，翻转方向
        if (pointInAnyObstacle2D(cpLon, cpLat, obstacles)) {
            cpLon = cx - outX * bufferDeg;
            cpLat = cy - outY * bufferDeg;
        }
        // 再次校验：翻转后仍然在障碍物内部则丢弃
        if (pointInAnyObstacle2D(cpLon, cpLat, obstacles)) continue;
        // 容差去重（哈希检查）
        size_t h = std::hash<double>{}(std::round(cpLon * 1e6)) ^
                   (std::hash<double>{}(std::round(cpLat * 1e6)) << 1);
        if (closedHashes.count(h)) continue;
        candidates.push_back({cpLon, cpLat});
    }
    // 按到终点距离升序排列，取前 K 个
    std::sort(candidates.begin(), candidates.end(),
        [goalLon, goalLat](const auto& a, const auto& b) {
            return std::hypot(a.first - goalLon, a.second - goalLat)
                 < std::hypot(b.first - goalLon, b.second - goalLat);
        });
    if ((int)candidates.size() > MAX_K) candidates.resize(MAX_K);
    return candidates;
}


      //新增时间解析
      void insertVectorBypassWaypoints(std::vector<std::array<int, 3>>& waypoints, int level, const BaseTile& baseTile, RouteMode mode, int startTime) {
          if (waypoints.empty()) return;

          auto db = drogon::app().getDbClient("default");
          if (!db) return;
          std::vector<VectorObstacle> obstacles;
          try {
              auto result = db->execSqlSync("SELECT id, boundary_data, alt_min, alt_max, shape FROM air_space WHERE space_type = 'WG' AND alt_min IS NOT NULL AND alt_max IS NOT NULL");
              geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
              geos::io::WKTReader wktReader(factory.get());
              for (auto row : result) {
                  VectorObstacle obs;
                  obs.id = row["id"].as<std::string>();
                  obs.alt_min = row["alt_min"].as<double>();
                  obs.alt_max = row["alt_max"].as<double>();

                  std::string boundaryStr = row["boundary_data"].as<std::string>();
                  int shape = 1;
                  try {
                      if (!row["shape"].isNull()) {
                          shape = row["shape"].as<int>();
                      }
                  } catch(...) {}
                  Json::Value boundaryJson;
                  Json::Reader reader;
                  if (reader.parse(boundaryStr, boundaryJson) && boundaryJson.isArray() && boundaryJson.size() > 0) {
                      if (shape == 2 && boundaryJson[0].isMember("radius")) {
                          double centerLon = boundaryJson[0]["longitude"].asDouble();
                          double centerLat = boundaryJson[0]["latitude"].asDouble();
                          double radius_m = 0.0;
                          if (boundaryJson[0]["radius"].isString()) {
                              radius_m = std::stod(boundaryJson[0]["radius"].asString());
                          } else if (boundaryJson[0]["radius"].isNumeric()) {
                              radius_m = boundaryJson[0]["radius"].asDouble();
                          }

                          double lat_deg_m = 111320.0;
                          double lon_deg_m = 111320.0 * std::cos(centerLat * M_PI / 180.0);

                          int num_segments = 32;
                          std::string wkt = "POLYGON((";
                          for (int i = 0; i < num_segments; ++i) {
                              double angle = 2.0 * M_PI * i / num_segments;
                              double d_lon = (radius_m * std::cos(angle)) / lon_deg_m;
                              double d_lat = (radius_m * std::sin(angle)) / lat_deg_m;
                              double lon = centerLon + d_lon;
                              double lat = centerLat + d_lat;
                              obs.vertices.push_back({lon, lat});
                              wkt += std::to_string(lon) + " " + std::to_string(lat) + ", ";
                          }
                          double first_lon = centerLon + (radius_m * std::cos(0)) / lon_deg_m;
                          double first_lat = centerLat + (radius_m * std::sin(0)) / lat_deg_m;
                          wkt += std::to_string(first_lon) + " " + std::to_string(first_lat) + "))";

                          obs.geom = wktReader.read(wkt);
                          obstacles.push_back(std::move(obs));
                      }
                      else if (boundaryJson.size() >= 3) {
                          std::string wkt = "POLYGON((";
                          for (size_t i = 0; i < boundaryJson.size(); ++i) {
                              double lon = boundaryJson[static_cast<int>(i)]["longitude"].asDouble();
                              double lat = boundaryJson[static_cast<int>(i)]["latitude"].asDouble();
                              obs.vertices.push_back({lon, lat});
                              wkt += std::to_string(lon) + " " + std::to_string(lat);
                              wkt += ", ";
                          }
                          double firstLon = boundaryJson[0]["longitude"].asDouble();
                          double firstLat = boundaryJson[0]["latitude"].asDouble();
                          wkt += std::to_string(firstLon) + " " + std::to_string(firstLat) + "))";

                          obs.geom = wktReader.read(wkt);
                          obstacles.push_back(std::move(obs));

                      }
                  }
              }
          } catch(const std::exception& e) {
              LOG_ERROR << "Failed to load vector obstacles: " << e.what();
              return;
          }

          //增加新表"fence"
          try {
              // 如果你的 fence 表名不一样或者需要加过滤条件（如 active=1），请调整此 SQL
              auto fenceResult = db->execSqlSync("SELECT id, boundary FROM fence");
              geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
              geos::io::WKTReader wktReader(factory.get());
              for (auto row : fenceResult) {
                  if (row["boundary"].isNull()) continue;
                  VectorObstacle obs;
                  obs.id = row["id"].as<std::string>();
                  std::string boundaryStr = row["boundary"].as<std::string>();
                  Json::Value boundaryObj;
                  Json::Reader reader;
                  // 开始解析 boundary 对象
                  if (reader.parse(boundaryStr, boundaryObj) && boundaryObj.isObject()) {

                      // 1. 提取高度信息 (代替原来的 alt_min / alt_max)
                      if (boundaryObj.isMember("altitudeRange") && boundaryObj["altitudeRange"].isArray() && boundaryObj["altitudeRange"].size() == 2) {
                          obs.alt_min = boundaryObj["altitudeRange"][0].asDouble();
                          obs.alt_max = boundaryObj["altitudeRange"][1].asDouble();
                      } else {
                          continue; // 如果没有高度数据，直接跳过
                      }
                      // 2. 提取 shape，兼容数据库内配成字符串 "1" 或是 数字 1
                      int shape = 1;
                      if (boundaryObj.isMember("shape")) {
                          shape = boundaryObj["shape"].isString() ? std::stoi(boundaryObj["shape"].asString()) : boundaryObj["shape"].asInt();
                      }
                      // 3. 提取边界点 boundaryData
                      if (shape == 1 && boundaryObj.isMember("boundaryData") && boundaryObj["boundaryData"].isArray()) {
                          Json::Value boundaryData = boundaryObj["boundaryData"];
                          if (boundaryData.size() >= 3) {
                              std::string wkt = "POLYGON((";

                              for (size_t k = 0; k < boundaryData.size(); ++k) {
                                  // 兼容处理：经纬度可能是带引号的字符串（如"30.536"），也可能是浮点数
                                  double lon = boundaryData[static_cast<int>(k)]["longitude"].isString() ?
                                               std::stod(boundaryData[static_cast<int>(k)]["longitude"].asString()) :
                                               boundaryData[static_cast<int>(k)]["longitude"].asDouble();

                                  double lat = boundaryData[static_cast<int>(k)]["latitude"].isString() ?
                                               std::stod(boundaryData[static_cast<int>(k)]["latitude"].asString()) :
                                               boundaryData[static_cast<int>(k)]["latitude"].asDouble();
                                  obs.vertices.push_back({lon, lat});
                                  wkt += std::to_string(lon) + " " + std::to_string(lat) + ", ";
                              }
                              // WKT 多边形闭合，将终点连回第一个点
                              double firstLon = boundaryData[0]["longitude"].isString() ?
                                                std::stod(boundaryData[0]["longitude"].asString()) :
                                                boundaryData[0]["longitude"].asDouble();
                              double firstLat = boundaryData[0]["latitude"].isString() ?
                                                std::stod(boundaryData[0]["latitude"].asString()) :
                                                boundaryData[0]["latitude"].asDouble();
                              wkt += std::to_string(firstLon) + " " + std::to_string(firstLat) + "))";
                              // 生成几何体并合并到总的 obstacles 集合里，供下方的射线探测共用
                              obs.geom = wktReader.read(wkt);
                              obstacles.push_back(std::move(obs));
                          }
                      }
                  }
              }
          } catch(const std::exception& e) {
              LOG_ERROR << "无法从数据库获取电子围栏边界信息: " << e.what();
          }
          // ================= 新增结束 =================

          // ================= 新增风险区查询 =================
          if (mode == RouteMode::BALANCED || mode == RouteMode::SAFEST) {
              try {
                  auto riskResult = db->execSqlSync(
                      "SELECT r.*, ST_AsText(a.geom) as geom_wkt FROM risk_area a JOIN risk_area_rule r ON a.type = r.type"
                  );

                  geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
                  geos::io::WKTReader wktReader(factory.get());

                  // TODO: 请根据您的业务逻辑替换为判断当天是工作日、周末还是节假日的代码
                  bool isWorkday = true;
                  bool isWeekend = false;
                  bool isHoliday = false;

                  for (auto row : riskResult) {
                      if (row["geom_wkt"].isNull()) continue;

                      bool isHighRisk = false;
                      bool isLowRisk = false;

                      auto getRuleTime = [&](const std::string& fieldName) {
                          return row[fieldName].isNull() ? "" : row[fieldName].as<std::string>();
                      };

                      if (isWorkday) {
                          isHighRisk = isTimeInRiskRanges(startTime, getRuleTime("workday_high_risk_time"));
                          isLowRisk = isTimeInRiskRanges(startTime, getRuleTime("workday_low_risk_time"));
                      } else if (isWeekend) {
                          isHighRisk = isTimeInRiskRanges(startTime, getRuleTime("weekend_high_risk_time"));
                          isLowRisk = isTimeInRiskRanges(startTime, getRuleTime("weekend_low_risk_time"));
                      } else if (isHoliday) {
                          isHighRisk = isTimeInRiskRanges(startTime, getRuleTime("holiday_high_risk_time"));
                          isLowRisk = isTimeInRiskRanges(startTime, getRuleTime("holiday_low_risk_time"));
                      }

                      std::string riskLevelStr = "无风险或中风险";
                      if (isHighRisk) riskLevelStr = "高风险";
                      else if (isLowRisk) riskLevelStr = "低风险";

                      bool needBypass = false;
                      if (mode == RouteMode::BALANCED && isHighRisk) {
                          needBypass = true;
                      } else if (mode == RouteMode::SAFEST && (isHighRisk || isLowRisk)) {
                          needBypass = true;
                      }

                      if (needBypass) {
                          std::string typeName = getRuleTime("type");
                          std::string wktStr = row["geom_wkt"].as<std::string>();
                          std::unique_ptr<geos::geom::Geometry> geomFull;
                          try {
                              geomFull = wktReader.read(wktStr);
                          } catch(...) {}

                          if (geomFull) {
                              static int risk_counter = 0;
                              for (size_t g = 0; g < geomFull->getNumGeometries(); ++g) {
                                  const geos::geom::Geometry* singleGeo = geomFull->getGeometryN(g);
                                  if (!singleGeo) continue;

                                  VectorObstacle obs;
                                  obs.id = "risk_area_" + typeName + "_" + std::to_string(risk_counter++) + "|" + riskLevelStr;
                                  obs.geom = singleGeo->clone();

                                  auto coords = singleGeo->getCoordinates();
                                  if (coords) {
                                      for (size_t i = 0; i < coords->getSize(); ++i) {
                                          obs.vertices.push_back({coords->getAt(i).x, coords->getAt(i).y});
                                      }
                                  }
                                  obs.alt_min = -1000.0;
                                  obs.alt_max = 10000.0;

                                  obstacles.push_back(std::move(obs));
                              }
                          }
                      }
                  }
              } catch(const std::exception& e) {
                  LOG_ERROR << "Failed to load risk areas: " << e.what();
              }
          }
          // ================= 新增风险区结束 =================

          if (obstacles.empty()) return;

          //TODO:============== 上面障碍物加载（保持原有逻辑不变） ==============


          // ============== 2. 算法参数 ==============
          const int MAX_CANDIDATES_K = 4;        // K-Limiter: 单次最多取 K 个候选
          const int MAX_OBSTACLE_HITS_M = 100;    // M-Limiter: 同一障碍物碰撞次数上限
          const double DELTA_DEDUP = 1e-6;       // 容差去重阈值（经纬度度数）
          // 安全缓冲距离（50m 转经纬度度数的近似值）
          // 精确值应根据当地纬度计算，这里取平均
          double buffer_m = 50.0;//缓冲区
          double lat_deg_m = 111320.0;
          double avg_lat = (baseTile.south + baseTile.north) / 2.0;
          double lon_deg_m = 111320.0 * std::cos(avg_lat * M_PI / 180.0);
          double bufferDegLon = buffer_m / lon_deg_m;
          double bufferDegLat = buffer_m / lat_deg_m;
          double bufferDeg = (bufferDegLon + bufferDegLat) / 2.0; // 取平均作为各向同性近似
          // ============== 3. 逐段执行增量可见性图 A* ==============
          std::vector<std::array<int, 3>> newWaypoints;
          newWaypoints.push_back(waypoints[0]);
          for (size_t seg = 0; seg < waypoints.size() - 1; ++seg) {
              std::array<int, 3> A_grid = newWaypoints.back();
              std::array<int, 3> B_grid = waypoints[seg + 1];
              // 将 A、B 转换为经纬度
              IJH a_ijh = {(uint32_t)A_grid[1], (uint32_t)A_grid[0], (uint32_t)A_grid[2]};
              LatLonHei A_ll = getLocalTileLatLon(rchToCode(a_ijh, level), baseTile);
              IJH b_ijh = {(uint32_t)B_grid[1], (uint32_t)B_grid[0], (uint32_t)B_grid[2]};
              LatLonHei B_ll = getLocalTileLatLon(rchToCode(b_ijh, level), baseTile);
              double goalLon = B_ll.longitude, goalLat = B_ll.latitude;
              // --- A* 数据结构 ---
              struct VisNode {
                  double lon, lat;
                  double g;  // 累积路径代价（经纬度距离）
                  double f;  // f = g + h
                  int parentIdx;  // 在 closedList 中的父节点索引，-1 表示起点
              };
              auto calcH = [&](double lon, double lat) {
                  return std::hypot(lon - goalLon, lat - goalLat);
              };
              // Open List（按 f 值的小顶堆）
              auto cmp = [](const VisNode& a, const VisNode& b) { return a.f > b.f; };
              std::priority_queue<VisNode, std::vector<VisNode>, decltype(cmp)> openList(cmp);
              std::vector<VisNode> closedList;              // Closed List（同时记录父链）
              std::unordered_set<size_t> closedHashes;      // 容差去重哈希
              std::unordered_map<size_t, int> hitCountMap;   // M-Limiter: obstacleIdx → 碰撞次数
              // 起点入队
              VisNode startNode{A_ll.longitude, A_ll.latitude, 0.0, calcH(A_ll.longitude, A_ll.latitude), -1};
              openList.push(startNode);
              bool found = false;
              int foundClosedIdx = -1;
              const int MAX_ITERS = 5000;  // 总迭代上限（安全阀）
              int iters = 0;
              int maxHitsReached = 0; // 记录触发 M-Limiter 的次数
              while (!openList.empty() && iters < MAX_ITERS) {
                  iters++;
                  VisNode current = openList.top();
                  openList.pop();
                  // 容差去重检查
                  size_t curHash = std::hash<double>{}(std::round(current.lon * 1e6)) ^
                                   (std::hash<double>{}(std::round(current.lat * 1e6)) << 1);
                  if (closedHashes.count(curHash)) continue;
                  closedHashes.insert(curHash);
                  int curIdx = (int)closedList.size();
                  closedList.push_back(current);
                  // --- 射线探测：current → goal ---
                  auto hitInfo = raycastObstacles2D(current.lon, current.lat, goalLon, goalLat, obstacles);
                  if (!hitInfo.has_value()) {
                      // 无碰撞：current 可直达 goal，记录终点并结束
                      found = true;
                      foundClosedIdx = curIdx;
                      break;
                  }
                  // --- 碰撞处理 ---
                  // M-Limiter 检查
                  size_t obsIdx = hitInfo->obstacleIdx;
                  int& hitCount = hitCountMap[obsIdx];
                  if (hitCount >= MAX_OBSTACLE_HITS_M) {
                      maxHitsReached++;
                      continue;  // 该障碍物碰撞次数耗尽，剪枝
                  }
                  hitCount++;
                  // 局部拐点解析
                  auto candidates = generateLocalCandidates2D(
                      *hitInfo, current.lon, current.lat, goalLon, goalLat, bufferDeg,
                      obstacles, closedHashes, MAX_CANDIDATES_K);
                  for (const auto& [cLon, cLat] : candidates) {
                      // 局部可见性检测：current → candidate
                      auto localHit = raycastObstacles2D(current.lon, current.lat, cLon, cLat, obstacles);
                      if (!localHit.has_value()) {
                          // 通视，压入 Open List
                          double newG = current.g + std::hypot(cLon - current.lon, cLat - current.lat);
                          double newH = calcH(cLon, cLat);
                          openList.push(VisNode{cLon, cLat, newG, newG + newH, curIdx});
                      }
                  }
              }
              // --- 回溯路径，插入中间途径点 ---
              if (found) {
                  std::vector<std::pair<double, double>> bypassPoints;
                  int traceIdx = foundClosedIdx;
                  while (traceIdx >= 0) {
                      const auto& nd = closedList[traceIdx];
                      bypassPoints.push_back({nd.lon, nd.lat});
                      traceIdx = nd.parentIdx;
                  }
                  std::reverse(bypassPoints.begin(), bypassPoints.end());
                  // 跳过第一个（起点 A，已在 newWaypoints 中）和最后一个（会在循环外 push B）
                  // 只插入中间的绕行拐点
                  for (size_t k = 1; k < bypassPoints.size(); ++k) {
                      IJH wp_ijh = localRowColHeiNumber(
                          static_cast<uint8_t>(level),
                          bypassPoints[k].first,
                          bypassPoints[k].second,
                          A_ll.height,  // 高度层保持不变
                          baseTile);
                      std::array<int, 3> wp_grid = {(int)wp_ijh.column, (int)wp_ijh.row, A_grid[2]};
                      if (wp_grid != newWaypoints.back()) {
                          newWaypoints.push_back(wp_grid);
                      }
                  }
              } else {
                  std::string failReason = "未知原因";
                  if (iters >= MAX_ITERS) {
                      failReason = "达到全局最大迭代次数 (" + std::to_string(MAX_ITERS) + ") 限制，搜索被强制终止。原因：地图中存在巨量交叠障碍物或局部死胡同导致状态空间爆炸。";
                  } else if (openList.empty()) {
                      failReason = "搜索空间耗尽 (OpenList为空)，未找到可行路径。";
                      if (maxHitsReached > 0) {
                          failReason += " 期间曾 " + std::to_string(maxHitsReached) + " 次触发单一障碍物碰撞上限(M-Limiter=" + std::to_string(MAX_OBSTACLE_HITS_M) + ")，导致后续探索被剪枝。可能是局部障碍物点数过多或反复陷入同一个复杂多边形内。";
                      } else {
                          failReason += " 起点/终点可能被完全包围，或者生成的绕行候选点全被其他障碍物阻挡导致无法继续探索。";
                      }
                  }
                  
                  LOG_WARN << "[VisGraph Bypass] 增量可见性图搜索失败，航段 " << seg
                           << " 保持原始直连。失败详情: " << failReason;
              }
              newWaypoints.push_back(B_grid);
          }
          waypoints = newWaypoints;
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

    double hWeight = 1.2;
    if (routeMode == RouteMode::SAFEST) {
        hWeight = 1.5;
    } else if (routeMode == RouteMode::BALANCED) {
        hWeight = 1.5;
    } else if (routeMode == RouteMode::SHORTEST) {
        hWeight =1.2;
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
            candidateListForChecker.push_back({code, arrival, norm.wdTime, norm.wdRule, true});
        }

        std::shared_ptr<std::unordered_map<string, GridEvaluator::CheckResult>> checkResultsPtr;
        if (!candidateListForChecker.empty()) {
            checkResultsPtr = co_await GridCheckAwaiter{evaluator, candidateListForChecker};
        }

        // 邻居缓冲区检查：如果任何一个生成的 26 邻居网格被阻挡，抛弃整个当前节点扩展
        bool allNeighborsPassable = true;
        bool skipNeighborBufferCheck = (cur.key == startKey);
        if (!skipNeighborBufferCheck) {
            for (const auto& nb : validNeighbors) {
                if (checkResultsPtr && checkResultsPtr->count(nb.code)) {
                    const auto& res = checkResultsPtr->at(nb.code);
                    if (!res.pass) {
                        allNeighborsPassable = false;
                        lastFailReason = "缓冲区检查失败: 邻居网格 " + nb.code + " " + res.reason;
                        break;
                    }
                } else {
                    allNeighborsPassable = false;
                    lastFailReason = "缓冲区检查失败: 邻居网格 " + nb.code + " 无检查结果";
                    break;
                }
            }
        }

        if (!allNeighborsPassable) {
            continue;
        }

        // ==========================================
        // 计算邻居节点的累积代价值 (融合转向惩罚)
        // ==========================================
        for (const auto& nb : validNeighbors) {
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
            //26个方向膨胀建立缓冲区
            std::unordered_set<std::string> expandedGridSet;
            for (const auto& code : lineGrids)
            {
                expandedGridSet.insert(code); //加入中心网格
                IJH centerIJH = getLocalTileRHC(code);
                int cx = centerIJH.column;
                int cy = centerIJH.row;
                int cz = centerIJH.layer;
                //扩展26个邻居保持一格宽缓冲区
                for (const auto& d : DIRECTIONS)
                {
                    int nx = cx + d[0];
                    int ny = cy + d[1];
                    int nz = cz + d[2];
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

        // [新增] 在抽稀后，复用基于矢量的自动增加绕行途径点逻辑
        std::vector<std::array<int, 3>> tempWaypoints;
        for (const auto& code : smoothPath) {
            IJH p = getLocalTileRHC(code);
            tempWaypoints.push_back({(int)p.column, (int)p.row, (int)p.layer});
        }

    //    insertVectorBypassWaypoints(tempWaypoints, level, baseTile);

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
        double cruisingSpeed = (*jsonBody).get("cruisingSpeed", 15.0).asDouble();

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
        options.speed = cruisingSpeed;

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
            // ==========================================
            // [新增] 执行基于矢量的自动增加绕行途径点
            // ==========================================
            LOG_INFO << "[A*] 开始预处理：尝试基于矢量射线拆分绕过障碍物...";
           insertVectorBypassWaypoints(waypoints, level, baseTile, currentMode, startTime);
        }
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
                segmentResult.path = co_await thinPathGreedy(segmentResult.path, gridEvaluator, currentSegmentStartTime, level, enableTrueHeightCheck, currentMode);
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