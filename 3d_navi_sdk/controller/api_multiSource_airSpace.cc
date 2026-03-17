// 引入空域统计API头文件
#include "api_multiSource_airSpace.h"
// 引入DQG全局基准瓦片
#include <dqg/GlobalBaseTile.h>
// 引入DQG 3D基础库
#include <dqg/DQG3DBasic.h>
// 引入DQG 3D多边形库
#include <dqg/DQG3DPolygon.h>
// 引入Drogon数据库客户端
#include <drogon/orm/DbClient.h>
// 引入OpenSSL加密库
#include <openssl/evp.h>
// 引入格式化输出
#include <iomanip>
// 引入字符串流
#include <sstream>
// 引入算法库
#include <algorithm>
// 引入数学库
#include <cmath>
// 引入时间库
#include <chrono>
// 引入无序集合
#include <unordered_set>
// 引入GEOS几何库
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/Coordinate.h>
#include <geos/io/WKTReader.h>
#include <geos/io/WKTWriter.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 辅助工具：MD5 哈希生成 (使用 OpenSSL EVP 接口)
// @param str 待哈希的字符串
// @return 32位十六进制MD5哈希值
static std::string stringToMD5(const std::string& str) {
    // 创建MD5上下文
    EVP_MD_CTX* context = EVP_MD_CTX_new();
    const EVP_MD* md = EVP_md5();
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int lengthOfHash = 0;

    if (context) {
        // 初始化哈希上下文
        EVP_DigestInit_ex(context, md, nullptr);
        // 更新哈希数据
        EVP_DigestUpdate(context, str.c_str(), str.size());
        // 获取哈希结果
        EVP_DigestFinal_ex(context, hash, &lengthOfHash);
        // 释放上下文
        EVP_MD_CTX_free(context);
    }

    // 将二进制哈希转换为十六进制字符串
    std::stringstream ss;
    for(unsigned int i = 0; i < lengthOfHash; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
    }
    return ss.str();
}

/**
 * 生成多边形的唯一ID
 * 该函数通过将Json格式的多边形数据转换为字符串，然后计算其MD5哈希值来生成唯一标识符
 *
 * @param polygonJson 表示多边形数据的Json::Value对象，包含多边形的顶点坐标等信息
 * @return std::string 返回基于多边形数据的32位十六进制MD5哈希值作为唯一ID
 *
 * @note 使用MD5作为ID生成算法，相同的多边形数据（包括坐标顺序）会生成相同的ID
 */
std::string api_multiSource_airSpace::generatePolygonId(const Json::Value& polygonJson) {
    // 将JSON对象序列化为字符串
    Json::FastWriter writer;
    std::string raw = writer.write(polygonJson);
    // 打印raw字符串，便于调试
    std::cout << "Polygon Raw String: " << raw << std::endl;
    // 计算MD5哈希值作为唯一ID
    return stringToMD5(raw);
}

// 接口实现：适飞区域统计
// 功能：计算指定多边形区域内适飞网格的占比，并统计到数据库
// 请求参数（JSON）：
//   - polygon: 多边形坐标数组，格式为 [[经度, 纬度, 高度], ...]
//   - level: 网格层级，决定网格的精度
//   - version: 数据版本号，用于数据更新控制
// 返回结果（JSON）：
//   - status: 状态（success/error）
//   - id: 多边形唯一标识符
//   - total_grids: 总网格数
//   - blocked_grids: 受阻网格数
//   - flyable_rate: 适飞率（0-1）
//   - create_time: 创建时间戳
//   - update_time: 更新时间戳
void api_multiSource_airSpace::flyableAreaStatistics(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) {
    // === 跨域处理 ===
    // 处理OPTIONS预检请求，允许跨域访问
    if (req->method() == Options) {
        auto resp = HttpResponse::newHttpResponse();
        resp->addHeader("Access-Control-Allow-Origin", "*");
        callback(resp);
        return;
    }

    // 记录请求开始时间（毫秒级时间戳）
    long long create_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // 解析JSON请求体
    auto jsonPtr = req->getJsonObject();
    if (!jsonPtr) {
        auto resp = HttpResponse::newHttpResponse();
        resp->setStatusCode(k400BadRequest);
        resp->setBody("无效的 JSON 请求体");
        callback(resp);
        return;
    }
    Json::Value& json = *jsonPtr;

    // === 1. 参数解析与验证 ===
    // 检查必需参数是否存在
    if (!json.isMember("polygon") || !json.isMember("level") || !json.isMember("version")) {
        Json::Value err;
        err["status"] = "error";
        err["message"] = "缺少必需参数: polygon, level, version";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // 提取参数值
    int level = json["level"].asInt();           // 网格层级
    double version = json["version"].asDouble();   // 数据版本号
    Json::Value polygonJson = json["polygon"];    // 多边形坐标数据

    // === 2. 网格化处理 ===
    // 初始化全局瓦片（使用 dqglib 库函数）
    try { getProjectBaseTile(); } catch (...) {
        Json::Value err;
        err["status"] = "error";
        err["message"] = "全局瓦片初始化失败";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // 将多边形JSON转换为字符串
    Json::FastWriter writer;
    std::string polygonStr = writer.write(polygonJson);

    // 调用库函数生成网格编码，高度固定为 0-120m
    // 返回多边形区域内所有网格的编码列表
    std::vector<std::string> gridCodes = getPolygonGridCodes(polygonStr, 120.0, 0.0, level, projectBaseTile);
    long long totalGrids = gridCodes.size();

    // === 3 & 4. Redis 查询与冲突判断 ===
    long long blockedGrids = 0;
    auto redis = drogon::app().getRedisClient();

    if (redis && totalGrids > 0) {
        // 批量查询配置：每批处理20个网格
        const size_t batchSize = 20;
        for (size_t i = 0; i < totalGrids; i += batchSize) {
            size_t end = std::min(i + batchSize, (size_t)totalGrids);

            // 构建MGET命令：查询每个网格的三种冲突类型
            // - gd_: 三维实景障碍物 (3D Scene Obstacle)
            // - za_: 一般障碍物 (Obstacle)
            // - dz_: 电子围栏 (Electronic Fence)
            std::string cmd = "MGET";
            for (size_t j = i; j < end; ++j) {
                std::string code = gridCodes[j];
                cmd += " gd_" + code + " za_" + code + " dz_" + code;
            }

            try {
                // 执行Redis同步查询
                auto res = redis->execCommandSync<std::vector<std::string>>(
                    [](const drogon::nosql::RedisResult& r){
                        std::vector<std::string> ret;
                        try {
                            auto arr = r.asArray();
                            for(const auto& item : arr) {
                                if (item.isNil()) ret.push_back("");
                                else ret.push_back(item.asString());
                            }
                        } catch (...) {}
                        return ret;
                    },
                    cmd.c_str()
                );

                // 解析查询结果，统计受阻网格
                // 每个网格对应3个查询结果（gd, za, dz）
                for (size_t k = 0; k < res.size(); k += 3) {
                    bool blocked = false;
                    // 如果任一冲突类型有值，则标记为受阻
                    if (!res[k].empty()) blocked = true;   // gd冲突
                    if (!res[k+1].empty()) blocked = true; // za冲突
                    if (!res[k+2].empty()) blocked = true; // dz冲突
                    if (blocked) blockedGrids++;
                }

            } catch (const std::exception& e) {
                LOG_ERROR << "Redis 查询错误: " << e.what();
            }
        }
    }

    // === 5. 计算适飞率 ===
    double flyableRate = 0.0;
    if (totalGrids > 0) {
        // 适飞率 = (总网格数 - 受阻网格数) / 总网格数
        flyableRate = (double)(totalGrids - blockedGrids) / (double)totalGrids;
    }

    // === 6. 记录结束时间 ===
    long long update_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // === 7. 存入数据库 ===
    // 生成多边形唯一标识符
    std::string polyId = generatePolygonId(polygonJson);

    // 获取数据库客户端（指定名称为 "default"）
    auto db = drogon::app().getDbClient("default");

    // 检查数据库客户端是否可用
    if (!db) {
        LOG_ERROR << "数据库客户端 'default' 未找到，请检查 config.json";
        Json::Value err;
        err["status"] = "error";
        err["message"] = "数据库配置错误";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    try {
        // 创建表（如果不存在）
        // 表结构：
        // - id: 多边形唯一标识符（主键）
        // - flyableRate: 适飞率（0-1之间的小数）
        // - version: 数据版本号
        // - create_time: 创建时间（毫秒时间戳）
        // - update_time: 更新时间（毫秒时间戳）
        std::string createTableSql = "CREATE TABLE IF NOT EXISTS airspaceFlyableStatistics ("
                                     "id TEXT PRIMARY KEY, "
                                     "flyableRate DOUBLE PRECISION, "
                                     "version DOUBLE PRECISION, "
                                     "create_time BIGINT, "
                                     "update_time BIGINT)";
        db->execSqlSync(createTableSql);

        // 插入或更新数据
        // 使用UPSERT语法：如果ID冲突则更新，否则插入
        // 只有当新版本号大于数据库中的版本号时才更新
        std::string sql = "INSERT INTO airspaceFlyableStatistics (id, flyableRate, version, create_time, update_time) "
                          "VALUES ($1, $2, $3, $4, $5) "
                          "ON CONFLICT (id) DO UPDATE SET "
                          "flyableRate = EXCLUDED.flyableRate, "
                          "version = EXCLUDED.version, "
                          "create_time = EXCLUDED.create_time, "
                          "update_time = EXCLUDED.update_time "
                          "WHERE airspaceFlyableStatistics.version < EXCLUDED.version";

        db->execSqlSync(sql, polyId, flyableRate, version, create_time, update_time);

    } catch (const std::exception& e) {
        LOG_ERROR << "数据库错误: " << e.what();
        Json::Value err;
        err["status"] = "error";
        err["message"] = "数据库操作失败: " + std::string(e.what());
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // === 返回结果 ===
    Json::Value ret;
    ret["status"] = "success";
    ret["id"] = polyId;
    ret["total_grids"] = (Json::UInt64)totalGrids;
    ret["blocked_grids"] = (Json::UInt64)blockedGrids;
    ret["flyable_rate"] = flyableRate;
    ret["create_time"] = (Json::UInt64)create_time;
    ret["update_time"] = (Json::UInt64)update_time;

    callback(HttpResponse::newHttpJsonResponse(ret));
}

/**
 * 接口实现：区域（地信小镇）适飞网格统计-基于数据库
 *
 * 请求参数（JSON）：
 *   - polygon: 多边形坐标数组，定义区域边界
 *     格式：[[经度1, 纬度1], [经度2, 纬度2], ...]
 *     单位：经纬度（度）
 *   - level: 网格层级，决定网格精度
 *   - top: 高度范围（区域顶部），单位米
 *   - bottom: 高度范围（区域底部），单位米
 *
 * 1、将输入区域按指定层级进行网格化，并计数区域所有网格数即为总网格数
 * 2、查询air_space表，查询表中所有WG类型空域，提取为多边形区域及其高度范围，作为禁飞区
 * 3、将区域多边形范围与禁飞区多边形范围取交集，再将交集区域按相同层级进行网格化，并计数即为禁飞区网格数
 * 4、总网格数减去禁飞区网格数即为无障碍物网格数
 * 5、计算适飞率：无障碍网格数 / 总网格数
 *
 * 返回结果（JSON）：
 *   - status: 请求状态（success/error）
 *   - flyable_grids: 无障碍物网格数（可飞行网格）
 *   - blocked_grids: 禁飞区网格数（不可飞行网格）
 *   - flyable_rate: 适飞率（小数形式，范围0-1）
 *
 */
void api_multiSource_airSpace::countFlyableGrids(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) {
    // === 跨域处理 ===
    if (req->method() == Options) {
        auto resp = HttpResponse::newHttpResponse();
        resp->addHeader("Access-Control-Allow-Origin", "*");
        callback(resp);
        return;
    }

    // 解析JSON请求体
    auto jsonPtr = req->getJsonObject();
    if (!jsonPtr) {
        auto resp = HttpResponse::newHttpResponse();
        resp->setStatusCode(k400BadRequest);
        resp->setBody("无效的 JSON 请求体");
        callback(resp);
        return;
    }
    Json::Value& json = *jsonPtr;

    // === 1. 参数解析与验证 ===
    if (!json.isMember("polygon") || !json.isMember("level") || !json.isMember("top") || !json.isMember("bottom")) {
        Json::Value err;
        err["status"] = "error";
        err["message"] = "缺少必需参数: polygon, level, top, bottom";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // 提取参数值
    int level = json["level"].asInt();
    int countLevel = json.isMember("countLevel") ? json["countLevel"].asInt() : 14;
    double top = json["top"].asDouble();
    double bottom = json["bottom"].asDouble();
    Json::Value polygonJson = json["polygon"];

    // 校验countLevel必须大于level
    if (countLevel <= level) {
        Json::Value err;
        err["status"] = "error";
        err["message"] = "countLevel必须大于level";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // === 2. 网格化处理 ===
    // 初始化全局瓦片
    try { getProjectBaseTile(); } catch (...) {
        Json::Value err;
        err["status"] = "error";
        err["message"] = "全局瓦片初始化失败";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // 将多边形JSON转换为字符串
    Json::FastWriter writer;
    std::string polygonStr = writer.write(polygonJson);

    // 调用库函数生成输入区域的网格编码
    std::vector<std::string> inputAreaGridCodes = getPolygonGridCodes(polygonStr, top, bottom, level, projectBaseTile);
    long long totalGrids = inputAreaGridCodes.size();

    LOG_INFO << "输入区域网格化完成: 层级=" << level << ", 高度范围=[" << bottom << "-" << top << "]米, 总网格数=" << totalGrids;

    if (totalGrids == 0) {
        Json::Value ret;
        ret["status"] = "success";
        ret["flyable_grids"] = (Json::UInt64)0;
        ret["blocked_grids"] = (Json::UInt64)0;
        ret["flyable_rate"] = 0.0;
        callback(HttpResponse::newHttpJsonResponse(ret));
        return;
    }

    // === 3. 查询数据库获取禁飞区数据 ===
    auto db = drogon::app().getDbClient("default");
    if (!db) {
        LOG_ERROR << "数据库客户端 'default' 未找到，请检查 config.json";
        Json::Value err;
        err["status"] = "error";
        err["message"] = "数据库配置错误";
        callback(HttpResponse::newHttpJsonResponse(err));
        return;
    }

    // 存储被禁飞区覆盖的网格总数
    std::unordered_set<std::string> blockedGridSet;
    
    try {
        // 查询air_space表，获取所有禁飞区数据
        // 筛选条件：space_type为'WG'且高度范围与输入区域有交集的禁飞区
        std::string sql = "SELECT id, boundary_data, alt_max, alt_min, shape FROM air_space WHERE "
                          "space_type = 'WG' AND "
                          "alt_max IS NOT NULL AND alt_min IS NOT NULL AND "
                          "alt_max >= $1 AND alt_min <= $2";

        auto result = db->execSqlSync(sql, bottom, top);
        
        LOG_INFO << "查询到 " << result.size() << " 个符合条件的禁飞区";

        // 初始化GEOS几何工厂
        geos::geom::GeometryFactory::Ptr geometryFactory = geos::geom::GeometryFactory::create();
        geos::io::WKTReader wktReader(geometryFactory.get());
        geos::io::WKTWriter wktWriter;
        
        // 将输入区域多边形转换为GEOS几何对象
        std::unique_ptr<geos::geom::Geometry> inputGeometry;
        try {
            std::string inputWKT = "POLYGON((";
            for (size_t i = 0; i < polygonJson.size(); ++i) {
                double lon = polygonJson[static_cast<int>(i)][0].asDouble();
                double lat = polygonJson[static_cast<int>(i)][1].asDouble();
                inputWKT += std::to_string(lon) + " " + std::to_string(lat);
                if (i < polygonJson.size() - 1) {
                    inputWKT += ", ";
                }
            }
            // 闭合多边形
            double firstLon = polygonJson[0][0].asDouble();
            double firstLat = polygonJson[0][1].asDouble();
            inputWKT += ", " + std::to_string(firstLon) + " " + std::to_string(firstLat);
            inputWKT += "))";
            
            inputGeometry = wktReader.read(inputWKT);
            LOG_DEBUG << "输入区域WKT: " << inputWKT.substr(0, std::min<size_t>(200, inputWKT.size()));
        } catch (const std::exception& e) {
            LOG_ERROR << "创建输入区域几何对象失败: " << e.what();
            Json::Value err;
            err["status"] = "error";
            err["message"] = "创建输入区域几何对象失败: " + std::string(e.what());
            callback(HttpResponse::newHttpJsonResponse(err));
            return;
        }

        // 对每个禁飞区进行处理
        for (size_t i = 0; i < result.size(); ++i) {
            // 获取禁飞区ID
            int zoneId = result[i]["id"].as<int>();
            std::string boundaryStr = result[i]["boundary_data"].as<std::string>();
            std::string shape = result[i]["shape"].isNull() ? "1" : result[i]["shape"].as<std::string>();

            // 打印查询到的数据用于调试
            LOG_INFO << "处理禁飞区 id=" << zoneId << ": shape=" << shape 
                     << ", alt_max=" << (result[i]["alt_max"].isNull() ? "NULL" : std::to_string(result[i]["alt_max"].as<double>()))
                     << ", alt_min=" << (result[i]["alt_min"].isNull() ? "NULL" : std::to_string(result[i]["alt_min"].as<double>()))
                     << ", boundary_data=" << boundaryStr.substr(0, std::min<size_t>(200, boundaryStr.size()));

            Json::Value boundaryJson;
            Json::Reader reader;

            // 安全获取高度值，处理NULL情况
            double altMax = 0.0;
            double altMin = 0.0;
            try {
                if (!result[i]["alt_max"].isNull()) {
                    altMax = result[i]["alt_max"].as<double>();
                }
                if (!result[i]["alt_min"].isNull()) {
                    altMin = result[i]["alt_min"].as<double>();
                }
            } catch (const std::exception& e) {
                LOG_WARN << "获取高度值失败: " << e.what();
                continue;
            }

            // 解析boundary_data
            if (!reader.parse(boundaryStr, boundaryJson)) {
                LOG_WARN << "解析 boundary_data 失败: " << boundaryStr;
                continue;
            }

            // 构建禁飞区多边形JSON
            Json::Value noFlyPolygonJson;
            
            if (shape == "2") {
                // 圆形区域（shape=2），构造近似多边形
                if (boundaryJson.isArray() && boundaryJson.size() > 0) {
                    Json::Value circlePoint = boundaryJson[0];
                    
                    double lon = 0.0;
                    double lat = 0.0;
                    double radius = 0.0;
                    
                    try {
                        if (!circlePoint["longitude"].isNull()) {
                            lon = circlePoint["longitude"].asDouble();
                        }
                        if (!circlePoint["latitude"].isNull()) {
                            lat = circlePoint["latitude"].asDouble();
                        }
                        if (!circlePoint["radius"].isNull()) {
                            if (circlePoint["radius"].isString()) {
                                radius = std::stod(circlePoint["radius"].asString());
                            } else if (circlePoint["radius"].isDouble() || circlePoint["radius"].isInt()) {
                                radius = circlePoint["radius"].asDouble();
                            }
                        }
                    } catch (const std::exception& e) {
                        LOG_WARN << "获取圆形参数失败: " << e.what();
                        continue;
                    }
                    
                    if (radius <= 0.0) {
                        LOG_WARN << "禁飞区 id=" << zoneId << ", 圆形半径无效: " << radius;
                        continue;
                    }
                    
                    // 构造圆形的近似多边形（使用36个点）
                    Json::Value circlePolygon(Json::arrayValue);
                    int points = 36;
                    for (int j = 0; j < points; ++j) {
                        double angle = 2.0 * M_PI * j / points;
                        double earthRadius = 6371000.0;
                        double deltaLon = (radius / earthRadius) * (180.0 / M_PI) / cos(lat * M_PI / 180.0);
                        double deltaLat = (radius / earthRadius) * (180.0 / M_PI);
                        
                        Json::Value point(Json::arrayValue);
                        point.append(lon + deltaLon * cos(angle));
                        point.append(lat + deltaLat * sin(angle));
                        circlePolygon.append(point);
                    }
                    noFlyPolygonJson = circlePolygon;
                    LOG_INFO << "禁飞区 id=" << zoneId << ", 圆形参数: lon=" << lon << ", lat=" << lat << ", radius=" << radius;
                } else {
                    continue;
                }
            } else {
                // 多边形区域（shape=1），转换格式
                if (boundaryJson.isArray()) {
                    Json::Value polygonArray(Json::arrayValue);
                    for (size_t j = 0; j < boundaryJson.size(); ++j) {
                        Json::Value point(Json::arrayValue);
                        Json::Value vertex = boundaryJson[static_cast<int>(j)];
                        
                        try {
                            double lon = vertex["longitude"].asDouble();
                            double lat = vertex["latitude"].asDouble();
                            point.append(lon);
                            point.append(lat);
                            polygonArray.append(point);
                        } catch (const std::exception& e) {
                            LOG_WARN << "转换多边形顶点失败: " << e.what();
                            continue;
                        }
                    }
                    noFlyPolygonJson = polygonArray;
                } else {
                    LOG_WARN << "禁飞区 id=" << zoneId << ", boundary_data 不是数组格式";
                    continue;
                }
            }
            
            // 计算禁飞区与输入区域的高度交集
            double intersectTop = std::min(top, altMax);
            double intersectBottom = std::max(bottom, altMin);
            
            if (intersectTop < intersectBottom) {
                LOG_WARN << "禁飞区 id=" << zoneId << ", 高度交集无效: top=" << intersectTop << ", bottom=" << intersectBottom;
                continue;
            }
            
            // 将禁飞区多边形转换为GEOS几何对象
            std::unique_ptr<geos::geom::Geometry> noFlyGeometry;
            try {
                std::string noFlyWKT = "POLYGON((";
                for (size_t j = 0; j < noFlyPolygonJson.size(); ++j) {
                    double lon = noFlyPolygonJson[static_cast<int>(j)][0].asDouble();
                    double lat = noFlyPolygonJson[static_cast<int>(j)][1].asDouble();
                    noFlyWKT += std::to_string(lon) + " " + std::to_string(lat);
                    if (j < noFlyPolygonJson.size() - 1) {
                        noFlyWKT += ", ";
                    }
                }
                // 闭合多边形
                double firstLon = noFlyPolygonJson[0][0].asDouble();
                double firstLat = noFlyPolygonJson[0][1].asDouble();
                noFlyWKT += ", " + std::to_string(firstLon) + " " + std::to_string(firstLat);
                noFlyWKT += "))";
                
                noFlyGeometry = wktReader.read(noFlyWKT);
                LOG_DEBUG << "禁飞区 id=" << zoneId << ", WKT: " << noFlyWKT.substr(0, std::min<size_t>(200, noFlyWKT.size()));
            } catch (const std::exception& e) {
                LOG_WARN << "创建禁飞区几何对象失败: " << e.what();
                continue;
            }
            
            // 计算多边形交集
            try {
                std::unique_ptr<geos::geom::Geometry> intersection = inputGeometry->intersection(noFlyGeometry.get());
                
                if (intersection->isEmpty()) {
                    LOG_INFO << "禁飞区 id=" << zoneId << ", 与输入区域无交集";
                    continue;
                }
                
                // 将交集多边形转换为JSON格式用于网格化
                std::string intersectWKT = wktWriter.write(intersection.get());
                LOG_INFO << "禁飞区 id=" << zoneId << ", 交集WKT: " << intersectWKT.substr(0, std::min<size_t>(500, intersectWKT.size()));
                
                // 解析WKT格式为多边形坐标
                // WKT格式示例: POLYGON((lon1 lat1, lon2 lat2, ..., lon1 lat1))
                // 可能还有: MULTIPOLYGON(((...)), ((...))) 等复杂几何
                Json::Value intersectPolygonJson(Json::arrayValue);
                std::string geometryType = intersection->getGeometryType();
                LOG_INFO << "禁飞区 id=" << zoneId << ", 交集几何类型: " << geometryType;

                // 处理POLYGON类型（兼容带空格的 POLYGON ((...)) 格式）
                if (intersectWKT.find("POLYGON") == 0) {
                    size_t start = intersectWKT.find("((") + 2;
                    size_t end = intersectWKT.find("))");
                    std::string coords = intersectWKT.substr(start, end - start);

                    std::istringstream iss(coords);
                    std::string coord;
                    while (std::getline(iss, coord, ',')) {
                        std::istringstream coordIss(coord);
                        double lon, lat;
                        if (coordIss >> lon >> lat) {
                            Json::Value point(Json::arrayValue);
                            point.append(lon);
                            point.append(lat);
                            intersectPolygonJson.append(point);
                        }
                    }
                }
                // 处理MULTIPOLYGON类型
                else if (intersectWKT.find("MULTIPOLYGON") == 0) {
                    LOG_WARN << "禁飞区 id=" << zoneId << ", 交集为MULTIPOLYGON类型，当前未支持";
                    continue;
                }
                // 处理LINESTRING或POINT类型
                else if (intersectWKT.find("LINESTRING") == 0 || intersectWKT.find("POINT") == 0) {
                    LOG_WARN << "禁飞区 id=" << zoneId << ", 交集为" << geometryType << "类型，无法网格化";
                    continue;
                }
                else {
                    LOG_WARN << "禁飞区 id=" << zoneId << ", 未知的WKT格式: " << intersectWKT.substr(0, 50);
                    continue;
                }
                
                if (intersectPolygonJson.size() < 3) {
                    LOG_WARN << "禁飞区 id=" << zoneId << ", 交集多边形顶点数不足: " << intersectPolygonJson.size();
                    continue;
                }
                
                // 对交集多边形进行网格化
                std::string intersectPolygonStr = writer.write(intersectPolygonJson);
                LOG_DEBUG << "禁飞区 id=" << zoneId << ", 交集多边形JSON: " << intersectPolygonStr.substr(0, std::min<size_t>(200, intersectPolygonStr.size()));
                
                std::vector<std::string> intersectGridCodes = getPolygonGridCodes(intersectPolygonStr, intersectTop, intersectBottom, level, projectBaseTile);
                
                size_t thisIntersectGrids = intersectGridCodes.size();
                
                // 将网格代码插入到unordered_set中自动去重
                for (const auto& gridCode : intersectGridCodes) {
                    blockedGridSet.insert(gridCode);
                }
                
                LOG_INFO << "禁飞区 id=" << zoneId << ", 多边形求交集完成: 交集顶点数=" << intersectPolygonJson.size()
                         << ", 交集网格数=" << thisIntersectGrids
                         << ", 当前唯一禁飞网格数=" << blockedGridSet.size();
                
            } catch (const std::exception& e) {
                LOG_WARN << "计算多边形交集失败: " << e.what();
                continue;
            }
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR << "数据库查询错误: " << e.what();
        Json::Value err;
        err["status"] = "error";
        err["message"] = "数据库查询失败: " + std::string(e.what());
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
        return;
    }

    // === 4. 计算适飞率 ===
    // 使用unordered_set已自动去重，blockedGridSet.size()即为唯一禁飞网格数
    // 多个禁飞区重叠的网格只统计一次，结果精确
    long long uniqueBlockedGrids = blockedGridSet.size() * std::pow(8, countLevel - level); // 根据层级差计算实际网格数
    long long countTotalGrids = totalGrids * std::pow(8, countLevel - level);
    long long flyableGrids = countTotalGrids - uniqueBlockedGrids;

    if (flyableGrids < 0) flyableGrids = 0; // 防止出现负数
    
    double flyableRate = 0.0;
    if (totalGrids > 0) {
        flyableRate = (double)flyableGrids / (double)countTotalGrids;
    }
    
    LOG_INFO << "统计信息: 输入区域总网格数=" << countTotalGrids << ", 唯一禁飞网格数=" << uniqueBlockedGrids
             << ", 无障碍网格数=" << flyableGrids << ", 适飞率=" << flyableRate;

    // === 5. 返回结果 ===
    Json::Value ret;
    ret["status"] = "success";
    ret["flyable_grids"] = (Json::UInt64)flyableGrids;
    ret["blocked_grids"] = (Json::UInt64)uniqueBlockedGrids;
    ret["flyable_rate"] = flyableRate;

    callback(HttpResponse::newHttpJsonResponse(ret));
}
