#include "api_multiSource_geometricGrid.h"
#include <drogon/HttpTypes.h>
#include <json/json.h>
#include <dqg/Data.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/DQG3DBuffer.h>
#include <dqg/DQG3DTil.h>
#include <dqg/GlobalBaseTile.h>
#include <dqg/DQG3DPolygon.h>
#include <vector>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <fstream>
#include <future>
#include <thread>
#include "LineToGrids.h"

using namespace api::multiSource;

/**
 * @brief 点球体缓冲区网格化接口（球体网格化）
 *
 * 该接口根据给定的经纬度点、高度和搜索半径，计算并返回在指定搜索范围内的所有网格信息。
 * 支持自定义网格层级、最大网格数量以及基础瓦片范围。
 *
 * @param req HTTP请求指针，包含请求参数
 * @param callback 异步回调函数，用于返回HTTP响应
 */
void geometricGrid::getGridByPointAndRadius(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const {
    try {
        // 获取请求体JSON对象
        auto body = req->getJsonObject();
        if (!body) {
            // 请求体非JSON格式时返回400错误
            Json::Value err; err["status"] = "error"; err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 必传字段检查
        if (!body->isMember("lon") || !body->isMember("lat") || !body->isMember("height") || !body->isMember("radius")) {
            Json::Value err; err["status"] = "error"; err["message"] = "缺少必需参数: lon, lat, height, radius";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 解析请求参数
        double lon = (*body)["lon"].asDouble();          // 经度
        double lat = (*body)["lat"].asDouble();          // 纬度
        double height = (*body)["height"].asDouble();    // 高度
        double radius = (*body)["radius"].asDouble();    // 搜索半径
        // 可选参数，默认网格层级为14
        int level = body->isMember("level") ? (*body)["level"].asInt() : 14;

        // 参数有效性验证：网格层级必须在有效范围内
        if (level < 0 || level > 21) {
            Json::Value err; err["status"] = "error"; err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }
        // 参数有效性验证：搜索半径必须为正数
        if (radius <= 0) {
            Json::Value err; err["status"] = "error"; err["message"] = "radius 必须大于 0";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 使用全局常量DQBaseTile作为基础瓦片范围
        const BaseTile& baseTile = getProjectBaseTile();

        // 构造点向量并调用dqglib库的getPointsBuffer函数获取结果
        PointLBHd pt; // 构造经纬度高度点
        pt.Lng = lon;
        pt.Lat = lat;
        pt.Hgt = height;
        std::vector<PointLBHd> points = { pt }; // 构建单点向量

        // 直接在 controller 中枚举局部行列层（轻量级），避免调用可能分配大量中间结构的 getPointsBuffer
        double lonStep = (baseTile.east - baseTile.west) / (1 << level);
        double latStep = (baseTile.north - baseTile.south) / (1 << level);
        double hgtStep = (baseTile.top - baseTile.bottom) / (1 << level);

        double latRad = lat * 3.14159265358979323846 / 180.0;
        double deltaLonDeg = (radius / (6371000.0 * cos(latRad))) * 180.0 / 3.14159265358979323846;
        double deltaLatDeg = (radius / 6371000.0) * 180.0 / 3.14159265358979323846;

        int dWest = static_cast<int>(ceil(deltaLonDeg / lonStep));
        int dEast = static_cast<int>(ceil(deltaLonDeg / lonStep));
        int dSouth = static_cast<int>(ceil(deltaLatDeg / latStep));
        int dNorth = static_cast<int>(ceil(deltaLatDeg / latStep));
        int dBelow = static_cast<int>(ceil(radius / hgtStep));
        int dAbove = dBelow;

        IJH center = localRowColHeiNumber(static_cast<uint8_t>(level), lon, lat, height, baseTile);
        int64_t maxIndex = (1LL << level) - 1;
        int64_t startC = std::max<int64_t>(0, static_cast<int64_t>(center.column) - dWest);
        int64_t endC = std::min<int64_t>(maxIndex, static_cast<int64_t>(center.column) + dEast);
        int64_t startR = std::max<int64_t>(0, static_cast<int64_t>(center.row) - dSouth);
        int64_t endR = std::min<int64_t>(maxIndex, static_cast<int64_t>(center.row) + dNorth);
        int64_t startH = std::max<int64_t>(0, static_cast<int64_t>(center.layer) - dBelow);
        int64_t endH = std::min<int64_t>(maxIndex, static_cast<int64_t>(center.layer) + dAbove);



        // 枚举并精确过滤距离，构造完整网格对象（去重）
        Json::Value cells(Json::arrayValue);

        // 优化：多线程并行计算
        unsigned int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) numThreads = 4;

        int64_t totalCols = endC - startC + 1;

        if (totalCols < numThreads * 2) {
            std::unordered_set<std::string> seenCodes;
            for (int64_t c = startC; c <= endC; ++c) {
                for (int64_t r = startR; r <= endR; ++r) {
                    for (int64_t h = startH; h <= endH; ++h) {
                        IJH ijh{static_cast<uint32_t>(r), static_cast<uint32_t>(c), static_cast<uint32_t>(h)};
                        LatLonHei TEMP = IJHToLocalTileLatLon(ijh, static_cast<uint32_t>(level), baseTile);
                        PointLBHd centerPt{TEMP.longitude, TEMP.latitude, TEMP.height};
                        if (distance3D(centerPt, pt) <= radius + hgtStep / 2) {
                            try {
                                std::string code = IJH2DQG_str(ijh.row, ijh.column, ijh.layer, static_cast<uint8_t>(level));
                                if (seenCodes.find(code) != seenCodes.end()) continue;
                                seenCodes.insert(code);

                                Json::Value item;
                                item["code"] = code;
                                item["bottom"] = TEMP.bottom;
                                item["top"] = TEMP.top;
                                Json::Value centerArr(Json::arrayValue);
                                centerArr.append(TEMP.longitude);
                                centerArr.append(TEMP.latitude);
                                centerArr.append(TEMP.height);
                                item["center"] = centerArr;
                                item["minlon"] = TEMP.west;
                                item["minlat"] = TEMP.south;
                                item["maxlon"] = TEMP.east;
                                item["maxlat"] = TEMP.north;

                                cells.append(item);
                            } catch (const std::exception &e) {
                                // ignore
                            }
                        }
                    }
                }
            }
        } else {
            std::vector<std::future<std::vector<Json::Value>>> futures;
            int64_t chunkSize = (totalCols + numThreads - 1) / numThreads;

            for (unsigned int i = 0; i < numThreads; ++i) {
                int64_t c_start = startC + i * chunkSize;
                int64_t c_end = std::min(c_start + chunkSize - 1, endC);
                if (c_start > c_end) break;

                futures.push_back(std::async(std::launch::async, [c_start, c_end, startR, endR, startH, endH, level, &baseTile, pt, radius, hgtStep]() {
                    std::vector<Json::Value> localCells;
                    for (int64_t c = c_start; c <= c_end; ++c) {
                        for (int64_t r = startR; r <= endR; ++r) {
                            for (int64_t h = startH; h <= endH; ++h) {
                                IJH ijh{static_cast<uint32_t>(r), static_cast<uint32_t>(c), static_cast<uint32_t>(h)};
                                LatLonHei TEMP = IJHToLocalTileLatLon(ijh, static_cast<uint32_t>(level), baseTile);
                                PointLBHd centerPt{TEMP.longitude, TEMP.latitude, TEMP.height};
                                if (distance3D(centerPt, pt) <= radius + hgtStep / 2) {
                                    try {
                                        std::string code = IJH2DQG_str(ijh.row, ijh.column, ijh.layer, static_cast<uint8_t>(level));

                                        Json::Value item;
                                        item["code"] = code;
                                        item["bottom"] = TEMP.bottom;
                                        item["top"] = TEMP.top;
                                        Json::Value centerArr(Json::arrayValue);
                                        centerArr.append(TEMP.longitude);
                                        centerArr.append(TEMP.latitude);
                                        centerArr.append(TEMP.height);
                                        item["center"] = centerArr;
                                        item["minlon"] = TEMP.west;
                                        item["minlat"] = TEMP.south;
                                        item["maxlon"] = TEMP.east;
                                        item["maxlat"] = TEMP.north;

                                        localCells.push_back(item);
                                    } catch (...) {}
                                }
                            }
                        }
                    }
                    return localCells;
                }));
            }

            for (auto& f : futures) {
                auto vec = f.get();
                for (const auto& v : vec) {
                    cells.append(v);
                }
            }
        }

        // 构建成功响应
        Json::Value res;
        res["status"] = "success";
        res["data"]["count"] = static_cast<Json::UInt>(cells.size()); // 返回网格数量
        res["data"]["cells"] = cells;                               // 返回网格详细信息
        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception &e) {
        // 捕获并处理所有异常，返回500错误
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/**
 * @brief 线矩形缓冲区网格化接口（管道网格化）
 *
 * 该接口根据给定的三维线坐标序列、宽度、高度和网格层级，计算并返回线的矩形缓冲区内的所有网格信息。
 * 使用lineBufferFilled函数实现局部网格化，基于DQG网格系统。
 *
 * 输入参数：
 * 1. line：三维线坐标序列，每个点包含经度、纬度和高度信息
 * 2. level：网格层级，决定网格精度（0-21）
 * 3. halfWidth：矩形缓冲区的半宽度
 * 4. halfHeight：矩形缓冲区的半高度
 *
 * @param req HTTP请求对象，包含JSON格式的请求体
 * @param callback 异步回调函数，用于返回HTTP响应
 */
void geometricGrid::getGridByPolylineAndRect(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const {
    try {
        // 获取并验证请求体是否为JSON格式
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数是否存在
        if (!body->isMember("line") || !body->isMember("level") || !body->isMember("halfWidth") || !body->isMember("halfHeight")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "缺少必需参数: line, level, halfWidth, halfHeight";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取并验证line参数格式（line应包含三维坐标点数组）
        auto lineJs = (*body)["line"];
        if (!lineJs.isArray() || lineJs.size() < 2) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "line 必须为包含至少2个点的数组";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 解析参数
        int level = (*body)["level"].asInt();              // 网格层级
        double halfWidth = (*body)["halfWidth"].asDouble(); // 半宽度
        double halfHeight = (*body)["halfHeight"].asDouble(); // 半高度

        // 验证level参数范围（0-21）
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证宽度和高度参数
        if (halfWidth <= 0 || halfHeight <= 0) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "halfWidth 和 halfHeight 必须大于 0";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 将JSON数组转换为PointLBHd向量格式
        std::vector<PointLBHd> linePoints;
        for (const auto &pt : lineJs) {
            // 验证每个点是否为包含3个元素的数组
            if (!pt.isArray() || pt.size() < 3) {
                Json::Value err;
                err["status"] = "error";
                err["message"] = "line 中的每个点必须是包含经度、纬度、高度的数组";
                auto resp = HttpResponse::newHttpJsonResponse(err);
                resp->setStatusCode(k400BadRequest);
                callback(resp);
                return;
            }

            PointLBHd point;
            point.Lng = pt[(Json::ArrayIndex)0].asDouble(); // 经度
            point.Lat = pt[(Json::ArrayIndex)1].asDouble(); // 纬度
            point.Hgt = pt[(Json::ArrayIndex)2].asDouble(); // 高度
            linePoints.push_back(point);
        }

        // 使用全局常量DQBaseTile作为基础瓦片范围
        const BaseTile& baseTile = ::getProjectBaseTile();

        // 调用lineBufferFilled函数实现线矩形缓冲区网格化
        std::vector<std::string> gridCodes;
        try {
            gridCodes = lineBufferFilled(linePoints, halfWidth, halfHeight, static_cast<uint8_t>(level), baseTile);
        } catch (const std::exception& e) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = std::string("线缓冲区网格化过程中发生错误: ") + e.what();
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // 获取每个网格的详细信息 (并行优化)
        std::vector<LatLonHei> gridDetails;
        size_t count = gridCodes.size();

        unsigned int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) numThreads = 4;

        if (count < 1000) {
            gridDetails.reserve(count);
            for (const auto& code : gridCodes) {
                try {
                    gridDetails.push_back(getLocalTileLatLon(code, baseTile));
                } catch (...) {}
            }
        } else {
            std::vector<std::future<std::vector<LatLonHei>>> futures;
            size_t chunkSize = (count + numThreads - 1) / numThreads;

            for (unsigned int i = 0; i < numThreads; ++i) {
                size_t start = i * chunkSize;
                size_t end = std::min(start + chunkSize, count);
                if (start >= end) break;

                futures.push_back(std::async(std::launch::async, [start, end, &gridCodes, &baseTile]() {
                    std::vector<LatLonHei> localDetails;
                    localDetails.reserve(end - start);
                    for (size_t k = start; k < end; ++k) {
                        try {
                            localDetails.push_back(getLocalTileLatLon(gridCodes[k], baseTile));
                        } catch (...) {}
                    }
                    return localDetails;
                }));
            }

            gridDetails.reserve(count);
            for (auto& f : futures) {
                auto vec = f.get();
                gridDetails.insert(gridDetails.end(), vec.begin(), vec.end());
            }
        }

        // 对结果进行排序
        std::sort(gridDetails.begin(), gridDetails.end(), [](const LatLonHei &a, const LatLonHei &b){
            return a.code < b.code; // 按网格编码排序
        });

        // 构造成功响应
        Json::Value res;
        res["status"] = "success";                         // 状态标识
        res["data"]["count"] = static_cast<Json::UInt>(gridDetails.size()); // 返回生成的网格数量
        res["data"]["cells"] = Json::Value(Json::arrayValue); // 创建JSON数组存储网格详细信息

        // 将网格详细信息添加到响应中
        for (const auto &g : gridDetails) {
            Json::Value item;
            item["code"] = g.code;            // 网格编码

            // 构建center数组：[经度, 纬度, 高度]
            Json::Value center(Json::arrayValue);
            center.append(g.longitude);      // 经度
            center.append(g.latitude);       // 纬度
            center.append(g.height);         // 高度
            item["center"] = center;

            item["minlon"] = g.west;          // 最小经度（西边界）
            item["maxlon"] = g.east;          // 最大经度（东边界）
            item["minlat"] = g.south;         // 最小纬度（南边界）
            item["maxlat"] = g.north;         // 最大纬度（北边界）
            item["bottom"] = g.bottom;        // 底边界
            item["top"] = g.top;              // 顶边界
            res["data"]["cells"].append(item);
        }

        auto resp = HttpResponse::newHttpJsonResponse(res);  // 创建JSON响应
        resp->setStatusCode(k200OK);                         // 设置响应状态码为200
        callback(resp);                                      // 异步返回响应
    } catch (const std::exception &e) {
        // 捕获并处理所有异常
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what(); // 包含异常信息
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError); // 设置响应状态码为500
        callback(resp);
    }
}

/**
 * @brief 将三维线进行网格化，返回覆盖该线的所有网格编码(线网格化)
 * 输入：
 * 1. 三维线（line）：由多个三维坐标点组成的数组，每个点包含经度、纬度和高度信息
 * 2. 网格层级（level）：网格的精度，取值范围为0-21，0表示最精细的网格，21表示最粗略的网格

 * @param req HTTP请求对象，包含JSON格式的请求体
 * @param callback 异步回调函数，用于返回HTTP响应
 */
void geometricGrid::getGridByLine(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const {
    try {
        // 获取并验证请求体是否为JSON格式
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数是否存在
        if (!body->isMember("line") || !body->isMember("level")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "缺少必需参数: line, level";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取并验证line参数格式（line应包含三维坐标点数组）
        auto lineJs = (*body)["line"];
        if (!lineJs.isArray() || lineJs.empty()) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "line 必须为非空数组";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 解析参数
        int level = (*body)["level"].asInt();        // 网格层级，决定网格精度

        // 验证level参数范围（0-21）
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        std::vector<std::array<double, 3>> linePoints;
        for (const auto &pt : lineJs) {
            if (!pt.isArray() || pt.size() < 3) continue;
            linePoints.push_back({
                pt[(Json::ArrayIndex)0].asDouble(),
                pt[(Json::ArrayIndex)1].asDouble(),
                pt[(Json::ArrayIndex)2].asDouble()
            });
        }

        // 验证转换后的点数量
        if (linePoints.size() < 2) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "line 必须包含至少2个有效三维点";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 使用全局常量DQBaseTile作为基础瓦片范围
        const BaseTile& baseTile = ::getProjectBaseTile();

        // 调用 singleLineToGrids2 获取严格按路径排序的网格编码
        std::vector<std::string> gridCodes = singleLineToGrids2(linePoints, level, baseTile);

        // 获取每个网格编码的详细信息，并保持原有顺序去重
        std::vector<LatLonHei> gridDetails;
        std::unordered_set<std::string> seenCodes; // 记录已经处理过的编码

        for (const std::string& code : gridCodes) {
            // 如果这个网格还没被加入过结果中
            if (seenCodes.find(code) == seenCodes.end()) {
                LatLonHei detail = getLocalTileLatLon(code, baseTile);
                gridDetails.push_back(detail);
                seenCodes.insert(code); // 标记为已处理
            }
        }

        // 构造成功响应
        Json::Value res;
        res["status"] = "success";                         // 状态标识
        res["data"]["count"] = static_cast<Json::UInt>(gridDetails.size()); // 返回生成的网格数量
        res["data"]["cells"] = Json::Value(Json::arrayValue); // 创建JSON数组存储网格详细信息

        // 将网格详细信息添加到响应中
        for (const auto &g : gridDetails) {
            Json::Value item;
            item["code"] = g.code;            // 网格编码

            // 构建center数组：[经度, 纬度, 高度]
            Json::Value center(Json::arrayValue);
            center.append(g.longitude);      // 经度
            center.append(g.latitude);       // 纬度
            center.append(g.height);         // 高度
            item["center"] = center;

            item["minlon"] = g.west;          // 最小经度（西边界）
            item["maxlon"] = g.east;          // 最大经度（东边界）
            item["minlat"] = g.south;         // 最小纬度（南边界）
            item["maxlat"] = g.north;         // 最大纬度（北边界）
            item["bottom"] = g.bottom;        // 底边界
            item["top"] = g.top;              // 顶边界
            res["data"]["cells"].append(item);
        }

        auto resp = HttpResponse::newHttpJsonResponse(res);  // 创建JSON响应
        resp->setStatusCode(k200OK);                         // 设置响应状态码为200
        callback(resp);                                      // 异步返回响应
    } catch (const std::exception &e) {
        // 捕获并处理所有异常
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what(); // 包含异常信息
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError); // 设置响应状态码为500
        callback(resp);
    }
}


/**
 * @brief 多边形局部网格化接口
 *
 * 该接口根据给定的多边形顶点坐标、网格层级、高度范围，计算并返回多边形区域内的所有网格信息。
 * 支持自定义网格层级和高度范围，使用DQG3DPolygon中的扫描线填充算法。
 *
 * 输入参数：
 * 1. polygon：多边形顶点数组，每个顶点包含经度、纬度和高度信息
 * 2. level：网格层级，决定网格精度（0-21）
 * 3. bottom：多边形区域底部高度
 * 4. top：多边形区域顶部高度
 *
 * @param req HTTP请求对象，包含JSON格式的请求体
 * @param callback 异步回调函数，用于返回HTTP响应
 */
void geometricGrid::getGridByPolygonAndHeight(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const {
    try {
        // 获取并验证请求体是否为JSON格式
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数是否存在
        if (!body->isMember("polygon") || !body->isMember("level") || !body->isMember("bottom") || !body->isMember("top")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "缺少必需参数: polygon, level, bottom, top";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取并验证polygon参数格式（polygon应包含三维坐标点数组）
        auto polygonJs = (*body)["polygon"];
        if (!polygonJs.isArray() || polygonJs.size() < 3) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "polygon 必须为包含至少3个点的数组";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 解析参数
        int level = (*body)["level"].asInt();        // 网格层级，决定网格精度
        double bottom = (*body)["bottom"].asDouble(); // 底部高度
        double top = (*body)["top"].asDouble();       // 顶部高度

        // 验证level参数范围（0-21）
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证高度参数
        if (top <= bottom) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "top 必须大于 bottom";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 将JSON数组转换为坐标向量格式（符合getPolygonGrids函数要求）
        std::vector<std::vector<double>> polygonCoords;
        for (const auto &pt : polygonJs) {
            // 跳过无效点（不是数组或数组长度小于2，无法表示坐标）
            if (!pt.isArray() || pt.size() < 2) continue;

            std::vector<double> coord;
            coord.push_back(pt[(Json::ArrayIndex)0].asDouble());   // 经度（第一个元素）
            coord.push_back(pt[(Json::ArrayIndex)1].asDouble());   // 纬度（第二个元素）

            polygonCoords.push_back(coord);
        }

        // 验证转换后的点数量
        if (polygonCoords.size() < 3) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "polygon 必须包含至少3个有效点（每个点至少包含经度和纬度）";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 使用全局常量DQBaseTile作为基础瓦片范围
        const BaseTile& baseTile = ::getProjectBaseTile();

        // 调用getPolygonGrids函数实现多边形网格化
        std::vector<std::string> gridCodes;
        try {
            // 将原始 polygon Json::Value 序列化为字符串并传入 getPolygonGridCodes
            Json::FastWriter writer;
            std::string polygonJson = writer.write(polygonJs);
            gridCodes = getPolygonGridCodes(polygonJson, top, bottom, static_cast<uint8_t>(level), baseTile);
        } catch (const std::exception& e) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = std::string("网格化过程中发生错误: ") + e.what();
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // 获取每个网格的详细信息 (并行优化)
        std::vector<LatLonHei> gridDetails;
        size_t count = gridCodes.size();

        unsigned int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) numThreads = 4;

        if (count < 1000) {
            gridDetails.reserve(count);
            for (const auto& code : gridCodes) {
                try {
                    gridDetails.push_back(getLocalTileLatLon(code, baseTile));
                } catch (...) {}
            }
        } else {
            std::vector<std::future<std::vector<LatLonHei>>> futures;
            size_t chunkSize = (count + numThreads - 1) / numThreads;

            for (unsigned int i = 0; i < numThreads; ++i) {
                size_t start = i * chunkSize;
                size_t end = std::min(start + chunkSize, count);
                if (start >= end) break;

                futures.push_back(std::async(std::launch::async, [start, end, &gridCodes, &baseTile]() {
                    std::vector<LatLonHei> localDetails;
                    localDetails.reserve(end - start);
                    for (size_t k = start; k < end; ++k) {
                        try {
                            localDetails.push_back(getLocalTileLatLon(gridCodes[k], baseTile));
                        } catch (...) {}
                    }
                    return localDetails;
                }));
            }

            gridDetails.reserve(count);
            for (auto& f : futures) {
                auto vec = f.get();
                gridDetails.insert(gridDetails.end(), vec.begin(), vec.end());
            }
        }

        // 对结果进行排序
        std::sort(gridDetails.begin(), gridDetails.end(), [](const LatLonHei &a, const LatLonHei &b){
            return a.code < b.code; // 按网格编码排序
        });

        // 构造成功响应
        Json::Value res;
        res["status"] = "success";                         // 状态标识
        res["data"]["count"] = static_cast<Json::UInt>(gridDetails.size()); // 返回生成的网格数量
        res["data"]["cells"] = Json::Value(Json::arrayValue); // 创建JSON数组存储网格详细信息

        // 将网格详细信息添加到响应中
        for (const auto &g : gridDetails) {
            Json::Value item;
            item["code"] = g.code;            // 网格编码

            // 构建center数组：[经度, 纬度, 高度]
            Json::Value center(Json::arrayValue);
            center.append(g.longitude);      // 经度
            center.append(g.latitude);       // 纬度
            center.append(g.height);         // 高度
            item["center"] = center;

            item["minlon"] = g.west;          // 最小经度（西边界）
            item["maxlon"] = g.east;          // 最大经度（东边界）
            item["minlat"] = g.south;         // 最小纬度（南边界）
            item["maxlat"] = g.north;         // 最大纬度（北边界）
            item["bottom"] = g.bottom;        // 底边界
            item["top"] = g.top;              // 顶边界
            res["data"]["cells"].append(item);
        }

        auto resp = HttpResponse::newHttpJsonResponse(res);  // 创建JSON响应
        resp->setStatusCode(k200OK);                         // 设置响应状态码为200
        callback(resp);                                      // 异步返回响应
    } catch (const std::exception &e) {
        // 捕获并处理所有异常
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what(); // 包含异常信息
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError); // 设置响应状态码为500
        callback(resp);
    }
}

/**
 * @brief 多边形局部表面网格化接口（仅生成外表面：顶/底/侧面）
 *
 * 该接口与 getGridByPolygonAndHeight 参数与返回格式完全一致，但调用的是仅对多边形体外部表面进行网格化的算法。
 * 使用 dqglib 中的 getPolygonSurfaceGridCodes 来生成外壳网格编码集合。
 *
 * @param req HTTP请求指针，包含请求参数
 * @param callback 异步回调函数，用于返回HTTP响应
 */
void geometricGrid::getSurfaceGridByPolygonAndHeight(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const {
    try {
        // 获取请求体JSON对象
        auto body = req->getJsonObject();
        if (!body) {
            // 请求体非JSON格式时返回400错误
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 必传字段检查：验证是否包含所有必需的参数
        if (!body->isMember("polygon") || !body->isMember("level") || !body->isMember("bottom") || !body->isMember("top")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "缺少必需参数: polygon, level, bottom, top";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取并验证多边形参数
        auto polygonJs = (*body)["polygon"];
        if (!polygonJs.isArray() || polygonJs.size() < 3) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "polygon 必须为包含至少3个点的数组";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 解析请求参数
        int level = (*body)["level"].asInt();
        double bottom = (*body)["bottom"].asDouble();
        double top = (*body)["top"].asDouble();

        // 参数有效性验证：网格层级必须在有效范围内
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 参数有效性验证：顶部高度必须大于底部高度
        if (top <= bottom) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "top 必须大于 bottom";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 解析多边形坐标点数据
        std::vector<std::vector<double>> polygonCoords;
        polygonCoords.reserve(polygonJs.size());

        // 遍历多边形点数组，提取经纬度坐标
        for (const auto &pt : polygonJs) {
            if (!pt.isArray() || pt.size() < 2) continue;
            polygonCoords.push_back({
                pt[(Json::ArrayIndex)0].asDouble(),  // 经度
                pt[(Json::ArrayIndex)1].asDouble()   // 纬度
            });
        }

        // 验证多边形有效点数量
        if (polygonCoords.size() < 3) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "polygon 必须包含至少3个有效点（每个点至少包含经度和纬度）";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取全局基础瓦片配置
        const BaseTile& baseTile = ::getProjectBaseTile();

        // 调用dqglib库函数获取多边形表面网格编码
        std::vector<std::string> gridCodes;
        try {
            // 将多边形JSON数组转换为字符串
            Json::FastWriter writer;
            std::string polygonJson = writer.write(polygonJs);

            // 调用核心算法：获取多边形表面网格编码
            gridCodes = getPolygonSurfaceGridCodes(
                polygonJson,                           // 多边形JSON字符串
                top,                                   // 顶部高度
                bottom,                                // 底部高度
                static_cast<uint8_t>(level),          // 网格层级
                baseTile                               // 基础瓦片配置
            );
        } catch (const std::exception& e) {
            // 网格化算法执行异常处理
            Json::Value err;
            err["status"] = "error";
            err["message"] = std::string("网格化过程中发生错误: ") + e.what();
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // 根据网格编码数量选择处理策略
        std::vector<LatLonHei> gridDetails;
        size_t count = gridCodes.size();
        unsigned int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) numThreads = 4;  // 默认线程数

        if (count < 1000) {
            // 小数据量：单线程处理
            gridDetails.reserve(count);
            for (const auto& code : gridCodes) {
                try {
                    // 将网格编码转换为经纬度详细信息
                    gridDetails.push_back(getLocalTileLatLon(code, baseTile));
                } catch (...) {
                    // 忽略单个网格编码转换失败的情况
                }
            }
        } else {
            // 大数据量：多线程并行处理
            std::vector<std::future<std::vector<LatLonHei>>> futures;
            size_t chunkSize = (count + numThreads - 1) / numThreads;

            // 创建多线程任务
            for (unsigned int i = 0; i < numThreads; ++i) {
                size_t start = i * chunkSize;
                size_t end = std::min(start + chunkSize, count);
                if (start >= end) break;

                futures.push_back(std::async(std::launch::async,
                    [start, end, &gridCodes, &baseTile]() {
                        std::vector<LatLonHei> localDetails;
                        localDetails.reserve(end - start);

                        // 处理分配给当前线程的数据块
                        for (size_t k = start; k < end; ++k) {
                            try {
                                localDetails.push_back(getLocalTileLatLon(gridCodes[k], baseTile));
                            } catch (...) {
                                // 忽略转换失败的网格
                            }
                        }
                        return localDetails;
                    }
                ));
            }

            // 收集多线程处理结果
            gridDetails.reserve(count);
            for (auto& f : futures) {
                auto vec = f.get();
                gridDetails.insert(gridDetails.end(), vec.begin(), vec.end());
            }
        }

        // 按网格编码排序，确保结果的一致性
        std::sort(gridDetails.begin(), gridDetails.end(),
            [](const LatLonHei &a, const LatLonHei &b) {
                return a.code < b.code;
            });

        // 构建成功响应JSON对象
        Json::Value res;
        res["status"] = "success";
        res["data"]["count"] = static_cast<Json::UInt>(gridDetails.size());
        res["data"]["cells"] = Json::Value(Json::arrayValue);

        // 遍历网格详细信息，构建响应数据
        for (const auto &g : gridDetails) {
            Json::Value item;
            item["code"] = g.code;  // 网格编码

            // 构建中心点坐标数组 [经度, 纬度, 高度]
            Json::Value center(Json::arrayValue);
            center.append(g.longitude);
            center.append(g.latitude);
            center.append(g.height);
            item["center"] = center;

            // 网格边界坐标
            item["minlon"] = g.west;   // 最小经度
            item["maxlon"] = g.east;   // 最大经度
            item["minlat"] = g.south;  // 最小纬度
            item["maxlat"] = g.north;  // 最大纬度

            // 高度范围
            item["bottom"] = g.bottom; // 底部高度
            item["top"] = g.top;       // 顶部高度

            res["data"]["cells"].append(item);
        }

        // 返回成功响应
        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception &e) {
        // 全局异常处理：捕获未预期的异常
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/**
 * @brief 带洞多边形网格化接口（体积填充）
 *
 * 输入 polygon 为 GeoJSON 风格的二维环列表，第 0 个环为外环（逆时针），后续为洞（顺时针）。
 * 处理流程：
 * 1) 对外环调用现有 getPolygonGridCodes 生成体积网格集合；
 * 2) 对每个洞单独调用 getPolygonGridCodes 获得洞内网格集合并从外环结果中剔除；
 * 3) 返回剩余网格的空间信息。
 */
void geometricGrid::getGridByPolygonWithHoles(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const {
    // 开启异常捕获块，确保任何错误都能被正确处理并返回给客户端
    try {
        // 获取请求体中的 JSON 对象
        auto body = req->getJsonObject();
        // 如果请求体不是有效的 JSON 格式，返回 400 错误
        if (!body) {
            Json::Value err; err["status"] = "error"; err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 检查必需参数是否存在：polygon（多边形环列表）、top（顶部高度）、bottom（底部高度）、level（层级）
        if (!body->isMember("polygon") || !body->isMember("top") || !body->isMember("bottom") || !body->isMember("level")) {
            Json::Value err; err["status"] = "error"; err["message"] = "缺少必需参数: polygon, top, bottom, level";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 获取 polygon 参数，这是一个环的数组，第 0 个是外环，后续是洞
        const auto& polygonJs = (*body)["polygon"];
        // 检查 polygon 是否为数组且不为空
        if (!polygonJs.isArray() || polygonJs.empty()) {
            Json::Value err; err["status"] = "error"; err["message"] = "polygon 必须为包含至少1个环的数组";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 提取并转换参数类型
        int level = (*body)["level"].asInt();          // 网格层级（0-21）
        double top = (*body)["top"].asDouble();        // 顶部高度
        double bottom = (*body)["bottom"].asDouble();  // 底部高度

        // 验证 level 参数范围，必须在 0-21 之间
        if (level < 0 || level > 21) {
            Json::Value err; err["status"] = "error"; err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }
        // 验证 top 必须大于 bottom
        if (top <= bottom) {
            Json::Value err; err["status"] = "error"; err["message"] = "top 必须大于 bottom";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 获取项目的基础瓦片配置，用于后续的坐标转换
        const BaseTile& baseTile = ::getProjectBaseTile();

        // 定义 lambda 函数：将环的点数组转换为标准 JSON 格式
        // 输入：环的点数组（可能包含额外的字段）
        // 输出：仅包含经纬度坐标的二维数组 [[lon1, lat1], [lon2, lat2], ...]
        auto ringToJson = [](const Json::Value& ring) -> Json::Value {
            Json::Value arr(Json::arrayValue);  // 创建结果数组
            // 遍历环中的每个点
            for (const auto& pt : ring) {
                // 跳过无效点（非数组或点数不足 2）
                if (!pt.isArray() || pt.size() < 2) continue;
                // 提取经纬度（取前两个元素）
                Json::Value p(Json::arrayValue);
                p.append(pt[(Json::ArrayIndex)0].asDouble());  // 经度
                p.append(pt[(Json::ArrayIndex)1].asDouble());  // 纬度
                arr.append(p);  // 添加到结果数组
            }
            return arr;  // 返回转换后的点数组
        };

        // 处理外环（第 0 个环）
        Json::Value outerRing = ringToJson(polygonJs[(Json::ArrayIndex)0]);
        // 验证外环至少需要 3 个点才能构成一个有效的多边形
        if (outerRing.size() < 3) {
            Json::Value err; err["status"] = "error"; err["message"] = "外环至少需要3个点";
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k400BadRequest); callback(resp); return;
        }

        // 创建 JSON 序列化器，用于将环的 JSON 转换为字符串
        Json::FastWriter writer;
        // 使用无序集合存储外环生成的网格编码（自动去重）
        std::unordered_set<std::string> outerCodes;
        try {
            // 将外环转换为 JSON 字符串格式
            // 注意：getPolygonGridCodes 期望的输入是单环点序列（而非嵌套环列表）
            std::string outerJson = writer.write(outerRing);
            // 调用网格化函数，获取外环覆盖的所有体积网格编码
            auto codes = getPolygonGridCodes(outerJson, top, bottom, static_cast<uint8_t>(level), baseTile);
            // 将所有网格编码插入集合中
            outerCodes.insert(codes.begin(), codes.end());
        } catch (const std::exception& e) {
            // 如果外环网格化失败，返回 500 错误
            Json::Value err; err["status"] = "error"; err["message"] = std::string("外环网格化失败: ") + e.what();
            auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k500InternalServerError); callback(resp); return;
        }

        // 处理洞：从第 1 个环开始，每个洞都需要从外环结果中剔除
        for (Json::ArrayIndex i = 1; i < polygonJs.size(); ++i) {
            // 转换当前洞的点数组
            Json::Value holeRing = ringToJson(polygonJs[i]);
            // 忽略无效洞（点数少于 3）
            if (holeRing.size() < 3) continue;
            try {
                // 将洞转换为 JSON 字符串
                // 同样传递单环点序列给网格化函数
                std::string holeJson = writer.write(holeRing);
                // 获取洞内所有的网格编码
                auto holeCodes = getPolygonGridCodes(holeJson, top, bottom, static_cast<uint8_t>(level), baseTile);
                // 从外环结果中删除洞内的网格（实现挖洞效果）
                for (const auto& c : holeCodes) outerCodes.erase(c);
            } catch (...) {
                // 忽略单个洞的异常以保证主流程可用
                // 这样即使某个洞处理失败，其他洞仍然会被处理
            }
        }

        // 转换网格详情：从网格编码获取详细的地理空间信息
        std::vector<std::string> finalCodes(outerCodes.begin(), outerCodes.end());  // 将集合转为向量
        std::vector<LatLonHei> gridDetails;  // 存储网格详情（经纬度、边界等）
        gridDetails.reserve(finalCodes.size());  // 预分配内存以提升性能

        // 获取 CPU 核心数用于多线程处理
        unsigned int numThreads = std::thread::hardware_concurrency();
        // 如果无法获取核心数，默认使用 4 个线程
        if (numThreads == 0) numThreads = 4;

        // 根据数据量选择处理策略：少于 1000 个网格使用单线程，否则使用多线程
        if (finalCodes.size() < 1000) {
            // 小数据量：单线程顺序处理
            for (const auto& code : finalCodes) {
                // 根据网格编码获取地理空间信息，忽略异常
                try { gridDetails.push_back(getLocalTileLatLon(code, baseTile)); } catch (...) {}
            }
        } else {
            // 大数据量：多线程并行处理以提升性能
            std::vector<std::future<std::vector<LatLonHei>>> futures;  // 存储异步任务结果
            // 计算每个线程处理的任务块大小（向上取整）
            size_t chunkSize = (finalCodes.size() + numThreads - 1) / numThreads;
            // 为每个线程创建异步任务
            for (unsigned int i = 0; i < numThreads; ++i) {
                // 计算当前线程处理的范围 [start, end)
                size_t start = i * chunkSize;
                size_t end = std::min(start + chunkSize, finalCodes.size());
                // 如果范围无效，退出循环
                if (start >= end) break;
                // 创建异步任务：并行处理指定范围内的网格编码
                futures.push_back(std::async(std::launch::async, [start, end, &finalCodes, &baseTile]() {
                    std::vector<LatLonHei> local;  // 线程局部结果
                    local.reserve(end - start);  // 预分配内存
                    // 处理分配给该线程的所有网格编码
                    for (size_t k = start; k < end; ++k) {
                        try { local.push_back(getLocalTileLatLon(finalCodes[k], baseTile)); } catch (...) {}
                    }
                    return local;  // 返回处理结果
                }));
            }
            // 等待所有异步任务完成并收集结果
            for (auto& f : futures) {
                auto part = f.get();  // 获取任务结果（阻塞等待）
                // 将部分结果合并到最终结果中
                gridDetails.insert(gridDetails.end(), part.begin(), part.end());
            }
        }

        // 按网格编码排序，确保结果有序
        std::sort(gridDetails.begin(), gridDetails.end(), [](const LatLonHei &a, const LatLonHei &b){ return a.code < b.code; });

        // 构建响应 JSON 对象
        Json::Value res;
        res["status"] = "success";  // 设置状态为成功
        res["data"]["count"] = static_cast<Json::UInt>(gridDetails.size());  // 网格总数
        res["data"]["cells"] = Json::Value(Json::arrayValue);  // 网格详情数组

        // 遍历所有网格，填充详细空间信息
        for (const auto &g : gridDetails) {
            Json::Value item;
            item["code"] = g.code;  // 网格编码
            // 中心点坐标数组 [经度, 纬度, 高度]
            Json::Value center(Json::arrayValue);
            center.append(g.longitude);  // 中心经度
            center.append(g.latitude);   // 中心纬度
            center.append(g.height);     // 中心高度
            item["center"] = center;
            // 边界信息
            item["minlon"] = g.west;    // 最小经度（西边界）
            item["maxlon"] = g.east;    // 最大经度（东边界）
            item["minlat"] = g.south;   // 最小纬度（南边界）
            item["maxlat"] = g.north;   // 最大纬度（北边界）
            item["bottom"] = g.bottom;  // 底部高度
            item["top"] = g.top;        // 顶部高度
            // 添加到结果数组
            res["data"]["cells"].append(item);
        }

        // 创建 HTTP 响应并设置状态码为 200（成功）
        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k200OK);
        // 调用回调函数返回响应
        callback(resp);
    } catch (const std::exception &e) {
        // 捕获未处理的异常，返回 500 错误
        Json::Value err; err["status"] = "error"; err["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(err); resp->setStatusCode(k500InternalServerError); callback(resp);
    }
}
