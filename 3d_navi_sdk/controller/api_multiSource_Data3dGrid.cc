
#include "../controller/api_multiSource_Data3dGrid.h"
#include <drogon/HttpTypes.h>
#include <drogon/drogon.h>
#include <json/json.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/DQG3DPolygon.h>
#include <dqg/DQGMathBasic.h>
#include <dqg/Data.h>
#include <dqg/Extractor.h>
#include <dqg/GlobalBaseTile.h>
#include <vector>
#include <optional>
#include <thread>
#include <memory>
#include <algorithm>
#include <unordered_set>

using namespace api::multiSource;

namespace {

// 常量定义
constexpr int kMinLevel = 0;                    // 最小网格层级
constexpr int kMaxLevel = 21;                   // 最大网格层级
constexpr double kMinLongitude = -360.0;        // 最小经度
constexpr double kMaxLongitude = 360.0;         // 最大经度
constexpr double kMinLatitude = -90.0;          // 最小纬度
constexpr double kMaxLatitude = 90.0;           // 最大纬度
constexpr size_t kHugeGridThreshold = 100000000; // 大网格阈值，超过此值使用append方式构建JSON

/**
 * @brief 创建默认的DQG目标区域
 *
 * @return LatLonHei 默认目标区域配置
 */
LatLonHei createDQDefaultTarget()
{
    LatLonHei target{};
    target.latitude = 30.558547181477985;
    target.longitude = 119.97828274319954;
    target.height = 150.0;
    target.west = 119.84367258256452;
    target.south = 30.49953220562118;
    target.east = 120.11289290383456;
    target.north = 30.61756215733479;
    target.bottom = 0.0;
    target.top = 600.0;
    target.code = "";
    return target;
}

/**
 * @brief 发送错误响应
 *
 * @param callback HTTP响应回调函数
 * @param message 错误消息
 * @param statusCode HTTP状态码，默认为400
 */
void sendErrorResponse(
    const std::function<void(const HttpResponsePtr&)>& callback,
    const std::string& message,
    HttpStatusCode statusCode = k400BadRequest)
{
    Json::Value response;
    response["status"] = "error";
    response["message"] = message;
    auto resp = HttpResponse::newHttpJsonResponse(response);
    resp->setStatusCode(statusCode);
    callback(resp);
}

/**
 * @brief 异步发送错误响应
 *
 * @param callback HTTP响应回调函数
 * @param message 错误消息
 * @param statusCode HTTP状态码，默认为500
 */
void sendErrorResponseAsync(
    const std::function<void(const HttpResponsePtr&)>& callback,
    const std::string& message,
    HttpStatusCode statusCode = k500InternalServerError)
{
    Json::Value response;
    response["status"] = "error";
    response["message"] = message;

    drogon::app().getLoop()->queueInLoop([response = std::move(response), callback, statusCode]() {
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(statusCode);
        callback(resp);
    });
}

/**
 * @brief 异步发送成功响应
 *
 * @param callback HTTP响应回调函数
 * @param data 响应数据
 */
void sendSuccessResponseAsync(
    const std::function<void(const HttpResponsePtr&)>& callback,
    Json::Value&& data)
{
    Json::Value response;
    response["status"] = "success";
    response["data"] = std::move(data);

    drogon::app().getLoop()->queueInLoop([response = std::move(response), callback]() {
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        resp->addHeader("Access-Control-Allow-Origin", "*");
        resp->addHeader("Content-Type", "application/json; charset=utf-8");
        callback(resp);
    });
}

/**
 * @brief 将网格信息转换为JSON格式
 *
 * @param grid 网格信息
 * @return Json::Value JSON格式的网格信息
 */
Json::Value convertGridToJson(const LatLonHei& grid)
{
    Json::Value item;
    item["code"] = grid.code;

    Json::Value center(Json::arrayValue);
    center.append(grid.longitude);
    center.append(grid.latitude);
    center.append(grid.height);
    item["center"] = center;

    item["minlon"] = grid.west;
    item["maxlon"] = grid.east;
    item["minlat"] = grid.south;
    item["maxlat"] = grid.north;
    item["bottom"] = grid.bottom;
    item["top"] = grid.top;

    return item;
}

/**
 * @brief 根据网格数量确定批处理大小
 *
 * 根据网格总数动态确定批处理大小，以平衡内存使用和处理效率
 *
 * @param gridCount 网格总数
 * @return size_t 批处理大小
 */
size_t determineBatchSize(size_t gridCount)
{
    if (gridCount > 1000000000) return 500;
    if (gridCount > 500000000) return 1000;
    if (gridCount > 100000000) return 2500;
    if (gridCount > 50000000) return 5000;
    if (gridCount > 10000000) return 10000;
    if (gridCount > 1000000) return 25000;
    return 50000;
}

/**
 * @brief 验证网格层级是否有效
 *
 * @param level 网格层级
 * @param errorMsg 错误消息输出参数
 * @return bool 验证结果
 */
bool validateLevel(int level, std::string& errorMsg)
{
    if (level < kMinLevel || level > kMaxLevel) {
        errorMsg = "level 必须在 " + std::to_string(kMinLevel) + " 到 " + std::to_string(kMaxLevel) + " 之间";
        return false;
    }
    return true;
}

/**
 * @brief 验证高度范围是否有效
 *
 * @param bottom 底部高度
 * @param top 顶部高度
 * @param errorMsg 错误消息输出参数
 * @return bool 验证结果
 */
bool validateHeight(double bottom, double top, std::string& errorMsg)
{
    if (top <= bottom) {
        errorMsg = "top 必须大于 bottom";
        return false;
    }
    return true;
}

/**
 * @brief 验证坐标是否在有效范围内
 *
 * @param target 目标区域
 * @param errorMsg 错误消息输出参数
 * @return bool 验证结果
 */
bool validateCoordinates(const LatLonHei& target, std::string& errorMsg)
{
    if (target.west < kMinLongitude || target.west > kMaxLongitude ||
        target.east < kMinLongitude || target.east > kMaxLongitude ||
        target.north < kMinLatitude || target.north > kMaxLatitude ||
        target.south < kMinLatitude || target.south > kMaxLatitude) {
        errorMsg = "边界坐标超出合法范围";
        return false;
    }
    return true;
}

/**
 * @brief 从点数组解析边界范围
 *
 * @param range 点数组，每个点为[longitude, latitude]
 * @param bottom 底部高度
 * @param top 顶部高度
 * @param errorMsg 错误消息输出参数
 * @return std::optional<LatLonHei> 解析结果，失败时为nullopt
 */
std::optional<LatLonHei> parseBoundsFromRange(
    const Json::Value& range,
    double bottom,
    double top,
    std::string& errorMsg)
{
    if (!range.isArray() || range.size() < 2) {
        errorMsg = "range 数组至少需要包含2个点";
        return std::nullopt;
    }

    double minLat = 90.0, maxLat = -90.0;
    double minLon = 360.0, maxLon = -360.0;

    for (const auto& point : range) {
        if (!point.isArray() || point.size() != 2) {
            errorMsg = "range 中的每个点必须是包含2个元素的数组 [longitude, latitude]";
            return std::nullopt;
        }

        double lon = point[0].asDouble();
        double lat = point[1].asDouble();

        minLat = std::min(minLat, lat);
        maxLat = std::max(maxLat, lat);
        minLon = std::min(minLon, lon);
        maxLon = std::max(maxLon, lon);
    }

    LatLonHei result{};
    result.west = minLon;
    result.east = maxLon;
    result.south = minLat;
    result.north = maxLat;
    result.bottom = bottom;
    result.top = top;
    result.latitude = (result.north + result.south) / 2.0;
    result.longitude = (result.east + result.west) / 2.0;
    result.height = (result.top + result.bottom) / 2.0;

    return result;
}

/**
 * @brief 从基本方向(west, east, north, south)解析边界范围
 *
 * @param json JSON对象
 * @param bottom 底部高度
 * @param top 顶部高度
 * @return LatLonHei 解析结果
 */
LatLonHei parseBoundsFromCardinal(const Json::Value& json, double bottom, double top)
{
    LatLonHei result{};
    result.west = json["west"].asDouble();
    result.east = json["east"].asDouble();
    result.north = json["north"].asDouble();
    result.south = json["south"].asDouble();
    result.bottom = bottom;
    result.top = top;
    result.latitude = (result.north + result.south) / 2.0;
    result.longitude = (result.east + result.west) / 2.0;
    result.height = (result.top + result.bottom) / 2.0;
    return result;
}

/**
 * @brief 从角点(leftTop, rightBottom)解析边界范围
 *
 * @param json JSON对象
 * @param bottom 底部高度
 * @param top 顶部高度
 * @return LatLonHei 解析结果
 */
LatLonHei parseBoundsFromCorners(const Json::Value& json, double bottom, double top)
{
    LatLonHei result{};
    const auto& lt = json["leftTop"];
    const auto& rb = json["rightBottom"];
    result.north = lt["latitude"].asDouble();
    result.west = lt["longitude"].asDouble();
    result.south = rb["latitude"].asDouble();
    result.east = rb["longitude"].asDouble();
    result.bottom = bottom;
    result.top = top;
    result.latitude = (result.north + result.south) / 2.0;
    result.longitude = (result.east + result.west) / 2.0;
    result.height = (result.top + result.bottom) / 2.0;
    return result;
}

/**
 * @brief 从请求JSON中解析边界范围
 *
 * 支持三种边界定义方式：
 * 1. range: 点数组
 * 2. west/east/north/south: 基本方向
 * 3. leftTop/rightBottom: 角点
 *
 * @param json 请求JSON对象
 * @param bottom 底部高度
 * @param top 顶部高度
 * @param errorMsg 错误消息输出参数
 * @return std::optional<LatLonHei> 解析结果，失败时为nullopt
 */
std::optional<LatLonHei> parseBoundsFromRequest(
    const Json::Value& json,
    double bottom,
    double top,
    std::string& errorMsg)
{
    if (json.isMember("range") && json["range"].isArray()) {
        return parseBoundsFromRange(json["range"], bottom, top, errorMsg);
    }

    if (json.isMember("west") && json.isMember("east") &&
        json.isMember("north") && json.isMember("south")) {
        return parseBoundsFromCardinal(json, bottom, top);
    }

    if (json.isMember("leftTop") && json.isMember("rightBottom")) {
        return parseBoundsFromCorners(json, bottom, top);
    }

    errorMsg = "必须提供边界参数或 zoneName";
    return std::nullopt;
}

/**
 * @brief 处理网格单元格数据
 *
 * 将网格单元格转换为JSON格式，并进行排序、去重和批处理
 *
 * @param gridCells 网格单元格数组
 * @param baseTileRef 基础瓦片引用
 * @return Json::Value 处理后的JSON数据
 */
Json::Value processGridCells(
    const std::vector<Gridbox>& gridCells,
    const BaseTile& baseTileRef)
{
    std::vector<LatLonHei> gridDetails;
    gridDetails.reserve(gridCells.size());

    // 将网格编码转换为经纬度信息
    for (const auto& cell : gridCells) {
        gridDetails.push_back(getLocalTileLatLon(cell.code, baseTileRef));
    }

    // 按编码排序
    std::sort(gridDetails.begin(), gridDetails.end(),
        [](const LatLonHei& a, const LatLonHei& b) { return a.code < b.code; });

    // 去除重复项
    gridDetails.erase(
        std::unique(gridDetails.begin(), gridDetails.end(),
            [](const LatLonHei& a, const LatLonHei& b) { return a.code == b.code; }),
        gridDetails.end());

    const size_t finalSize = gridDetails.size();
    const size_t batchSize = determineBatchSize(finalSize);

    Json::Value cells(Json::arrayValue);
    const bool useAppend = (finalSize > kHugeGridThreshold);

    if (!useAppend) {
        cells.resize(static_cast<Json::ArrayIndex>(finalSize));
    }

    // 批处理转换网格数据为JSON
    for (size_t i = 0; i < finalSize; i += batchSize) {
        const size_t endIdx = std::min(i + batchSize, finalSize);

        for (size_t j = i; j < endIdx; ++j) {
            Json::Value item = convertGridToJson(gridDetails[j]);

            if (useAppend) {
                cells.append(std::move(item));
            } else {
                cells[static_cast<Json::ArrayIndex>(j)] = std::move(item);
            }
        }
    }

    Json::Value data;
    data["count"] = static_cast<Json::UInt>(finalSize);
    data["cells"] = std::move(cells);

    return data;
}

/**
 * @brief 执行网格计算
 *
 * 在独立线程中执行网格计算，避免阻塞主线程
 *
 * @param target 目标区域
 * @param level 网格层级
 * @param baseTile 基础瓦片
 * @param callback HTTP响应回调函数
 */
void executeGridCalculation(
    LatLonHei target,
    int level,
    BaseTile baseTile,
    std::function<void(const HttpResponsePtr&)> callback)
{
    try {
        // 计算指定区域和层级的网格
        auto gridCellsResult = gridCubeRegion(
            target.west, target.east, target.north, target.south,
            target.bottom, target.top, level, baseTile);

        if (!gridCellsResult.has_value()) {
            sendErrorResponseAsync(callback, "网格化计算失败，请检查输入参数");
            return;
        }

        auto gridCells = std::move(gridCellsResult.value());
        if (gridCells.empty()) {
            Json::Value data;
            data["count"] = 0u;
            data["cells"] = Json::Value(Json::arrayValue);
            sendSuccessResponseAsync(callback, std::move(data));
            return;
        }

        // 处理网格数据并发送响应
        Json::Value data = processGridCells(gridCells, baseTile);
        sendSuccessResponseAsync(callback, std::move(data));

    } catch (const std::bad_alloc&) {
        sendErrorResponseAsync(callback,
            "内存分配失败，请减小查询范围或降低网格层级");
    } catch (const std::exception& e) {
        sendErrorResponseAsync(callback,
            std::string("网格计算异常: ") + e.what());
    }
}

}

/**
 * @brief 将立方体区域转换为网格编码的API接口
 *
 * 该接口接受HTTP POST请求，请求体为JSON格式，包含以下参数：
 * - level: 网格层级(必需)
 * - zoneName: 区域名称(可选，如果提供则使用默认区域)
 * - bottom/top: 底部/顶部高度(当未提供zoneName时必需)
 * - west/east/north/south: 边界坐标(可选，与range二选一)
 * - range: 点数组定义的边界(可选，与基本方向坐标二选一)
 * - leftTop/rightBottom: 角点定义的边界(可选，与上述两种方式二选一)
 *
 * @param req HTTP请求对象
 * @param callback HTTP响应回调函数
 */
void Data3dGrid::cubeRegionToGridcode(
    const HttpRequestPtr& req,
    std::function<void(const HttpResponsePtr&)>&& callback) const
{
    try {
        // 解析JSON请求体
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            sendErrorResponse(callback, "请求体必须是有效的JSON格式");
            return;
        }

        const Json::Value& json = *jsonBody;

        // 检查必需参数
        if (!json.isMember("level")) {
            sendErrorResponse(callback, "必须提供 level 参数");
            return;
        }
        int level = json["level"].asInt();

        // 验证层级参数
        std::string errorMsg;
        if (!validateLevel(level, errorMsg)) {
            sendErrorResponse(callback, errorMsg);
            return;
        }

        // 检查是否提供了区域名称
        std::string zoneName;
        if (json.isMember("zoneName")) {
            zoneName = json["zoneName"].asString();
        }

        // 如果未提供区域名称，则必须提供高度参数
        double bottom = 0.0, top = 0.0;
        if (zoneName.empty()) {
            if (!json.isMember("bottom") || !json.isMember("top")) {
                sendErrorResponse(callback, "未提供 zoneName 时，必须提供 bottom/top 参数");
                return;
            }
            bottom = json["bottom"].asDouble();
            top = json["top"].asDouble();

            // 验证高度参数
            if (!validateHeight(bottom, top, errorMsg)) {
                sendErrorResponse(callback, errorMsg);
                return;
            }
        }

        // 确定目标区域
        LatLonHei chosenTarget{};
        if (!zoneName.empty()) {
            // 使用默认目标区域
            chosenTarget = createDQDefaultTarget();
        } else {
            // 从请求中解析边界
            auto bounds = parseBoundsFromRequest(json, bottom, top, errorMsg);
            if (!bounds.has_value()) {
                sendErrorResponse(callback, errorMsg);
                return;
            }
            chosenTarget = bounds.value();
        }

        // 验证坐标范围
        if (!validateCoordinates(chosenTarget, errorMsg)) {
            sendErrorResponse(callback, errorMsg);
            return;
        }

        // 获取项目基础瓦片
        BaseTile baseTile = ::getProjectBaseTile();

        // 在独立线程中执行网格计算
        std::thread([chosenTarget, level, baseTile, callback = std::move(callback)]() {
            executeGridCalculation(chosenTarget, level, baseTile, callback);
        }).detach();

    } catch (const std::exception& e) {
        sendErrorResponse(callback,
            std::string("服务器内部错误: ") + e.what(),
            k500InternalServerError);
    }
}