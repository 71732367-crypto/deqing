#include "../controller/api_multiSource_basicGrid.h"
#include <drogon/HttpTypes.h>
#include <json/json.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/GlobalBaseTile.h>
#include <dqg/DQG2D.h>

using namespace api::multiSource;

/// 接口函数：根据给定的经纬度、高度和层级获取网格详细信息（编码、边框、中心点、行列层号）。
/// req HTTP 请求对象，包含客户端发送的数据（JSON 格式），需包括 longitude、latitude、height、level 字段
/// 返回的 JSON 数据包含 status、data 、 message字段，其中 status 表示处理结果（success 表示成功，error 表示失败），message 提供错误信息
/// 注意：此函数使用全局常量 projectBaseTile 作为基础瓦片进行局部网格计算
void basicGrid::getGridByPoint(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 解析请求体中的 JSON 数据
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数是否存在
        if (!jsonBody->isMember("longitude") || !jsonBody->isMember("latitude") ||
            !jsonBody->isMember("height") || !jsonBody->isMember("level")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: longitude, latitude, height, level";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 提取并转换输入参数
        double longitude = (*jsonBody)["longitude"].asDouble();
        double latitude = (*jsonBody)["latitude"].asDouble();
        double height = (*jsonBody)["height"].asDouble();
        uint8_t level = static_cast<uint8_t>((*jsonBody)["level"].asInt());

        // 对输入参数进行合法性校验
        if (longitude < -180.0 || longitude > 180.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "经度必须在-180到180之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (latitude < -90.0 || latitude > 90.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "纬度必须在-90到90之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (level < 1 || level > 15) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "层级必须在1到15之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查坐标是否在局部基础瓦片范围内
        const BaseTile& baseTile = ::getProjectBaseTile();
        if (longitude < baseTile.west || longitude > baseTile.east ||
            latitude < baseTile.south || latitude > baseTile.north ||
            height < baseTile.bottom || height > baseTile.top) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "坐标超出局部基础瓦片范围。经度范围：[" +
                std::to_string(baseTile.west) + ", " + std::to_string(baseTile.east) +
                "]，纬度范围：[" + std::to_string(baseTile.south) + ", " + std::to_string(baseTile.north) +
                "]，高度范围：[" + std::to_string(baseTile.bottom) + ", " + std::to_string(baseTile.top) + "]";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用核心算法函数生成局部网格编码
        std::string gridCode = getLocalCode(level, longitude, latitude, height, baseTile);

        // 获取网格边界和中心点信息
        LatLonHei boundary = getLocalTileLatLon(gridCode, baseTile);

        // 获取行列层号信息
        IJH ijh = getLocalTileRHC(gridCode);

        // 构造成功响应数据
        Json::Value response;
        response["status"] = "success";

        // 局部网格编码
        response["data"]["code"] = gridCode;

        // 中心点信息 [longitude, latitude, height]
        Json::Value centerArray(Json::arrayValue);
        centerArray.append(boundary.longitude);
        centerArray.append(boundary.latitude);
        centerArray.append(boundary.height);
        response["data"]["center"] = centerArray;

        // 边框信息
        response["data"]["minlon"] = boundary.west;
        response["data"]["maxlon"] = boundary.east;
        response["data"]["minlat"] = boundary.south;
        response["data"]["maxlat"] = boundary.north;
        response["data"]["top"] = boundary.top;
        response["data"]["bottom"] = boundary.bottom;

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        // 捕获异常并构造错误响应
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/// 接口函数：根据经纬度和层级获取网格编码
/// req HTTP 请求对象，包含客户端发送的数据（JSON 格式），需包括 longitude、latitude、level 字段
/// 返回的 JSON 数据包含 status、data 、 message字段，其中 status 表示处理结果（success 表示成功，error 表示失败），message 提供错误信息
void basicGrid::getCodeByLB(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 解析请求体中的 JSON 数据
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数是否存在
        if (!jsonBody->isMember("longitude") || !jsonBody->isMember("latitude") ||
            !jsonBody->isMember("level")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: longitude, latitude, level";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 提取并转换输入参数
        double longitude = (*jsonBody)["longitude"].asDouble();
        double latitude = (*jsonBody)["latitude"].asDouble();
        uint8_t level = static_cast<uint8_t>((*jsonBody)["level"].asInt());

        // 对输入参数进行合法性校验
        if (longitude < -180.0 || longitude > 180.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "经度必须在-180到180之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (latitude < -90.0 || latitude > 90.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "纬度必须在-90到90之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (level < 1 || level > 15) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "层级必须在1到15之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用LB2DQG_str函数获取网格编码
        std::string gridCode = LB2DQG_str(longitude, latitude, level);

        // 构造成功响应数据
        Json::Value response;
        response["status"] = "success";
        response["data"]["longitude"] = longitude;
        response["data"]["latitude"] = latitude;
        response["data"]["level"] = static_cast<int>(level);
        response["data"]["gridCode"] = gridCode;

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        // 捕获异常并构造错误响应
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}


/// 接口函数：将给定的局部网格编码转换为其对应中心点的经纬度和高度信息。
/// req HTTP 请求对象，包含客户端发送的数据（JSON 格式），需包括 gridCode 字段
/// 返回的 JSON 数据包含 status、data 、 message字段，其中 status 表示处理结果（success 表示成功，error 表示失败），message 提供错误信息
/// 注意：此函数使用全局常量 projectBaseTile 作为基础瓦片进行局部网格解码
void basicGrid::getCenterByGrid(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 解析请求体中的 JSON 数据
        auto jsonBody = req->getJsonObject();

        // 校验 JSON 数据有效性
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查是否提供了 gridCode 参数
        if (!jsonBody->isMember("gridCode")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: gridCode";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 提取并校验网格编码参数
        std::string gridCode = (*jsonBody)["gridCode"].asString();
        if (gridCode.empty()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "gridCode 不能为空";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用局部网格解码函数获取中心点
        LatLonHei center = getLocalTileLatLon(gridCode, ::getProjectBaseTile());

        // 构造成功响应数据
        Json::Value response;
        response["status"] = "success";
        response["data"]["gridCode"] = gridCode;
        response["data"]["longitude"] = center.longitude;
        response["data"]["latitude"] = center.latitude;
        response["data"]["height"] = center.height;

        // 额外返回层级信息（局部网格编码长度）
        if (!gridCode.empty()) {
            response["data"]["level"] = static_cast<int>(gridCode.length());
        }

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        // 异常处理：捕获所有标准异常并返回500错误
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}


/// 接口函数：根据给定的局部网格编码计算其边界坐标信息。
/// req HTTP 请求对象，包含客户端发送的数据（JSON 格式），需包括 gridCode 字段
/// 返回的 JSON 数据包含 status、data 、 message字段，其中 status 表示处理结果（success 表示成功，error 表示失败），message 提供错误信息
/// 注意：此函数使用全局常量 projectBaseTile 作为基础瓦片进行局部网格边界计算
void basicGrid::getGridBoundaryByCode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 解析请求体中的 JSON 数据
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查是否提供了 gridCode 参数
        if (!jsonBody->isMember("gridCode")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: gridCode";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取并校验 gridCode 是否为空
        std::string gridCode = (*jsonBody)["gridCode"].asString();
        if (gridCode.empty()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "gridCode 不能为空";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用局部网格解码函数获取边界信息
        LatLonHei boundary = getLocalTileLatLon(gridCode, ::getProjectBaseTile());
        
        // 获取行列高信息
        IJH ijh = getLocalTileRHC(gridCode);

        // 构造成功响应数据
        Json::Value response;
        response["status"] = "success";
        response["data"]["code"] = gridCode;  // 返回输入的网格编码
        response["data"]["west"] = boundary.west;      // 西边界
        response["data"]["south"] = boundary.south;    // 南边界
        response["data"]["east"] = boundary.east;      // 东边界
        response["data"]["north"] = boundary.north;    // 北边界
        response["data"]["bottom"] = boundary.bottom;   // 返回包围盒底部高度
        response["data"]["top"] = boundary.top;        // 返回包围盒顶部高度
        response["data"]["center"]["longitude"] = boundary.longitude; // 返回中心点经纬度和高度
        response["data"]["center"]["latitude"] = boundary.latitude;
        response["data"]["center"]["height"] = boundary.height;
        response["data"]["row"] = static_cast<int>(ijh.row); // 返回行号
        response["data"]["column"] = static_cast<int>(ijh.column); //列号
        response["data"]["layer"] = static_cast<int>(ijh.layer);  //垂直层级
        response["data"]["level"] = static_cast<int>(gridCode.length());  //水平层级（局部网格编码长度）
        response["data"]["octNum"] = 0; // 局部网格没有八叉树编号，设为0

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        // 异常处理：捕获所有标准异常，并返回 500 错误
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/// 接口函数：获取指定网格编码在特定层级的父级网格编码。
/// req HTTP 请求对象，包含客户端发送的数据（JSON 格式），需包括 gridCode 和 level 字段
/// 返回的 JSON 数据包含 status、data 、 message字段，其中 status 表示处理结果（success 表示成功，error 表示失败），message 提供错误信息
void basicGrid::getLevelFatherCode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查是否提供了必要的参数
        if (!jsonBody->isMember("gridCode") || !jsonBody->isMember("level")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: gridCode, level";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        std::string gridCode = (*jsonBody)["gridCode"].asString();
        int level = (*jsonBody)["level"].asInt();

        // 校验 gridCode 是否为空
        if (gridCode.empty()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "gridCode 不能为空";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 校验 level 的取值范围
        if (level < 0 || level > 21) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "level 必须在0到21之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用库中函数获取指定层级的父网格编码
        std::string parentCode = ::getLevelFatherCode(gridCode, static_cast<uint8_t>(level));

        Json::Value response;
        response["status"] = "success";
        response["data"]["gridCode"] = gridCode;
        response["data"]["level"] = level;
        response["data"]["parentCode"] = parentCode;

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/// 接口函数：获取指定网格编码的所有子级网格编码。
/// req HTTP 请求对象，包含客户端发送的数据（JSON 格式），需包括 gridCode 字段
/// 返回的 JSON 数据包含 status、data 、 message字段，其中 status 表示处理结果（success 表示成功，error 表示失败），message 提供错误信息
void basicGrid::getChildCode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 解析请求体中的JSON数据
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查是否提供了必需参数 gridCode
        if (!jsonBody->isMember("gridCode")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: gridCode";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 获取并校验 gridCode 是否为空
        std::string parentCode = (*jsonBody)["gridCode"].asString();
        if (parentCode.empty()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "gridCode 不能为空";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用 dqglib 中的全局函数 getChildCode，使用全局作用域避免与当前成员函数同名冲突
        std::vector<std::string> children = ::getChildCode(parentCode);

        // 构造成功响应的JSON结构
        Json::Value response;
        response["status"] = "success";
        response["data"]["gridCode"] = parentCode;
        Json::Value arr(Json::arrayValue);
        for (const auto &c : children) arr.append(c);
        response["data"]["childcode"] = arr;

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        // 异常处理：捕获标准异常并返回500错误信息
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/// 接口函数：获取全局基础瓦片数据
/// req HTTP 请求对象
/// 返回的 JSON 数据包含 status、data 字段，其中 data 包含全局基础瓦片的边界信息
void basicGrid::getProjectBaseTile(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 获取全局基础瓦片数据
        const BaseTile& baseTile = ::getProjectBaseTile();
        
        // 构造成功响应数据
        Json::Value response;
        response["status"] = "success";
        response["data"]["west"] = baseTile.west;
        response["data"]["south"] = baseTile.south;
        response["data"]["east"] = baseTile.east;
        response["data"]["north"] = baseTile.north;
        response["data"]["bottom"] = baseTile.bottom;
        response["data"]["top"] = baseTile.top;
        
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);
        
    } catch (const std::exception& e) {
        // 异常处理：捕获标准异常并返回500错误信息
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}


/// 接口函数：局部网格编码转换为全球网格编码。
/// req 需包含 localCode 字段；可选字段：无（使用项目基础瓦片推导局部配置）
/// 返回 data.globalCode 为对应的全球编码
void basicGrid::getLocalToGlobal(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (!jsonBody->isMember("localCode")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: localCode";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        const std::string localCode = (*jsonBody)["localCode"].asString();
        if (localCode.empty()) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "localCode 不能为空";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 使用项目基础瓦片构造区域四角点，推导局部格网配置
        const BaseTile& baseTile = ::getProjectBaseTile();
        PointLBd A{baseTile.west, baseTile.north};
        PointLBd B{baseTile.east, baseTile.north};
        PointLBd C{baseTile.west, baseTile.south};
        PointLBd D{baseTile.east, baseTile.south};

        LocalGridConfig config = localGridConfig(A, B, C, D);
        std::string globalCode;
        try {
            globalCode = localToGlobal(localCode, config);
            std::cout << globalCode << std::endl;
        }

        catch (const std::exception& e) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = std::string("转换失败: ") + e.what();
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        Json::Value response;
        response["status"] = "success";
        response["data"]["localCode"] = localCode;
        response["data"]["globalCode"] = globalCode;
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);
    } catch (const std::exception& e) {
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

///接口函数：经纬高点转局部网格编码
void basicGrid::getGridCodeByPoint(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 解析请求体中的 JSON 数据
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "请求体必须是有效的JSON格式";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数是否存在
        if (!jsonBody->isMember("longitude") || !jsonBody->isMember("latitude") ||
            !jsonBody->isMember("height") || !jsonBody->isMember("level")) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "缺少必需参数: longitude, latitude, height, level";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 提取并转换输入参数
        double longitude = (*jsonBody)["longitude"].asDouble();
        double latitude = (*jsonBody)["latitude"].asDouble();
        double height = (*jsonBody)["height"].asDouble();
        uint8_t level = static_cast<uint8_t>((*jsonBody)["level"].asInt());

        // 对输入参数进行合法性校验
        if (longitude < -180.0 || longitude > 180.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "经度必须在-180到180之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (latitude < -90.0 || latitude > 90.0) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "纬度必须在-90到90之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        if (level < 1 || level > 15) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "层级必须在1到15之间";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查坐标是否在局部基础瓦片范围内
        const BaseTile& baseTile = ::getProjectBaseTile();
        if (longitude < baseTile.west || longitude > baseTile.east ||
            latitude < baseTile.south || latitude > baseTile.north ||
            height < baseTile.bottom || height > baseTile.top) {
            Json::Value response;
            response["status"] = "error";
            response["message"] = "坐标超出局部基础瓦片范围。经度范围：[" +
                std::to_string(baseTile.west) + ", " + std::to_string(baseTile.east) +
                "]，纬度范围：[" + std::to_string(baseTile.south) + ", " + std::to_string(baseTile.north) +
                "]，高度范围：[" + std::to_string(baseTile.bottom) + ", " + std::to_string(baseTile.top) + "]";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 调用核心算法函数生成局部网格编码
        std::string gridCode = getLocalCode(level, longitude, latitude, height, baseTile);


        // 构造成功响应数据
        Json::Value response;
        response["status"] = "success";

        // 局部网格编码
        response["code"] = gridCode;


        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception& e) {
        // 捕获异常并构造错误响应
        Json::Value response;
        response["status"] = "error";
        response["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}
