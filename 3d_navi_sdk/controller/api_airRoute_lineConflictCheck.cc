#include "api_airRoute_lineConflictCheck.h"
#include <drogon/drogon.h>
#include <json/json.h>
#include <dqg/GlobalBaseTile.h>
#include <dqg/Data.h>
#include "conditionCheck.h"
#include <dqg/DQG3DProximity.h>

using drogon::HttpRequestPtr;
using drogon::HttpResponse;
using drogon::HttpResponsePtr;
using drogon::k200OK;
using drogon::k400BadRequest;
using drogon::k500InternalServerError;
using drogon::app;

// 线网格化冲突检测接口
void api_airRoute_lineConflictCheck::asyncHandleHttpRequest(const HttpRequestPtr &req, std::function<void(const HttpResponsePtr &)> &&callback)
{
    // 根据请求路径判断使用哪个接口
    std::string path = req->getPath();
    bool isFirstConflictMode = (path.find("/checkFirst") != std::string::npos);

    auto resp = std::make_shared<Json::Value>(Json::objectValue);

    try
    {
        auto body = req->getJsonObject();
        if (!body)
        {
            (*resp)["status"] = "error";
            (*resp)["message"] = "请求体必须为 JSON";
            auto out = HttpResponse::newHttpJsonResponse(*resp);
            out->setStatusCode(k400BadRequest);
            callback(out);
            return;
        }

        if (!body->isMember("points") || !(*body)["points"].isArray() || (*body)["points"].size() < 2)
        {
            (*resp)["status"] = "error";
            (*resp)["message"] = "points 必须是长度>=2的数组";
            auto out = HttpResponse::newHttpJsonResponse(*resp);
            out->setStatusCode(k400BadRequest);
            callback(out);
            return;
        }

        if (!body->isMember("startTime") || !(*body)["startTime"].isNumeric())
        {
            (*resp)["status"] = "error";
            (*resp)["message"] = "startTime 必须提供";
            auto out = HttpResponse::newHttpJsonResponse(*resp);
            out->setStatusCode(k400BadRequest);
            callback(out);
            return;
        }

        const double startTime = (*body)["startTime"].asDouble();
        int level = body->isMember("level") && (*body)["level"].isInt() ? (*body)["level"].asInt() : 14;
        if (level < 1 || level > 21)
        {
            (*resp)["status"] = "error";
            (*resp)["message"] = "level 必须在 1..21 范围内";
            auto out = HttpResponse::newHttpJsonResponse(*resp);
            out->setStatusCode(k400BadRequest);
            callback(out);
            return;
        }

        Json::Value options = body->get("condition", Json::Value(Json::objectValue));
        if (!options.isObject()) options = Json::Value(Json::objectValue);

        auto redisClient = app().getRedisClient();
        if (!redisClient)
        {
            (*resp)["status"] = "error";
            (*resp)["message"] = "Redis 客户端获取失败";
            auto out = HttpResponse::newHttpJsonResponse(*resp);
            out->setStatusCode(k500InternalServerError);
            callback(out);
            return;
        }

        const BaseTile &baseTile = getProjectBaseTile();
        const auto codes = plancheck::polylineToCodes((*body)["points"], level, baseTile);

        if (isFirstConflictMode)
        {
            // 调用第一个冲突模式的检测函数
            plancheck::checkLineConflictFirst(codes, startTime, options, redisClient,
                [callback, resp, baseTile](plancheck::ConflictResult result) {
                    if (!result.pass)
                    {
                        (*resp)["status"] = "failed";
                        (*resp)["reason"] = result.reason;

                        // 构建第一个冲突网格的详细信息
                        Json::Value gridInfo;

                        try {
                            auto gridDetail = getLocalTileLatLon(result.code, baseTile);

                            gridInfo["code"] = gridDetail.code;

                            // 构建中心点坐标数组 [经度, 纬度, 高度]
                            Json::Value center(Json::arrayValue);
                            center.append(gridDetail.longitude);
                            center.append(gridDetail.latitude);
                            center.append(gridDetail.height);
                            gridInfo["center"] = center;

                            // 网格边界坐标
                            gridInfo["minlon"] = gridDetail.west;   // 最小经度
                            gridInfo["maxlon"] = gridDetail.east;   // 最大经度
                            gridInfo["minlat"] = gridDetail.south;  // 最小纬度
                            gridInfo["maxlat"] = gridDetail.north;  // 最大纬度

                            // 高度范围
                            gridInfo["bottom"] = gridDetail.bottom; // 底部高度
                            gridInfo["top"] = gridDetail.top;       // 顶部高度

                        } catch (...) {
                            // 如果获取网格详细信息失败，只设置基本信息
                            gridInfo["code"] = result.code;
                        }

                        (*resp)["grid"] = gridInfo;

                        // 有冲突时返回400状态码
                        auto out = HttpResponse::newHttpJsonResponse(*resp);
                        out->setStatusCode(k400BadRequest);
                        callback(out);
                    }
                    else
                    {
                        (*resp)["status"] = "success";
                        (*resp)["reason"] = "无冲突";

                        // 无冲突时返回200状态码
                        auto out = HttpResponse::newHttpJsonResponse(*resp);
                        out->setStatusCode(k200OK);
                        callback(out);
                    }
                });
        }
        else
        {
            // 调用异步检测函数（返回所有冲突），在 Lambda 回调中处理结果
            plancheck::checkLineConflict(codes, startTime, options, redisClient,
                [callback, resp, baseTile](plancheck::ConflictResult result) {
                    if (!result.pass)
                    {
                        (*resp)["status"] = "failed";

                        // 构建所有冲突网格的详细信息数组
                        Json::Value gridArray(Json::arrayValue);

                        for (const auto& conflict : result.conflicts) {
                            Json::Value gridInfo;
                            gridInfo["reason"] = conflict.reason;

                            // 获取冲突网格的详细信息
                            try {
                                auto gridDetail = getLocalTileLatLon(conflict.code, baseTile);

                                gridInfo["code"] = gridDetail.code;

                                // 构建中心点坐标数组 [经度, 纬度, 高度]
                                Json::Value center(Json::arrayValue);
                                center.append(gridDetail.longitude);
                                center.append(gridDetail.latitude);
                                center.append(gridDetail.height);
                                gridInfo["center"] = center;

                                // 网格边界坐标
                                gridInfo["minlon"] = gridDetail.west;   // 最小经度
                                gridInfo["maxlon"] = gridDetail.east;   // 最大经度
                                gridInfo["minlat"] = gridDetail.south;  // 最小纬度
                                gridInfo["maxlat"] = gridDetail.north;  // 最大纬度

                                // 高度范围
                                gridInfo["bottom"] = gridDetail.bottom; // 底部高度
                                gridInfo["top"] = gridDetail.top;       // 顶部高度

                            } catch (...) {
                                // 如果获取网格详细信息失败，只设置基本信息
                                gridInfo["code"] = conflict.code;
                            }

                            gridArray.append(gridInfo);
                        }

                        (*resp)["grid"] = gridArray;

                        // 有冲突时返回400状态码
                        auto out = HttpResponse::newHttpJsonResponse(*resp);
                        out->setStatusCode(k400BadRequest);
                        callback(out);
                    }
                    else
                    {
                        (*resp)["status"] = "success";
                        (*resp)["reason"] = "无冲突";

                        // 无冲突时返回200状态码
                        auto out = HttpResponse::newHttpJsonResponse(*resp);
                        out->setStatusCode(k200OK);
                        callback(out);
                    }
                });
        }
    }
    catch (const std::exception &e)
    {
        (*resp)["status"] = "error";
        (*resp)["message"] = std::string("服务器内部错误: ") + e.what();
        auto out = HttpResponse::newHttpJsonResponse(*resp);
        out->setStatusCode(k500InternalServerError);
        callback(out);
    }
}