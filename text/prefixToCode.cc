








//text文件夹为测试文档在项目中无实际作用








#include "prefixToCode.h"
#include <drogon/HttpTypes.h>
#include <json/json.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/GlobalBaseTile.h>
#include <string>

using namespace api::multiSource;

void redisGrid::getGridsByPrefix(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            Json::Value response; response["status"] = "error"; response["message"] = "请求体必须是JSON";
            callback(HttpResponse::newHttpJsonResponse(response));
            return;
        }

        if (!jsonBody->isMember("prefix")) {
            Json::Value response; response["status"] = "error"; response["message"] = "缺少 prefix 参数";
            callback(HttpResponse::newHttpJsonResponse(response));
            return;
        }

        std::string prefix = (*jsonBody)["prefix"].asString();
        // 新增：提取游标 (cursor)，默认为 "0" 表示从头开始
        std::string cursor = jsonBody->isMember("cursor") ? (*jsonBody)["cursor"].asString() : "0";
        // 新增：提取单次扫描数量，默认 2000
        int count = jsonBody->isMember("count") ? (*jsonBody)["count"].asInt() : 4000;
        // 新增：层级筛选，0 表示不限层级
        int level = jsonBody->isMember("level") ? (*jsonBody)["level"].asInt() : 0;

        auto redisClient = drogon::app().getRedisClient();
        if (!redisClient) {
            Json::Value response; response["status"] = "error"; response["message"] = "Redis未连接";
            callback(HttpResponse::newHttpJsonResponse(response));
            return;
        }

        std::string pattern = prefix + "*";

        // 改用 SCAN 命令进行非阻塞游标迭代
        redisClient->execCommandAsync(
            [callback, prefix, level](const drogon::nosql::RedisResult &r) {
                Json::Value response;
                response["status"] = "success";
                Json::Value dataArray(Json::arrayValue);
                std::string nextCursor = "0";

                // SCAN 返回的是一个数组，[0] 是下次的游标，[1] 是匹配的 key 数组
               // SCAN 返回的是一个数组，[0] 是下次的游标，[1] 是匹配的 key 数组
                if (r.type() == drogon::nosql::RedisResultType::kArray) {
                    auto resultArray = r.asArray();
                    if (resultArray.size() >= 2) {
                        nextCursor = resultArray[0].asString();
                        auto keys = resultArray[1].asArray();

                        const BaseTile& baseTile = ::getProjectBaseTile();

                        // ⭐️ 新增：定义一个 set 收集本次扫描到的层级
                        std::set<size_t> foundLevels;

                        for (const auto &item : keys) {
                            std::string fullKey = item.asString();

                            if (fullKey.find(prefix) == 0) {
                                std::string gridCode = fullKey.substr(prefix.length());

                                // 记录当前网格层级
                                foundLevels.insert(gridCode.length());

                                if (level > 0 && gridCode.length() != level) {
                                    continue;
                                }

                                Json::Value gridInfo;
                                gridInfo["gridCode"] = gridCode;

                                try {
                                    LatLonHei boundary = getLocalTileLatLon(gridCode, baseTile);
                                    gridInfo["west"] = boundary.west;
                                    gridInfo["south"] = boundary.south;
                                    gridInfo["east"] = boundary.east;
                                    gridInfo["north"] = boundary.north;
                                    gridInfo["bottom"] = boundary.bottom;
                                    gridInfo["top"] = boundary.top;

                                    Json::Value centerArray(Json::arrayValue);
                                    centerArray.append(boundary.longitude);
                                    centerArray.append(boundary.latitude);
                                    centerArray.append(boundary.height);
                                    gridInfo["center"] = centerArray;
                                } catch (...) {
                                    continue;
                                }
                                dataArray.append(gridInfo);
                            }
                        }

                        // ⭐️ 新增：将 set 转换为 JSON 数组返回给前端
                        Json::Value levelArray(Json::arrayValue);
                        for (auto l : foundLevels) {
                            levelArray.append((int)l);
                        }
                        response["renderedLevels"] = levelArray;
                    }
                }

                response["data"] = dataArray;
                response["count"] = static_cast<int>(dataArray.size());
                response["nextCursor"] = nextCursor;

                auto resp = HttpResponse::newHttpJsonResponse(response);
                resp->setStatusCode(k200OK);
                callback(resp);
            },
            [callback](const std::exception &err) {
                Json::Value response; response["status"] = "error";
                response["message"] = std::string("Redis 查询失败: ") + err.what();
                callback(HttpResponse::newHttpJsonResponse(response));
            },
            // 执行 SCAN 游标匹配
            "SCAN %s MATCH %s COUNT %d", cursor.c_str(), pattern.c_str(), count
        );

    } catch (const std::exception& e) {
        Json::Value response; response["status"] = "error";
        response["message"] = std::string("内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}