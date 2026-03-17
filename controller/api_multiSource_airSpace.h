#pragma once

#include <drogon/HttpController.h>
#include <drogon/nosql/RedisClient.h>
#include <json/json.h>
#include <string>
#include <vector>

using namespace drogon;

class api_multiSource_airSpace : public drogon::HttpController<api_multiSource_airSpace>
{
public:
    METHOD_LIST_BEGIN
    // 注册适飞区域统计接口
    ADD_METHOD_TO(api_multiSource_airSpace::flyableAreaStatistics, "/api/multiSource/airSpace/flyableStatistics", Post, Options);
    // 注册全县适飞网格统计接口
    ADD_METHOD_TO(api_multiSource_airSpace::countFlyableGrids, "/api/multiSource/airSpace/countFlyableGrids", Post, Options);
    METHOD_LIST_END

    // 接口声明
    void flyableAreaStatistics(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback);
    void countFlyableGrids(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback);

private:
    // 辅助函数：生成多边形的唯一ID（基于坐标点的哈希）
    std::string generatePolygonId(const Json::Value& polygonJson);
};