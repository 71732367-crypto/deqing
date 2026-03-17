#pragma once

#include <drogon/HttpController.h>

using namespace drogon;

namespace api
{
namespace multiSource
{
class geometricGrid : public drogon::HttpController<geometricGrid>
{
  public:
    METHOD_LIST_BEGIN
    // use METHOD_ADD to add your custom processing function here;
    // METHOD_ADD(geometricGrid::get, "/{2}/{1}", Get); // path is /api/multiSource/geometricGrid/{arg2}/{arg1}
    // METHOD_ADD(geometricGrid::your_method_name, "/{1}/{2}/list", Get); // path is /api/multiSource/geometricGrid/{arg1}/{arg2}/list
    // ADD_METHOD_TO(geometricGrid::your_method_name, "/absolute/path/{1}/{2}/list", Get); // path is /absolute/path/{arg1}/{2}/list


    ADD_METHOD_TO(geometricGrid::getGridByPointAndRadius, "/api/multiSource/geometricGrid/getGridByPointAndRadius", Post); // 点缓冲区网格化接口

    ADD_METHOD_TO(geometricGrid::getGridByLine, "/api/multiSource/geometricGrid/getGridByLine", Post);  // 线网格化接口

    ADD_METHOD_TO(geometricGrid::getGridByPolygonAndHeight, "/api/multiSource/geometricGrid/getGridByPolygonAndHeight", Post); // 多边形网格化接口

    ADD_METHOD_TO(geometricGrid::getSurfaceGridByPolygonAndHeight, "/api/multiSource/geometricGrid/getSurfaceGridByPolygonAndHeight", Post); // 多边形表面网格化接口

    ADD_METHOD_TO(geometricGrid::getGridByPolylineAndRect, "/api/multiSource/geometricGrid/getGridByPolylineAndRect", Post); // 线矩形缓冲区网格化接口

    ADD_METHOD_TO(geometricGrid::getGridByPolygonWithHoles, "/api/multiSource/geometricGrid/getGridByPolygonWithHoles", Post); // 带洞多边形网格化接口

    METHOD_LIST_END
    // your declaration of processing function maybe like this:
    // void get(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, int p1, std::string p2);
    // void your_method_name(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, double p1, int p2) const;

    void getGridByPointAndRadius(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    // 从 lineGrid 移动过来的函数声明
    void getGridByLine(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    // 多边形网格化函数声明
    void getGridByPolygonAndHeight(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    // 多边形表面（外壳）网格化函数声明
    void getSurfaceGridByPolygonAndHeight(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    // 线矩形缓冲区网格化函数声明
    void getGridByPolylineAndRect(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    // 带洞多边形网格化函数声明
    void getGridByPolygonWithHoles(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
};
}
}
