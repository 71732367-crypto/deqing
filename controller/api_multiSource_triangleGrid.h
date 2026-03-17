#pragma once

#include <drogon/HttpController.h>
#include <unordered_set>
#include <string>
#include <dqg/Extractor.h>

// 前向声明
struct BaseTile;
struct PointLBHd;
struct LatLonHei;

using namespace drogon;

namespace api
{
namespace multiSource
{

// 使用 Extractor 模块中的 PolyhedronGridResult 和 computePolyhedronGridFill 函数
// 这些函数和结构体现在位于 dqg::Extractor 命名空间中

class triangleGrid : public drogon::HttpController<triangleGrid>
{
  public:
    METHOD_LIST_BEGIN
    // use METHOD_ADD to add your custom processing function here;
    // METHOD_ADD(triangleGrid::get, "/{2}/{1}", Get); // path is /api/multiSource/triangleGrid/{arg2}/{arg1}
    // METHOD_ADD(triangleGrid::your_method_name, "/{1}/{2}/list", Get); // path is /api/multiSource/triangleGrid/{arg1}/{arg2}/list
    // ADD_METHOD_TO(triangleGrid::your_method_name, "/absolute/path/{1}/{2}/list", Get); // path is /absolute/path/{arg1}/{arg2}/list
    ADD_METHOD_TO(triangleGrid::osgbToGridJson, "/api/multiSource/triangleGrid/osgbToGridJson", Post); // 从 .osgb 生成 GeoJSON 三角形网格数据库
    ADD_METHOD_TO(triangleGrid::testDatabase, "/api/multiSource/triangleGrid/testDatabase", Get); // 测试数据库连接和数据存储
    ADD_METHOD_TO(triangleGrid::fillTriangleWithCubes, "/api/multiSource/triangleGrid/fillTriangleWithCubes", Post); // 用 dqg-3d 立方体填充单个三角形
    ADD_METHOD_TO(triangleGrid::fillTetraWithCubes, "/api/multiSource/triangleGrid/fillTetraWithCubes", Post); // 用 dqg-3d 立方体填充四顶点（四面体/三棱锥）
    ADD_METHOD_TO(triangleGrid::fillPolyhedronWithCubes, "/api/multiSource/triangleGrid/fillPolyhedronWithCubes", Post); // 用 dqg-3d 立方体填充由三角面定义的不规则多面体

    METHOD_LIST_END
    // your declaration of processing function maybe like this:
    // void get(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, int p1, std::string p2);
    // void your_method_name(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, double p1, int p2) const;
    /// 从 .osgb 层级目录生成 GeoJSON 网格数据库
    void osgbToGridJson(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 测试数据库连接和数据存储
    void testDatabase(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 用 dqg-3d 立方体填充单个三角形。请求体 JSON: { "points": [ {"longitude":..,"latitude":..,"height":..}, ...3个点... ], "level": 14 }
    void fillTriangleWithCubes(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 用 dqg-3d 立方体填充由四个顶点定义的体（四面体/三棱锥）。请求体 JSON: { "points": [ {...}, ...4个点... ], "level": 14 }
    void fillTetraWithCubes(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 用 dqg-3d 立方体填充由三角面（faces）定义的不规则多面体。请求体 JSON: { "faces": [ [[lon,lat,h],[lon,lat,h],[lon,lat,h]], ... ], "level": 14 }
    void fillPolyhedronWithCubes(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
};
}
}
