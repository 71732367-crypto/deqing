#pragma once

#include <drogon/HttpController.h>

using namespace drogon;

namespace api
{
namespace multiSource
{
class basicGrid : public drogon::HttpController<basicGrid>
{
  public:
    METHOD_LIST_BEGIN
    // use METHOD_ADD to add your custom processing function here;
    // METHOD_ADD(basicGrid::get, "/{2}/{1}", Get); // path is /api/multiSource/basicGrid/{arg2}/{arg1}
    // METHOD_ADD(basicGrid::your_method_name, "/{1}/{2}/list", Get); // path is /api/multiSource/basicGrid/{arg1}/{arg2}/list
    // ADD_METHOD_TO(basicGrid::your_method_name, "/absolute/path/{1}/{2}/list", Get); // path is /absolute/path/{arg1}/{arg2}/list
    

    ADD_METHOD_TO(basicGrid::getCenterByGrid, "/api/multiSource/basicGrid/getCenterByGrid", Post); // 网格编码->中心坐标接口
    ADD_METHOD_TO(basicGrid::getGridByPoint, "/api/multiSource/basicGrid/getGridByPoint", Post); // 点网格化详细信息接口
    ADD_METHOD_TO(basicGrid::getGridBoundaryByCode, "/api/multiSource/basicGrid/getGridBoundaryByCode", Post); // 获取网格边界（包围盒）接口
    ADD_METHOD_TO(basicGrid::getLevelFatherCode, "/api/multiSource/basicGrid/getLevelFatherCode", Post); // 指定层级父网格编码接口
    ADD_METHOD_TO(basicGrid::getChildCode, "/api/multiSource/basicGrid/getChildCode", Post); // 单个父网格获取子网格编码接口（单数）
    ADD_METHOD_TO(basicGrid::getProjectBaseTile, "/api/multiSource/basicGrid/getProjectBaseTile", Get); // 获取全局基础瓦片数据接口
    ADD_METHOD_TO(basicGrid::getCodeByLB, "/api/multiSource/basicGrid/getCodeByLB", Post); // 根据经纬度和层级获取网格编码接口
    ADD_METHOD_TO(basicGrid::getLocalToGlobal, "/api/multiSource/basicGrid/getLocalToGlobal", Post); // 局部网格编码转换为全球网格编码接口
    ADD_METHOD_TO(basicGrid::getGridCodeByPoint, "/api/multiSource/basicGrid/getGridCodeByPoint", Post); // 点网格化详细信息接口
  
    METHOD_LIST_END
    // your declaration of processing function maybe like this:
    // void get(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, int p1, std::string p2);
    // void your_method_name(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, double p1, int p2) const;


    ///将网格编码转换为网格中心点的经纬度坐标
    void getCenterByGrid(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 根据经纬度坐标获取网格详细信息（编码、边框、中心点、行列层号）
    void getGridByPoint(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 将网格编码转换为网格边界坐标
    void getGridBoundaryByCode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 获取指定层级父级网格编码
    void getLevelFatherCode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 获取子级网格编码列表
    void getChildCode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 获取全局基础瓦片数据
    void getProjectBaseTile(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 根据经纬度和层级获取网格编码
    void getCodeByLB(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    ///接口函数：局部网格编码转换为全球网格编码。
    void getLocalToGlobal(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
    /// 接口函数：根据给定的经纬度、高度和层级获取网格编码。
    void getGridCodeByPoint(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;


};
}
}
