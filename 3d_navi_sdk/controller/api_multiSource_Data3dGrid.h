#pragma once

#include <drogon/HttpController.h>

using namespace drogon;

namespace api
{
namespace multiSource
{
class Data3dGrid : public drogon::HttpController<Data3dGrid>
{
  public:
    METHOD_LIST_BEGIN
    // use METHOD_ADD to add your custom processing function here;
    // METHOD_ADD(Data3dGrid::get, "/{2}/{1}", Get); // path is /api/multiSource/Data3dGrid/{arg2}/{arg1}
    // METHOD_ADD(Data3dGrid::your_method_name, "/{1}/{2}/list", Get); // path is /api/multiSource/Data3dGrid/{arg1}/{arg2}/list
    // ADD_METHOD_TO(Data3dGrid::your_method_name, "/absolute/path/{1}/{2}/list", Get); // path is /absolute/path/{arg1}/{arg2}/list

    ADD_METHOD_TO(Data3dGrid::cubeRegionToGridcode, "/api/multiSource/basicGrid/cubeRegionToGridcode", Post); // 区域空域网格化接口

    METHOD_LIST_END
    // your declaration of processing function maybe like this:
    // void get(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, int p1, std::string p2);
    // void your_method_name(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback, double p1, int p2) const;


    /// 区域空域网格化
    void cubeRegionToGridcode(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

};
}
}
