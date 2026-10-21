#pragma once

#include <drogon/HttpController.h>

using namespace drogon;

namespace api
{
namespace multiSource
{

class topology : public drogon::HttpController<topology>
{
  public:
    METHOD_LIST_BEGIN
    // 1. 查询同层级邻居
    ADD_METHOD_TO(topology::getNeighbors, "/api/multiSource/basicGrid/neighbors", Post);

    // 2. 判断两个网格的空间关系（支持不同层级）
    ADD_METHOD_TO(topology::getRelation, "/api/multiSource/basicGrid/relation", Post);

    // 3. 查询多个网格的最小包围祖先网格
    ADD_METHOD_TO(topology::getCommonAncestor, "/api/multiSource/basicGrid/commonAncestor", Post);
    METHOD_LIST_END

    // ==========================================
    // 接口处理函数声明（Drogon 标准异步回调签名）
    // ==========================================

    /// 1. 查询同层级邻居
    void getNeighbors(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    /// 2. 判断两个网格的空间关系
    void getRelation(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

    /// 3. 查询两个网格的最小包围父网格
    void getCommonAncestor(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;

};

} // namespace multiSource
} // namespace api