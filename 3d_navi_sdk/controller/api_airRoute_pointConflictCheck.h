#pragma once

#include <drogon/HttpController.h>

using namespace drogon;

class api_airRoute_pointConflictCheck : public drogon::HttpController<api_airRoute_pointConflictCheck>
{
public:
    METHOD_LIST_BEGIN
    // 冲突检测：点-球形缓冲区与编码集合的交集判断
    ADD_METHOD_TO(api_airRoute_pointConflictCheck::pointBuffer, "/api/airRoute/conflictCheck/pointBuffer", Post);
    METHOD_LIST_END

    // 冲突检测：点-球形缓冲区与编码集合的交集判断
    void pointBuffer(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback);

};