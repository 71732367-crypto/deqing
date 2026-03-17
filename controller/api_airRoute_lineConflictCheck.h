#pragma once

#include <drogon/HttpSimpleController.h>

using namespace drogon;

class api_airRoute_lineConflictCheck : public drogon::HttpSimpleController<api_airRoute_lineConflictCheck>
{
public:
    void asyncHandleHttpRequest(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) override;
    PATH_LIST_BEGIN
    // 线冲突检测（按编码序列与时间区间做规则校验）
    PATH_ADD("/api/airRoute/lineConflict/check", drogon::Post);
    // 线冲突检测（遇到第一个冲突就返回）
    PATH_ADD("/api/airRoute/lineConflict/checkFirst", drogon::Post);
    PATH_LIST_END
};
