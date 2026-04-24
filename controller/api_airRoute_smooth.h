#pragma once

#include <drogon/HttpController.h>
#include <drogon/utils/coroutine.h>

using namespace drogon;

namespace api
{
    namespace airRoute
    {
        /**
         * @brief 航路平滑控制器
         * 专门负责对 A* 算法生成的锯齿路径进行后处理优化
         */
        class Smooth : public drogon::HttpController<Smooth>
        {
        public:
            METHOD_LIST_BEGIN
            // 注册接口路径为 /api/airRoute/smooth/process
            METHOD_ADD(Smooth::process, "/process", Post);
            METHOD_LIST_END

            // 异步处理接口
            Task<void> process(const HttpRequestPtr req,
                              std::function<void (const HttpResponsePtr &)> callback);
        };
    }
}