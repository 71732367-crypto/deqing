#pragma once

#include <drogon/HttpController.h>

using namespace drogon;

namespace api
{
    namespace multiSource
    {
        /**
         * @brief 专门处理从 Redis 中读取与检索网格数据的控制器
         */
        class redisGrid : public drogon::HttpController<redisGrid>
        {
        public:
            METHOD_LIST_BEGIN
            // 注册路由：POST /api/multiSource/redisGrid/getGridsByPrefix
            ADD_METHOD_TO(redisGrid::getGridsByPrefix, "/api/multiSource/redisGrid/getGridsByPrefix", Post);
            METHOD_LIST_END

            /**
             * @brief 接口函数：前端根据指定前缀（如 "ad_"）异步从 Redis 批量检索网格编码并转换成空间几何边界
             * @param req 包含 Json 请求体，必须携带 "prefix" 字段
             */
            void getGridsByPrefix(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const;
        };
    }
}