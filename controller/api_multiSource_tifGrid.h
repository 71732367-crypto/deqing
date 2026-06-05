#pragma once
#include <drogon/HttpController.h>

namespace api {
    namespace multiSource {

        class tifGrid : public drogon::HttpController<tifGrid> {
        public:
            METHOD_LIST_BEGIN
            // 定义路由，前端将通过这个 URL 访问该接口
            ADD_METHOD_TO(tifGrid::processTif, "/api/multiSource/tifGrid/tifToGridcode", drogon::Post);
            METHOD_LIST_END

            void processTif(const drogon::HttpRequestPtr& req,
                            std::function<void(const drogon::HttpResponsePtr&)>&& callback) const;
        };

    } // namespace multiSource
} // namespace api