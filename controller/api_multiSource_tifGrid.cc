#include "api_multiSource_tifGrid.h"
#include <dqg/TIFtoCode.h>
#include <thread>
#include <future>
#include <chrono> // 引入时间库，用于休眠回收内存
#include <json/json.h>

using namespace api::multiSource;
using namespace drogon;

void tifGrid::processTif(const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) const {
    auto jsonBody = req->getJsonObject();
    if (!jsonBody) {
        auto resp = HttpResponse::newHttpJsonResponse(Json::Value("error"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    if (!jsonBody->isMember("tifPath") || !jsonBody->isMember("level")) {
        Json::Value error;
        error["status"] = "error";
        error["message"] = "缺少必需参数: tifPath 或 level";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    std::string tifPath = (*jsonBody)["tifPath"].asString();
    TIFGridProcessor::TIFGridConfig config;
    config.level = (*jsonBody)["level"].asInt();

    if (jsonBody->isMember("noDataValue")) {
        config.noDataValue = (*jsonBody)["noDataValue"].asDouble();
    }

    // 先给前端返回成功，防止大文件处理导致 HTTP 自动超时
    Json::Value ret;
    ret["status"] = "success";
    ret["message"] = "后台网格化正在进行，将平稳存入 Redis";
    ret["redisKey"] = "demgrid:level:" + std::to_string(config.level);
    auto resp = HttpResponse::newHttpJsonResponse(ret);
    resp->addHeader("Access-Control-Allow-Origin", "*");
    callback(resp);

    // =================================================================================
    // 独立线程处理：边算边存 Redis (带物理刹车)
    // =================================================================================
    std::thread([tifPath, config]() {
        try {
            // 获取 Redis 客户端
            auto redisClient = drogon::app().getRedisClient();
            if (!redisClient) {
                throw std::runtime_error("Redis 客户端未初始化，请检查 config.json 中的 redis_clients 配置");
            }

            // 构造 Redis 集合的 Key
            std::string redisKey = "demgrid:level:" + std::to_string(config.level);

            // 定义分批入库的回调函数
            auto batchSaver = [redisClient, redisKey](const std::vector<std::string>& batch) {
                if (batch.empty()) return;

                // ✨ 核心机制 1：声明 Promise 充当物理刹车
                std::promise<void> waitPromise;
                auto waitFuture = waitPromise.get_future();

                // 开启 Redis 事务 (Pipeline) 批量打包指令
                redisClient->newTransactionAsync([redisKey, batch, &waitPromise](const drogon::nosql::RedisTransactionPtr &trans) {
                    for(const auto& code : batch) {
                        trans->execCommandAsync(
                            [](const drogon::nosql::RedisResult &) {},
                            [](const std::exception &) {},
                            "SADD %s %s", redisKey.c_str(), code.c_str()
                        );
                    }

                    // 提交事务
                    trans->execute(
                        [batchSize = batch.size(), &waitPromise](const drogon::nosql::RedisResult &) {
                            LOG_INFO << "成功将 " << batchSize << " 个网格推入 Redis 集合";
                            waitPromise.set_value(); // 成功解锁
                        },
                        [&waitPromise](const std::exception &err) {
                            LOG_ERROR << "Redis 批量写入失败: " << err.what();
                            waitPromise.set_value(); // 失败也要解锁防死锁
                        }
                    );
                });

                // ✨ 核心机制 2：强行阻塞当前工作线程，等待 Redis 落盘完毕
                waitFuture.get();

                // ✨ 核心机制 3：物理休眠 10 毫秒，给 WSL 内存回收喘息的时间，彻底告别 OOM 闪退！
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            };

            // 开始计算并触发回调
            size_t totalCount = TIFGridProcessor::convertTIFtoGridCodes(tifPath, config, batchSaver);
            LOG_INFO << "========== TIF 异步后台处理全部完成！总计存入 Redis: " << totalCount << " 个 ==========";

        } catch (const std::exception& e) {
            LOG_ERROR << "TIF 网格化处理异常: " << e.what();
        }
    }).detach();
}