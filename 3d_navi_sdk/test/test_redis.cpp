//
// Created by CUMTB on 2025/12/13.
//
#include <drogon/drogon.h>
#include <drogon/nosql/RedisClient.h>
#include <drogon/nosql/RedisResult.h>
#include <trantor/net/InetAddress.h>
#include <iostream>
#include <string>

using namespace drogon::nosql;
using namespace std;

int main() {
    trantor::InetAddress addr("127.0.0.1", 6379);
    auto redisClient = RedisClient::newRedisClient(
        addr,
        1,                   // 连接池大小
        "",                  // 密码（无则空字符串）
        0,                   // 数据库编号
        "test_client"        // 客户端名称
    );

    // 异步执行PING命令
    redisClient->execCommandAsync(
        [](const RedisResult &result) {
            cout << "Redis PING 结果：" << result.asString() << endl;
        },
        [](const std::exception &err) {
            cerr << "Redis 命令异常：" << err.what() << endl;
        },
        std::string_view("PING")
    );

    // 启动Drogon事件循环（异步操作依赖）
    drogon::app().run();

    return 0;
}
