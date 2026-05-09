#include <drogon/drogon.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/GlobalBaseTile.h>
#include <dqg/Data.h>
#include "controller/api_airRoute_Astar.h"
#include <iostream>
#include <fstream>
#include <clocale>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
void configureConsoleEncoding() {
#ifdef _WIN32
    // Ensure narrow UTF-8 logs render correctly on Windows consoles.
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    std::setlocale(LC_ALL, ".UTF-8");
#endif
}
}


int main() {
    configureConsoleEncoding();
    try {
        // 从JSON配置文件初始化全局基础瓦片数据
        std::string configFilePath = "./region.json";
        
        // 检查配置文件是否存在
        std::ifstream configFile(configFilePath);
        if (!configFile.is_open()) {
            // 如果默认路径不存在，尝试当前目录
            configFilePath = "./region.json";
            configFile.open(configFilePath);
        }

        if (!configFile.is_open()) {
            // 如果默认路径不存在，尝试当前目录
            configFilePath = "../region.json";
            configFile.open(configFilePath);
        }

        
        if (!configFile.is_open()) {
            throw std::runtime_error("无法找到配置文件 region.json，请确保文件存在于 /app/region.json 或 ./region.json");
        }
        configFile.close();
        
        // 使用配置文件初始化基础网格数据
        if (!initializeProjectBaseTileFromConfig(configFilePath)) {
            throw std::runtime_error("从配置文件初始化基础网格数据失败");
        }

        
        std::cout << "basetiel范围: [" << projectBaseTile.west << ", " << projectBaseTile.south << "] 到 ["
                  << projectBaseTile.east << ", " << projectBaseTile.north << "]" << std::endl;
        std::cout << "basetile高度范围: [" << projectBaseTile.bottom << ", " << projectBaseTile.top << "]" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "初始化全局基础瓦片失败: " << e.what() << std::endl;
        return -1;
    }

    // 网格尺度信息打印
    std::cout << "网格尺度信息: " << std::endl;
    // 剖分层级对应的网格尺寸关系
    const int baseLevel = 2;  // 基础层级
    std::cout << "剖分层级对应的网格尺寸: " << std::endl;
    for (int level = 1; level < 32 - baseLevel; level++) {
                double size = 78125.0 / std::pow(2.0, level);
        std::cout << "  层级 " << level << " -> 尺寸 ≈ " << size << "m" << std::endl;
    }

    // 从JSON配置文件初始化Drogon配置
    std::string drogonConfigPath = "../config.json";  // 默认假设从cmake-build运行
    std::ifstream drogonConfigFile(drogonConfigPath);
    if (!drogonConfigFile.is_open()) {
        // 尝试当前目录
        drogonConfigPath = "./config.json";
        drogonConfigFile.open(drogonConfigPath);
        if (!drogonConfigFile.is_open()) {
            // 尝试其他可能的路径
            drogonConfigPath = "/app/config.json";
            drogonConfigFile.open(drogonConfigPath);
        }
    }
    
    if (!drogonConfigFile.is_open()) {
        std::cerr << "尝试的配置文件路径:" << std::endl;
        std::cerr << "1. ../config.json" << std::endl;
        std::cerr << "2. ./config.json" << std::endl;
        std::cerr << "3. /app/config.json" << std::endl;
        throw std::runtime_error("无法找到Drogon配置文件 config.json");
    }
    drogonConfigFile.close();
    
    std::cout << "使用配置文件: " << drogonConfigPath << std::endl;
    drogon::app().loadConfigFile(drogonConfigPath);

    
    // 初始化 A* 算法配置（从 config.json 加载）
    api::airRoute::initializeAstarConfig();
    
    // ==========================================
    // 全局 CORS 跨域配置开始
    // ==========================================

    // 1. 拦截所有的 OPTIONS 预检请求，直接返回 200 并携带跨域头
    drogon::app().registerPreRoutingAdvice([](const drogon::HttpRequestPtr &req,
                                              drogon::FilterCallback &&defer,
                                              drogon::FilterChainCallback &&deferNext) {
        if (req->method() == drogon::HttpMethod::Options) {
            auto res = drogon::HttpResponse::newHttpResponse();
            res->addHeader("Access-Control-Allow-Origin", "*");
            res->addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
            res->addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With");
            res->setStatusCode(drogon::k200OK);
            defer(res);
            return;
        }
        deferNext(); // 不是 OPTIONS 请求，放行给后续路由
    });

    // 2. 在所有请求处理完成后，为正常的响应（如 200 的 POST 请求）补充跨域头
    drogon::app().registerPostHandlingAdvice([](const drogon::HttpRequestPtr &req,
                                                const drogon::HttpResponsePtr &res) {
        // 如果想更严谨，可以根据前端的 Origin 动态设置，这里简单粗暴允许所有 (*)
        res->addHeader("Access-Control-Allow-Origin", "*");
    });


// ==============================================================
    // ✨ 新增：服务端启动时的“全量规则深度穿透测试”
    // 一次性扫描 hl, gd, ad, wdd, fxq 等所有规则类别
    // ==============================================================
    drogon::app().registerBeginningAdvice([]() {
        LOG_INFO << "==========================================";
        LOG_INFO << "正在执行 Redis 全量规则字段深度测试...";

        auto redisClient = drogon::app().getRedisClient();
        if (!redisClient) {
            LOG_FATAL << "致命错误：Redis 客户端未配置！";
            exit(1);
        }

        // 我们指定一个网格编码进行测试 (请确保 Redis 里有这个网格的数据)
        std::string testGrid = "30031122216";

        // ---------------------------------------------------------
        // 1. 测试所有的 String 类型 (红线约束 & 离散代价)
        // ---------------------------------------------------------
        std::vector<std::string> stringPrefixes = {
            "hl", "hlz", "fx", "gd", "dt", "dz", "za", "dc", "tx", "dh", "jk"
        };
        for (const auto& p : stringPrefixes) {
            // hlz 规则在底层实际查询的是 hl
            std::string queryPrefix = (p == "hlz") ? "hl" : p;
            std::string key = queryPrefix + "_" + testGrid;

            redisClient->execCommandAsync(
                [key, p](const drogon::nosql::RedisResult &r) {
                    if (!r.isNil()) {
                        LOG_INFO << "√[String测试] 成功读取 [" << p << "] -> 示例值: " << r.asString();
                    } else {
                        LOG_WARN << "X[String测试] 缺失 [" << p << "] 数据 (Key: " << key << ")";
                    }
                },
                [](const std::exception &err) {}, "GET %s", key.c_str()
            );
        }

        // ---------------------------------------------------------
           // 2. 测试所有的 Set 类型 (空域检查)
           // ---------------------------------------------------------
           std::string setKey = "ad_" + testGrid;
           redisClient->execCommandAsync(
               [setKey](const drogon::nosql::RedisResult &r) {
                   // ✨ 增加拦截：不仅要是数组，而且数组不能为空！
                   if (!r.isNil() && r.type() == drogon::nosql::RedisResultType::kArray && !r.asArray().empty()) {
                       LOG_INFO << "√[Set测试] 成功读取空域 [ad] -> 包含 " << r.asArray().size() << " 个元素";
                   } else {
                       LOG_WARN << "X[Set测试] 缺失空域 [ad] 数据 (Key: " << setKey << ")";
                   }
               },
               [](const std::exception &err) {}, "SMEMBERS %s", setKey.c_str()
           );

           // ---------------------------------------------------------
           // 3. 测试所有的 Hash 类型 (气象数据、隐私区)
           // ---------------------------------------------------------
           std::vector<std::string> hashPrefixes = {"wdd", "wdh", "privacy"};
           for (const auto& p : hashPrefixes) {
               std::string key = p + "_" + testGrid;
               redisClient->execCommandAsync(
                   [key, p](const drogon::nosql::RedisResult &r) {
                       // ✨ 增加拦截：不仅要是数组，而且数组不能为空！(HGETALL 返回 key-value 交替的数组)
                       if (!r.isNil() && r.type() == drogon::nosql::RedisResultType::kArray && !r.asArray().empty()) {
                           LOG_INFO << "√[Hash测试] 成功读取字典 [" << p << "] -> 获取到完整 Hash 结构，包含 "
                                    << r.asArray().size() / 2 << " 个键值对";
                       } else {
                           LOG_WARN << "X[Hash测试] 缺失字典 [" << p << "] 数据 (Key: " << key << ")";
                       }
                   },
                   [](const std::exception &err) {}, "HGETALL %s", key.c_str()
               );
           }

        // ---------------------------------------------------------
        // 4. 测试 JSON-String 类型 (离散风险评估 fxq)
        // ---------------------------------------------------------
        std::string fxqKey = "fxq_" + testGrid;
        redisClient->execCommandAsync(
            [fxqKey](const drogon::nosql::RedisResult &r) {
                if (r.isNil()) {
                    LOG_WARN << "X[JSON测试] 缺失 [fxq] 风险区数据 (Key: " << fxqKey << ")";
                    return;
                }

                std::string jsonStr = r.asString();
                Json::Value jVal;
                Json::CharReaderBuilder builder;
                std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                std::string errs;

                if (reader->parse(jsonStr.c_str(), jsonStr.c_str() + jsonStr.length(), &jVal, &errs)) {
                    LOG_INFO << "√[JSON测试] 成功解析 [fxq] 字典！开始提取所有风险时间字段:";

                    std::vector<std::string> fxqFields = {
                        "workday_low_risk_time", "workday_mid_risk_time", "workday_high_risk_time",
                        "weekend_low_risk_time", "weekend_mid_risk_time", "weekend_high_risk_time",
                        "holiday_low_risk_time", "holiday_mid_risk_time", "holiday_high_risk_time"
                    };

                    for(const auto& f : fxqFields) {
                        if (jVal.isMember(f)) {
                            LOG_INFO << "   √成功提取 [" << f << "] -> " << jVal[f].asString();
                        } else {
                            LOG_WARN << "   X缺失字段 [" << f << "]";
                        }
                    }
                } else {
                    LOG_ERROR << "X [JSON测试] 解析 [fxq] 失败！底层原因: " << errs;
                }
                LOG_INFO << "==========================================";
            },
            [](const std::exception &err) {}, "GET %s", fxqKey.c_str()
        );
    });
    // ==============================================================
    //Set HTTP listener address and port
    // Note: The port in config.json will be used, this is just a fallback
    drogon::app().addListener("0.0.0.0", 9997);

    //Run HTTP framework,the method will block in the internal event loop
    drogon::app().run();
    return 0;
}