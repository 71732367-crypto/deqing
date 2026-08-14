#include "models/TIFF.h"
#include <drogon/drogon.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/GlobalBaseTile.h>
#include <dqg/Data.h>
#include "controller/api_airRoute_Astar.h"
#include <iostream>
#include <fstream>
#include <clocale>
#include "GridEvaluatorLib/GridEvaluator.h"
#include <cstdlib>
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

        
        std::cout << "baseTile范围: [" << projectBaseTile.west << ", " << projectBaseTile.south << "] 到 ["
                  << projectBaseTile.east << ", " << projectBaseTile.north << "]" << std::endl;
        std::cout << "baseTile高度范围: [" << projectBaseTile.bottom << ", " << projectBaseTile.top << "]" << std::endl;
        
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
                double size = (projectBaseTile.top - projectBaseTile.bottom) / std::pow(2.0, level);
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

    std::string weightConfigPath = "./weight.json";
    if (!api::airRoute::loadWeightConfig(weightConfigPath)) {
        if (!api::airRoute::loadWeightConfig("../weight.json")) {
            api::airRoute::loadWeightConfig("/app/weight.json");
        }
    }
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
    //  服务端启动时的“TIFF 高程文件加载与状态检查”
    // ==============================================================
    drogon::app().registerBeginningAdvice([]() {
         LOG_INFO << "==========================================";
         LOG_INFO << "正在初始化 TIFF 高程数据...";

         std::string tiffFilePath = "";

         // 1. 从 Drogon 的 custom_config 中读取文件路径
         try {
             const Json::Value& customConfig = drogon::app().getCustomConfig();
             if (customConfig.isMember("tiff_file_path")) {
                 tiffFilePath = customConfig["tiff_file_path"].asString();
                 LOG_INFO << "已从 config.json 提取 TIFF 路径: " << tiffFilePath;
             } else {
                 // 如果 JSON 里忘写了，给个默认后备路径
                 tiffFilePath = "./data/dem.tif";
                 LOG_WARN << "config.json 中未配置 tiff_file_path，使用默认路径: " << tiffFilePath;
             }
         } catch (const std::exception& e) {
             tiffFilePath = "./data/dem.tif";
             LOG_ERROR << "解析 custom_config 异常: " << e.what() << "，退回默认路径: " << tiffFilePath;
         }

         // 2. 传递给 TiffReader 进行初始化
         bool isTiffLoaded = TiffReader::getInstance().init(tiffFilePath);

         if (!isTiffLoaded) {
             LOG_WARN << "------------------------------------------";
             LOG_WARN << "警告: TIFF 高程文件加载失败或未找到！路径: " << tiffFilePath;
             LOG_WARN << "警告: 当前系统处于【未使用 TIFF 文件获取真高】状态！";
             LOG_WARN << "注意: 航线规划中的防撞地与 120m 适飞区绝对真高校验将被降级处理。";
             LOG_WARN << "------------------------------------------";
         } else {
             LOG_INFO << "TIFF 高程文件加载成功！真高防撞系统已激活。";
         }
         LOG_INFO << "==========================================";
     });
    // ==============================================================
    //  服务端启动时的“Redis 纯连接探活检查
    // ==============================================================
    drogon::app().registerBeginningAdvice([]() {
          LOG_INFO << "==========================================";
          LOG_INFO << "正在检查 Redis 连接状态...";

          auto redisClient = drogon::app().getRedisClient();
          if (!redisClient) {
              LOG_FATAL << "错误：config.json 中未配置 Redis 客户端！";
              exit(1);
          }

          auto hasResponded = std::make_shared<bool>(false);

          // 1. 发送 PING 命令
          redisClient->execCommandAsync(
              [hasResponded](const drogon::nosql::RedisResult &r) {
                  *hasResponded = true;
                  LOG_INFO << "Redis 真实连接成功！(服务器响应: " << r.asString() << ")";
                  LOG_INFO << "==========================================";
              },
              [hasResponded](const std::exception &err) {
                  *hasResponded = true;
                  LOG_ERROR << "错误：Redis 连接失败！底层报错: " << err.what();
              },
              "PING"
          );

          // 2. 超时看门狗（精简版）
          drogon::app().getLoop()->runAfter(3.0, [hasResponded]() {
              if (!(*hasResponded)) {
                  LOG_ERROR << "错误：Redis 连接超时 ";
                  LOG_INFO << "==========================================";
              }
          });
      });
    //Set HTTP listener address and port
    // Note: The port in config.json will be used, this is just a fallback
    drogon::app().addListener("0.0.0.0", 9997);

    //Run HTTP framework,the method will block in the internal event loop
    drogon::app().run();
    return 0;
}