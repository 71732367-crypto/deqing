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
    
    //Set HTTP listener address and port
    // Note: The port in config.json will be used, this is just a fallback
    drogon::app().addListener("0.0.0.0", 9997);
    
    //Run HTTP framework,the method will block in the internal event loop
    drogon::app().run();
    return 0;
}
