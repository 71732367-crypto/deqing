#pragma once

#include <dqg/Data.h>
#include <string>

/// @brief 全局基础瓦片变量声明
extern BaseTile projectBaseTile;

/// @brief 初始化全局基础瓦片数据
/// @param A 西南角坐标点
/// @param B 西北角坐标点  
/// @param C 东北角坐标点
/// @param D 东南角坐标点
void initializeProjectBaseTile(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D);

/// @brief 从JSON配置文件初始化全局基础瓦片数据
/// @param configFilePath JSON配置文件路径
/// @return 是否成功初始化
bool initializeProjectBaseTileFromConfig(const std::string& configFilePath);

/// @brief 获取全局基础瓦片数据
/// @return 基础瓦片常量引用
const BaseTile& getProjectBaseTile();
/// @brief 获取当前研究区域的互操作标识（基于 projectBaseTile 计算）
/// @return 区域标识字符串（16位十六进制）
std::string getProjectRegionId();