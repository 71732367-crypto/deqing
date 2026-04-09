#pragma once

#include <vector>
#include <string>
#include <array>
#include <dqg/DQG3DBasic.h>

/**
 * 线网格化主入口函数 (基于 3D DDA 算法)
 * @param lineReq - 折线坐标数组，元素为 {lon, lat, height}
 * @param level - 网格层级
 * @param baseTile - 基础网格配置
 * @returns 顺序排列且去重后的网格编码数组
 */
std::vector<std::string> singleLineToGrids2(const std::vector<std::array<double, 3>>& lineReq, int level, const BaseTile& baseTile);