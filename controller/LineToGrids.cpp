#include "LineToGrids.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

// 核心算法：3D DDA (Amanatides & Woo)
// 沿着线段方向寻找所有相交网格，保证面邻接且有序
static std::vector<std::string> lineInsertPointsDDA(const std::array<double, 3>& p1, const std::array<double, 3>& p2, uint8_t level, const BaseTile& baseTile) {
    // p1[0]: lon, p1[1]: lat, p1[2]: height
    std::string startCode = getLocalCode(level, p1[0], p1[1], p1[2], baseTile);
    std::string endCode = getLocalCode(level, p2[0], p2[1], p2[2], baseTile);

    if (startCode == endCode) {
        return {startCode};
    }

    LatLonHei startGrid = getLocalTileLatLon(startCode, baseTile);

    // 计算网格单位尺寸 (在此局部区域假设网格大小一致)
    double dLonSize = std::abs(startGrid.east - startGrid.west);
    double dLatSize = std::abs(startGrid.north - startGrid.south);
    double dHeightSize = std::abs(startGrid.top - startGrid.bottom);

    // 1. 初始化步进方向 (Step)
    int stepLon = p2[0] > p1[0] ? 1 : -1;
    int stepLat = p2[1] > p1[1] ? 1 : -1;
    int stepH = p2[2] > p1[2] ? 1 : -1;

    // 辅助函数：处理被除数为 0 的情况，防止崩溃 (使用 1e-15)
    auto safeDivisor = [](double val) {
        return std::abs(val) > 1e-15 ? std::abs(val) : 1e-15;
    };
    auto safeDivisorSigned = [](double val) {
        return std::abs(val) > 1e-15 ? val : (val >= 0 ? 1e-15 : -1e-15);
    };

    // 2. 计算跨越一个完整网格所需的参数 t 的增量 (tDelta)
    double tDeltaLon = dLonSize / safeDivisor(p2[0] - p1[0]);
    double tDeltaLat = dLatSize / safeDivisor(p2[1] - p1[1]);
    double tDeltaH = dHeightSize / safeDivisor(p2[2] - p1[2]);

    // 3. 计算到达下一个网格边界所需的初始 t 值 (tMax)
    double tMaxLon, tMaxLat, tMaxH;

    if (stepLon > 0) {
        tMaxLon = (startGrid.east - p1[0]) / safeDivisorSigned(p2[0] - p1[0]);
    } else {
        tMaxLon = (p1[0] - startGrid.west) / safeDivisorSigned(p1[0] - p2[0]);
    }

    if (stepLat > 0) {
        tMaxLat = (startGrid.north - p1[1]) / safeDivisorSigned(p2[1] - p1[1]);
    } else {
        tMaxLat = (p1[1] - startGrid.south) / safeDivisorSigned(p1[1] - p2[1]);
    }

    if (stepH > 0) {
        tMaxH = (startGrid.top - p1[2]) / safeDivisorSigned(p2[2] - p1[2]);
    } else {
        tMaxH = (p1[2] - startGrid.bottom) / safeDivisorSigned(p1[2] - p2[2]);
    }

    std::vector<std::string> codes;
    codes.push_back(startCode);
    std::string currentCode = startCode;

    // 使用当前点位置进行迭代
    double currLon = p1[0];
    double currLat = p1[1];
    double currH = p1[2];

    // 安全计数器，防止极端情况死循环
    int safetyNet = 1000;

    while (currentCode != endCode && safetyNet > 0) {
        safetyNet--;

        // 寻找最近的边界 (比较哪个方向的 t 值最小)
        if (tMaxLon < tMaxLat) {
            if (tMaxLon < tMaxH) {
                // 跨越经度边界
                currLon += stepLon * dLonSize;
                tMaxLon += tDeltaLon;
            } else {
                // 跨越高度边界
                currH += stepH * dHeightSize;
                tMaxH += tDeltaH;
            }
        } else {
            if (tMaxLat < tMaxH) {
                // 跨越纬度边界
                currLat += stepLat * dLatSize;
                tMaxLat += tDeltaLat;
            } else {
                // 跨越高度边界
                currH += stepH * dHeightSize;
                tMaxH += tDeltaH;
            }
        }

        // 获取新位置的编码
        currentCode = getLocalCode(level, currLon, currLat, currH, baseTile);

        // 避免因为浮点数精度在边界点反复计算同一个网格
        if (currentCode != codes.back()) {
            codes.push_back(currentCode);
        }
    }

    // 确保终点编码被包含 (补偿 DDA 在极点处的精度丢失)
    if (codes.back() != endCode) {
        codes.push_back(endCode);
    }

    return codes;
}

// 供控制器调用的主函数实现
std::vector<std::string> singleLineToGrids2(const std::vector<std::array<double, 3>>& lineReq, int level, const BaseTile& baseTile) {
    if (lineReq.size() < 2) {
        throw std::invalid_argument("输入必须包含至少两个点的数组");
    }

    std::vector<std::string> allCodes;
    uint8_t uLevel = static_cast<uint8_t>(level); // 适配 dqg 库中要求的 level 类型

    for (size_t i = 0; i < lineReq.size() - 1; ++i) {
        const auto& p1 = lineReq[i];
        const auto& p2 = lineReq[i + 1];

        std::vector<std::string> segmentCodes = lineInsertPointsDDA(p1, p2, uLevel, baseTile);

        if (allCodes.empty()) {
            allCodes = segmentCodes;
        } else {
            // 衔接处去重：如果上一段的终点和这一段的起点相同，则跳过重复点
            const std::string& lastCode = allCodes.back();
            size_t startIdx = (lastCode == segmentCodes[0]) ? 1 : 0;
            for (size_t j = startIdx; j < segmentCodes.size(); ++j) {
                allCodes.push_back(segmentCodes[j]);
            }
        }
    }

    return allCodes;
}