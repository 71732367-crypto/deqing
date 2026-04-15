/*
Copyright (c) [2025] [AnYiZong]
All rights reserved.
No part of this code may be copied or distributed in any form without permission.
*/


#include <dqg/DQG3DTil.h>
#include<sstream>
#include <stdexcept>
#include <cstdint>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <string>

const double EPSILON = 1e-10;




//JS版本的2D和3D直线算法（有问题，线包不全）
std::vector<IJ> bresenham2D(const IJ& p0, const IJ& p1) {
	std::vector<IJ> points;

	int64_t dx = std::abs(static_cast<int64_t>(p1.column) - static_cast<int64_t>(p0.column));
	int64_t dy = std::abs(static_cast<int64_t>(p1.row) - static_cast<int64_t>(p0.row));
	int64_t sx = (p0.column < p1.column) ? 1 : -1;
	int64_t sy = (p0.row < p1.row) ? 1 : -1;
	int64_t err = dx - dy;

	int64_t x = static_cast<int64_t>(p0.column);
	int64_t y = static_cast<int64_t>(p0.row);

	while (true) {
		if (x < 0 || y < 0 || x > std::numeric_limits<uint32_t>::max() || y > std::numeric_limits<uint32_t>::max()) {
			throw std::range_error("bresenham2D: coordinate overflow");
		}
		points.push_back({ static_cast<uint32_t>(y), static_cast<uint32_t>(x) });
		if (x == static_cast<int64_t>(p1.column) && y == static_cast<int64_t>(p1.row)) break;

		int64_t e2 = 2 * err;
		if (e2 > -dy) { err -= dy; x += sx; }
		if (e2 < dx) { err += dx; y += sy; }
	}

	return points;
}

/**
 * @brief 三维Bresenham算法实现
 *
 * 使用Bresenham算法计算三维空间中两点之间的网格路径。该算法通过增量计算
 * 避免浮点数运算，高效地生成连接两点的所有网格点，适用于三维空间中的直线绘制、
 * 路径规划等场景。
 *
 * @param point1 起点，包含行(row)、列(column)和层(layer)三个维度的坐标
 * @param point2 终点，包含行(row)、列(column)和层(layer)三个维度的坐标
 * @return std::vector<IJH> 包含路径上所有网格点的向量
 */
std::vector<IJH> bresenham3D(const IJH& point1, const IJH& point2) {
    // 结果容器，存储路径上的所有网格点
    std::vector<IJH> result;

    // 定义坐标的最大值常量，用于边界检查
    constexpr uint64_t MAX_COORD = std::numeric_limits<uint32_t>::max();

    /**
     * @brief 坐标验证函数
     *
     * 验证坐标值是否在有效范围内，防止溢出
     *
     * @param val 待验证的坐标值
     * @throw std::range_error 如果坐标值超出有效范围
     */
    auto validate = [MAX_COORD](int64_t val) {
        if (val < 0 || static_cast<uint64_t>(val) > MAX_COORD)
            throw std::range_error("Coordinate overflow in bresenham3D");
    };

    // 计算各维度的坐标差值（终点减去起点）
    int64_t dx = static_cast<int64_t>(point2.column) - point1.column; // 列方向差值
    int64_t dy = static_cast<int64_t>(point2.row) - point1.row;      // 行方向差值
    int64_t dz = static_cast<int64_t>(point2.layer) - point1.layer;  // 层方向差值

    // 计算各维度差值的绝对值
    int64_t abs_dx = std::abs(dx);  // 列方向差值绝对值
    int64_t abs_dy = std::abs(dy);  // 行方向差值绝对值
    int64_t abs_dz = std::abs(dz);  // 层方向差值绝对值

    // 确定主导方向：选择差值最大的方向作为步进的主导方向
    int64_t steps = std::max({ abs_dx, abs_dy, abs_dz });

    // 特殊情况处理：如果起点和终点是同一个点
    if (steps == 0) {
        result.push_back(point1); // 直接添加该点
        return result;
    }

    // 确定主导轴方向标志
    bool z_dominant = (abs_dz == steps);                     // Z轴（层）是否为主导方向
    bool y_dominant = (abs_dy == steps) && !z_dominant;      // Y轴（行）是否为主导方向
    bool x_dominant = (abs_dx == steps) && !y_dominant && !z_dominant; // X轴（列）是否为主导方向

    // 确定各维度的步进方向（正向或负向）
    int64_t sign_x = (dx > 0) ? 1 : -1;  // 列方向步进符号
    int64_t sign_y = (dy > 0) ? 1 : -1;  // 行方向步进符号
    int64_t sign_z = (dz > 0) ? 1 : -1;  // 层方向步进符号

    // 初始化当前位置为起点
    int64_t x = point1.column;  // 当前列坐标
    int64_t y = point1.row;     // 当前行坐标
    int64_t z = point1.layer;   // 当前层坐标

    // 初始化误差项
    int64_t err_xy = 0, err_z = 0;

    // 根据主导方向设置初始误差值
    // 误差计算基于2倍差值以避免浮点运算
    if (x_dominant) {
        // X轴主导时，计算Y和Z相对于X的误差初始值
        err_xy = 2 * abs_dy - abs_dx;
        err_z = 2 * abs_dz - abs_dx;
    }
    else if (y_dominant) {
        // Y轴主导时，计算X和Z相对于Y的误差初始值
        err_xy = 2 * abs_dx - abs_dy;
        err_z = 2 * abs_dz - abs_dy;
    }
    else if (z_dominant) {
        // Z轴主导时，计算X和Y相对于Z的误差初始值
        err_xy = 2 * abs_dx - abs_dz;
        err_z = 2 * abs_dy - abs_dz;
    }

    // 执行Bresenham算法的主循环
    // 按照主导方向的步数进行迭代
    for (int64_t i = 0; i <= steps; ++i) {
        // 验证当前坐标是否有效
        validate(x); validate(y); validate(z);

        // 将当前点添加到结果中
        // 注意：IJH结构中的参数顺序是(row, column, layer)
        result.push_back({
            static_cast<uint32_t>(y),   // 行坐标
            static_cast<uint32_t>(x),   // 列坐标
            static_cast<uint32_t>(z)    // 层坐标
        });

        // 如果已到达终点，提前退出循环
        if (x == point2.column && y == point2.row && z == point2.layer)
            break;

        // 根据主导方向更新坐标和误差
        if (x_dominant) {
            // X轴主导：X坐标每次步进1
            x += sign_x;

            // 检查Y坐标是否需要步进
            if (err_xy >= 0) {
                y += sign_y;
                err_xy -= 2 * abs_dx;  // 重置误差
            }

            // 检查Z坐标是否需要步进
            if (err_z >= 0) {
                z += sign_z;
                err_z -= 2 * abs_dx;   // 重置误差
            }

            // 更新误差累加值
            err_xy += 2 * abs_dy;
            err_z += 2 * abs_dz;
        }
        else if (y_dominant) {
            // Y轴主导：Y坐标每次步进1
            y += sign_y;

            // 检查X坐标是否需要步进
            if (err_xy >= 0) {
                x += sign_x;
                err_xy -= 2 * abs_dy;  // 重置误差
            }

            // 检查Z坐标是否需要步进
            if (err_z >= 0) {
                z += sign_z;
                err_z -= 2 * abs_dy;   // 重置误差
            }

            // 更新误差累加值
            err_xy += 2 * abs_dx;
            err_z += 2 * abs_dz;
        }
        else if (z_dominant) {
            // Z轴主导：Z坐标每次步进1
            z += sign_z;

            // 检查X坐标是否需要步进
            if (err_xy >= 0) {
                x += sign_x;
                err_xy -= 2 * abs_dz;  // 重置误差
            }

            // 检查Y坐标是否需要步进
            if (err_z >= 0) {
                y += sign_y;
                err_z -= 2 * abs_dz;   // 重置误差
            }

            // 更新误差累加值
            err_xy += 2 * abs_dx;
            err_z += 2 * abs_dy;
        }
    }

    // 返回生成的路径点列表
    return result;
}

///*************************************以下为三维直线生成局部网格******************************************************
/**
	以下为三维直线生成局部网格
	*输入：三维线上点坐标序列（经、纬、高）、层级、basetile
	*输出：三维线网格化后的局部网格序列
**/


// ================== 辅助计算函数 ==================

/**
 * @brief 计算点 Q 到线段 AB 的投影参数 t (0.0 ~ 1.0)
 * * @param A 起点
 * @param B 终点
 * @param Q 待投影的点（通常是网格中心）
 * @return double t值，0表示在A点，1表示在B点，按路径走向增加
 */
double projectPointToSegmentT(const PointLBHd& A, const PointLBHd& B, const PointLBHd& Q) {
    double ABx = B.Lng - A.Lng;
    double ABy = B.Lat - A.Lat;
    double ABz = B.Hgt - A.Hgt;

    double AQx = Q.Lng - A.Lng;
    double AQy = Q.Lat - A.Lat;
    double AQz = Q.Hgt - A.Hgt;

    double abSquared = ABx * ABx + ABy * ABy + ABz * ABz;

    // 防止除以零（起点终点重合）
    if (abSquared < 1e-20) {
        return 0.0;
    }

    // 向量点积计算投影
    double t = (AQx * ABx + AQy * ABy + AQz * ABz) / abSquared;

    // 约束在 [0, 1] 范围内
    return std::max(0.0, std::min(1.0, t));
}

/**
 * @brief 计算直线和平面交点（保持原逻辑）
 */
PointLBHd calculateIntersectionForLine(const PointLBHd& A, const PointLBHd& B, double planeValue, const std::string& planeType) {
    double t;
    const double EPSILON = 1e-10;

    if (planeType == "lon") {
        if (std::abs(B.Lng - A.Lng) < EPSILON) return {0, 0, 0};
        t = (planeValue - A.Lng) / (B.Lng - A.Lng);
    }
    else if (planeType == "lat") {
        if (std::abs(B.Lat - A.Lat) < EPSILON) return {0, 0, 0};
        t = (planeValue - A.Lat) / (B.Lat - A.Lat);
    }
    else if (planeType == "height") {
        if (std::abs(B.Hgt - A.Hgt) < EPSILON) return {0, 0, 0};
        t = (planeValue - A.Hgt) / (B.Hgt - A.Hgt);
    }
    else {
        return {0, 0, 0};
    }

    if (t < 0 || t > 1) return {0, 0, 0};

    return {
        A.Lng + t * (B.Lng - A.Lng),
        A.Lat + t * (B.Lat - A.Lat),
        A.Hgt + t * (B.Hgt - A.Hgt)
    };
}

struct GridBoundaries {
    std::vector<double> longitudes;
    std::vector<double> latitudes;
    std::vector<double> heights;
};

/**
 * @brief 获取线段穿过的所有网格边界值（保持原逻辑）
 */
GridBoundaries getGridBoundaries(const LatLonHei& startGrid, const PointLBHd& p1, const PointLBHd& p2) {
    const double diffLon = startGrid.east - startGrid.west;
    const double diffLat = startGrid.north - startGrid.south;
    const double diffHeight = startGrid.top - startGrid.bottom;
    const double EPSILON = 1e-10;

    GridBoundaries boundaries;

    // 经度边界
    const double lonStart = std::min(p1.Lng, p2.Lng);
    const double lonEnd = std::max(p1.Lng, p2.Lng);
    double currentLon = p1.Lng > p2.Lng ?
        std::floor((p1.Lng - startGrid.west) / diffLon) * diffLon + startGrid.west :
        std::ceil((p1.Lng - startGrid.west) / diffLon) * diffLon + startGrid.west;
    const double lonStep = p1.Lng < p2.Lng ? diffLon : -diffLon;

    while (p1.Lng < p2.Lng ? currentLon <= lonEnd : currentLon >= lonStart) {
        if (std::abs(currentLon - p1.Lng) > EPSILON && std::abs(currentLon - p2.Lng) > EPSILON) {
            boundaries.longitudes.push_back(currentLon);
        }
        currentLon += lonStep;
    }

    // 纬度边界
    const double latStart = std::min(p1.Lat, p2.Lat);
    const double latEnd = std::max(p1.Lat, p2.Lat);
    double currentLat = p1.Lat > p2.Lat ?
        std::floor((p1.Lat - startGrid.south) / diffLat) * diffLat + startGrid.south :
        std::ceil((p1.Lat - startGrid.south) / diffLat) * diffLat + startGrid.south;
    const double latStep = p1.Lat < p2.Lat ? diffLat : -diffLat;

    while (p1.Lat < p2.Lat ? currentLat <= latEnd : currentLat >= latStart) {
        if (std::abs(currentLat - p1.Lat) > EPSILON && std::abs(currentLat - p2.Lat) > EPSILON) {
            boundaries.latitudes.push_back(currentLat);
        }
        currentLat += latStep;
    }

    // 高度边界
    const double heightStart = std::min(p1.Hgt, p2.Hgt);
    const double heightEnd = std::max(p1.Hgt, p2.Hgt);
    double currentHeight = p1.Hgt > p2.Hgt ?
        std::floor((p1.Hgt - startGrid.bottom) / diffHeight) * diffHeight + startGrid.bottom :
        std::ceil((p1.Hgt - startGrid.bottom) / diffHeight) * diffHeight + startGrid.bottom;
    const double heightStep = p1.Hgt < p2.Hgt ? diffHeight : -diffHeight;

    while (p1.Hgt < p2.Hgt ? currentHeight <= heightEnd : currentHeight >= heightStart) {
        if (std::abs(currentHeight - p1.Hgt) > EPSILON && std::abs(currentHeight - p2.Hgt) > EPSILON) {
            boundaries.heights.push_back(currentHeight);
        }
        currentHeight += heightStep;
    }

    return boundaries;
}

// ================== 核心网格化逻辑 ==================

/**
 * @brief 计算单条线段穿过的网格，并按路径顺序排序（核心修改函数）
 * * 逻辑：
 * 1. 找出所有候选网格（包含容差范围内的）放入 Set 去重。
 * 2. 计算每个网格中心点。
 * 3. 计算中心点在直线上的投影参数 t。
 * 4. 按 t 排序返回。
 */
std::vector<std::string> lineInsertPoints(const PointLBHd& p1, const PointLBHd& p2, uint8_t level, const BaseTile& baseTile) {
    const std::string startCode = getLocalCode(level, p1.Lng, p1.Lat, p1.Hgt, baseTile);
    const std::string endCode = getLocalCode(level, p2.Lng, p2.Lat, p2.Hgt, baseTile);

    // 如果起点和终点在同一个网格内，直接返回
    if (startCode == endCode) {
        return {startCode};
    }

    const LatLonHei startGrid = getLocalTileLatLon(startCode, baseTile);
    const GridBoundaries boundaries = getGridBoundaries(startGrid, p1, p2);

    // 使用 Set 收集去重后的候选网格
    std::unordered_set<std::string> candidateCodes;

    candidateCodes.insert(startCode);
    candidateCodes.insert(endCode);

    // 辅助lambda：添加交点及偏移点
    auto addIntersectionWithOffsets = [&](const PointLBHd& intersection, const std::string& type) {
        candidateCodes.insert(getLocalCode(level, intersection.Lng, intersection.Lat, intersection.Hgt, baseTile));

        if (type == "lon") {
            const double diffLon = startGrid.east - startGrid.west;
            const double ratio = diffLon / 10.0;
            candidateCodes.insert(getLocalCode(level, intersection.Lng - ratio, intersection.Lat, intersection.Hgt, baseTile));
            candidateCodes.insert(getLocalCode(level, intersection.Lng + ratio, intersection.Lat, intersection.Hgt, baseTile));
        } else if (type == "lat") {
            const double diffLat = startGrid.north - startGrid.south;
            const double ratio = diffLat / 10.0;
            candidateCodes.insert(getLocalCode(level, intersection.Lng, intersection.Lat - ratio, intersection.Hgt, baseTile));
            candidateCodes.insert(getLocalCode(level, intersection.Lng, intersection.Lat + ratio, intersection.Hgt, baseTile));
        } else if (type == "height") {
            const double diffHeight = startGrid.top - startGrid.bottom;
            const double ratio = diffHeight / 10.0;
            candidateCodes.insert(getLocalCode(level, intersection.Lng, intersection.Lat, intersection.Hgt - ratio, baseTile));
            candidateCodes.insert(getLocalCode(level, intersection.Lng, intersection.Lat, intersection.Hgt + ratio, baseTile));
        }
    };

    for (double lon : boundaries.longitudes) {
        PointLBHd inter = calculateIntersectionForLine(p1, p2, lon, "lon");
        if (inter.Lng != 0 || inter.Lat != 0 || inter.Hgt != 0) addIntersectionWithOffsets(inter, "lon");
    }
    for (double lat : boundaries.latitudes) {
        PointLBHd inter = calculateIntersectionForLine(p1, p2, lat, "lat");
        if (inter.Lng != 0 || inter.Lat != 0 || inter.Hgt != 0) addIntersectionWithOffsets(inter, "lat");
    }
    for (double height : boundaries.heights) {
        PointLBHd inter = calculateIntersectionForLine(p1, p2, height, "height");
        if (inter.Lng != 0 || inter.Lat != 0 || inter.Hgt != 0) addIntersectionWithOffsets(inter, "height");
    }

    // --- 新增逻辑：排序阶段 ---

    // 临时结构用于排序
    struct CodeWithT {
        std::string code;
        double t;
    };
    std::vector<CodeWithT> sortedList;
    sortedList.reserve(candidateCodes.size());

    for (const auto& code : candidateCodes) {
        // 反算网格中心点
        LatLonHei tile = getLocalTileLatLon(code, baseTile);
        PointLBHd center;
        center.Lng = (tile.west + tile.east) / 2.0;
        center.Lat = (tile.south + tile.north) / 2.0;
        center.Hgt = (tile.bottom + tile.top) / 2.0;

        // 计算投影参数 t
        double t = projectPointToSegmentT(p1, p2, center);
        sortedList.push_back({code, t});
    }

    // 按 t 值升序排列
    std::sort(sortedList.begin(), sortedList.end(), [](const CodeWithT& a, const CodeWithT& b) {
        return a.t < b.t;
    });

    // 提取排序后的编码
    std::vector<std::string> result;
    result.reserve(sortedList.size());
    for (const auto& item : sortedList) {
        result.push_back(item.code);
    }

    return result;
}

/**
 * @brief 将折线转换为网格编码（核心修改函数）
 * * 逻辑：
 * 1. 遍历每一段线段。
 * 2. 获取该段的有序网格序列。
 * 3. 拼接到总结果中，处理“首尾相接”时的重复点。
 */
std::vector<std::string> lineToGrids(const std::vector<PointLBHd>& line, uint8_t level, const BaseTile& baseTile) {
    std::vector<std::string> allCodes;

    for (size_t i = 0; i < line.size() - 1; i++) {
        const PointLBHd& p1 = line[i];
        const PointLBHd& p2 = line[i + 1];

        // 获取单段的有序网格
        std::vector<std::string> segmentCodes = lineInsertPoints(p1, p2, level, baseTile);

        if (allCodes.empty()) {
            // 第一段直接完全添加
            allCodes = std::move(segmentCodes);
        } else {
            // 衔接去重：如果总结果的最后一个 == 新段的第一个，跳过新段第一个
            if (!segmentCodes.empty()) {
                if (allCodes.back() == segmentCodes.front()) {
                    allCodes.insert(allCodes.end(), segmentCodes.begin() + 1, segmentCodes.end());
                } else {
                    allCodes.insert(allCodes.end(), segmentCodes.begin(), segmentCodes.end());
                }
            }
        }
    }

    return allCodes;
}

/**
 * @brief （主函数）三维线网格化为局部网格
 */
std::vector<std::string> lineToLocalCode(const std::vector<PointLBHd>& lineReq, uint8_t level, const BaseTile& baseTile) {
    if (lineReq.size() < 2) {
        throw std::invalid_argument("输入必须包含至少两个点的数组");
    }

    return lineToGrids(lineReq, level, baseTile);
}



/**
 * @brief 根据一个经纬度、距离和方位角计算另一个经纬度
 * @param lng 起点经度（单位：度）
 * @param lat 起点纬度（单位：度）
 * @param brng 方位角（单位：度，正北为 0°，顺时针增加）
 * @param dist 距离（单位：米）
 * @return 返回目标点的经纬度
 */
PointLBd getLngLat(double lng, double lat, double brng, double dist) {
	const double a = WGS84a;       // ���뾶
	const double f = WGS84f;       // ����
	const double radBrng = brng * RegToRad; // ��λ��ת��Ϊ����
	const double sinAlpha1 = std::sin(radBrng);
	const double cosAlpha1 = std::cos(radBrng);
	const double tanU1 = (1 - f) * std::tan(lat * RegToRad);
	const double cosU1 = 1.0 / std::sqrt(1 + tanU1 * tanU1);
	const double sinU1 = tanU1 * cosU1;

	double sigma1 = std::atan2(tanU1, cosAlpha1);
	double sinAlpha = cosU1 * sinAlpha1;
	double cosSqAlpha = 1 - sinAlpha * sinAlpha;
	double uSq = cosSqAlpha * (a * a - WGS84b * WGS84b) / (WGS84b * WGS84b);
	double A = 1 + uSq / 16384.0 * (4096.0 + uSq * (-768.0 + uSq * (320.0 - 175.0 * uSq)));
	double B = uSq / 1024.0 * (256.0 + uSq * (-128.0 + uSq * (74.0 - 47.0 * uSq)));

	double sigma = dist / (WGS84b * A);
	double sigmaP;
	double sinSigma, cosSigma, cos2SigmaM, deltaSigma;

	do {
		cos2SigmaM = std::cos(2.0 * sigma1 + sigma);
		sinSigma = std::sin(sigma);
		cosSigma = std::cos(sigma);
		deltaSigma = B * sinSigma * (cos2SigmaM + B / 4.0 *
			(cosSigma * (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM) -
				B / 6.0 * cos2SigmaM * (-3.0 + 4.0 * sinSigma * sinSigma) *
				(-3.0 + 4.0 * cos2SigmaM * cos2SigmaM)));
		sigmaP = sigma;
		sigma = dist / (WGS84b * A) + deltaSigma;
	} while (std::fabs(sigma - sigmaP) > 1e-12);

	double tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
	double lat2 = std::atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1,
		(1.0 - f) * std::sqrt(sinAlpha * sinAlpha + tmp * tmp));
	double lambda = std::atan2(sinSigma * sinAlpha1,
		cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1);
	double C = f / 16.0 * cosSqAlpha * (4.0 + f * (4.0 - 3.0 * cosSqAlpha));
	double L = lambda - (1.0 - C) * f * sinAlpha *
		(sigma + C * sinSigma *
			(cos2SigmaM + C * cosSigma * (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM)));

	double lng2 = lng + L * (180.0 / PI);
	double lat2Deg = lat2 * (180.0 / PI);

	return { lng2, lat2Deg };
}


/**
*  @brief   单个三角面片网格化
			需要的实现：
*           1.  isObjectValueEqual ：判断两个IJH行列号点三维坐标是否相等
*           2.  collinear_3Points检查三点是否共线
*           3.  getUnique 二维点去重
*           4.  calculatePlaneEquation计算平面方程
*           5.  bresenham2D
*           6.  bresenham3D
*  @author  AnYiZong
*
*****************************************************************************
*/
#define DEBUG_PRINT(expr) std::cout << #expr << " = " << (expr) << std::endl;

vector<IJH> triangularGrid(const IJH& p1, const IJH& p2, const IJH& p3, const uint8_t level) {

	std::vector<IJH> result;
	const uint32_t MAX_COORD = (1U << level) - 1;
	const uint32_t ErrorV = static_cast<uint32_t>(MAX_COORD * 0.99);
	// DEBUG：如果任意一个点的 layer > 500，则打印并直接返回空结果
	//if (p1.layer > ErrorV || p2.layer > ErrorV || p3.layer > ErrorV) {
	// std::cerr << "[ TriangularGrid INPUT ERROR] Input point exceeds layer limit:\n";
	//	if (p1.layer > ErrorV)
	//	std::cerr << "  p1: (" << p1.row << ", " << p1.column << ", " << p1.layer << ")\n";
	//	if (p2.layer > ErrorV)
	//	std::cerr << "  p2: (" << p2.row << ", " << p2.column << ", " << p2.layer << ")\n";
	//	if (p3.layer > ErrorV)
	//	std::cerr << "  p3: (" << p3.row << ", " << p3.column << ", " << p3.layer << ")\n";
	//	return result;  // 返回空结果
	//}


	if (p1.layer > ErrorV || p2.layer > ErrorV || p3.layer > ErrorV) {
		return result;
	}

	// 共点检查
	if (p1 == p2 && p2 == p3) {
		result.push_back(p1);
		return result;
	}

	// 共线检查
	if (collinear_3Points(p1, p2, p3)) {
		/*std::cout << "[DEBUG] Collinear check triggered.\n";
		std::cout << "p1: (" << p1.row << ", " << p1.column << ", " << p1.layer << ")\n";
		std::cout << "p2: (" << p2.row << ", " << p2.column << ", " << p2.layer << ")\n";
		std::cout << "p3: (" << p3.row << ", " << p3.column << ", " << p3.layer << ")\n";*/
		auto line12 = bresenham3D(p1, p2);
		auto line23 = bresenham3D(p2, p3);
		auto line13 = bresenham3D(p1, p3);

		// 合并结果并去重
		std::vector<IJH> merged;
		merged.reserve(line12.size() + line23.size() + line13.size());
		merged.insert(merged.end(), line12.begin(), line12.end());
		merged.insert(merged.end(), line23.begin(), line23.end());
		merged.insert(merged.end(), line13.begin(), line13.end());

		//std::cout << "[DEBUG] Merged before sort: size = " << merged.size() << "\n";
		std::sort(merged.begin(), merged.end(), [](const IJH& a, const IJH& b) {
			if (a.row != b.row) return a.row < b.row;
			if (a.column != b.column) return a.column < b.column;
			return a.layer < b.layer;
			});

		auto last = std::unique(merged.begin(), merged.end());
		merged.erase(last, merged.end());

		return merged;
	}

	// 共面处理（XY平面投影）
	const int64_t x1 = p1.column, y1 = p1.row, z1 = p1.layer;
	const int64_t x2 = p2.column, y2 = p2.row, z2 = p2.layer;
	const int64_t x3 = p3.column, y3 = p3.row, z3 = p3.layer;

	// 计算平面参数
	const int64_t a = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
	const int64_t b = (x3 - x1) * (z2 - z1) - (x2 - x1) * (z3 - z1);
	const int64_t c = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
	const int64_t d = a * x1 + b * y1 + c * z1;


	// 计算各轴绝对差值
	const int64_t abs_a = std::abs(a);
	const int64_t abs_b = std::abs(b);
	const int64_t abs_c = std::abs(c);
	const int64_t max_abs = std::max({ abs_a, abs_b, abs_c });

	// 判断投影平面（XY平面）
	const bool diff_abc = (abs_a != abs_b) && (abs_a != abs_c) && (abs_b != abs_c);
	bool is_xy_plane = false;

	if ((max_abs == abs_c && diff_abc) ||
		(abs_c == abs_a && abs_c > abs_b) ||
		(abs_c > abs_a && abs_a == abs_b)) {
		is_xy_plane = true;
	}

	if (is_xy_plane) {
		// 投影到XY平面
		const IJ p1_xy{ p1.row, p1.column };
		const IJ p2_xy{ p2.row, p2.column };
		const IJ p3_xy{ p3.row, p3.column };

		// 生成边界的二维点
		auto edge12 = bresenham2D(p1_xy, p2_xy);
		auto edge13 = bresenham2D(p1_xy, p3_xy);
		auto edge23 = bresenham2D(p2_xy, p3_xy);

		// 合并并去重
		std::vector<IJ> all_points;
		all_points.reserve(edge12.size() + edge13.size() + edge23.size());
		all_points.insert(all_points.end(), edge12.begin(), edge12.end());
		all_points.insert(all_points.end(), edge13.begin(), edge13.end());
		all_points.insert(all_points.end(), edge23.begin(), edge23.end());

		std::sort(all_points.begin(), all_points.end(), [](const IJ& a, const IJ& b) {
			return std::tie(a.row, a.column) < std::tie(b.row, b.column);
			});

		auto last = std::unique(all_points.begin(), all_points.end(),
			[](const IJ& a, const IJ& b) { return a.row == b.row && a.column == b.column; });
		all_points.erase(last, all_points.end());

		// 按行组织并填充列
		std::map<uint32_t, std::vector<uint32_t>> row_map;
		for (const auto& pt : all_points) {
			row_map[pt.row].push_back(pt.column);
		}

		// 填充水平线
		std::vector<IJ> filled;
		for (const auto& [row, cols] : row_map) {
			if (cols.empty()) continue;
			const auto [min_it, max_it] = std::minmax_element(cols.begin(), cols.end());
			for (uint32_t c = *min_it; c <= *max_it; ++c) {
				filled.push_back({ row, c });
			}
		}

		// 计算Z坐标并生成最终结果
		for (const auto& ij : filled) {
			const int64_t numerator = d - a * ij.column - b * ij.row;
			const double z = static_cast<double>(numerator) / c;
			 uint32_t layer = static_cast<uint32_t>(std::abs(std::round(z)));
			if (layer > MAX_COORD) {
				std::cerr << "[XY]Layer " << layer << " exceeds max coord at ("
					<< ij.row << ", " << ij.column << ")\n";
				throw std::runtime_error("Coordinate exceeds level limit");
			}

			result.push_back({ ij.row, ij.column, layer });
		}
	}

	// YZ平面投影处理
	const bool is_yz_plane = (max_abs == abs_a && diff_abc) ||
		(abs_a == abs_b && abs_a > abs_c) ||
		(abs_a > abs_b && abs_b == abs_c) ||
		(abs_a == abs_b && abs_b == abs_c && abs_a != 0);

	if (is_yz_plane) {
		// 投影到YZ平面（使用Row作为Y坐标，layer作为Z坐标）
		const IJ p1_yz{ p1.row, p1.layer };
		const IJ p2_yz{ p2.row, p2.layer };
		const IJ p3_yz{ p3.row, p3.layer };

		// 生成边界点
		auto edge12 = bresenham2D(p1_yz, p2_yz);
		auto edge13 = bresenham2D(p1_yz, p3_yz);
		auto edge23 = bresenham2D(p2_yz, p3_yz);

		// 合并并去重
		std::vector<IJ> all_points;
		all_points.reserve(edge12.size() + edge13.size() + edge23.size());
		all_points.insert(all_points.end(), edge12.begin(), edge12.end());
		all_points.insert(all_points.end(), edge13.begin(), edge13.end());
		all_points.insert(all_points.end(), edge23.begin(), edge23.end());

		std::sort(all_points.begin(), all_points.end(), [](const IJ& a, const IJ& b) {
			return std::tie(a.row, a.column) < std::tie(b.row, b.column);
			});

		auto last = std::unique(all_points.begin(), all_points.end(),
			[](const IJ& a, const IJ& b) { return a.row == b.row && a.column == b.column; });
		all_points.erase(last, all_points.end());

		// 按行组织数据（Y坐标）
		std::map<uint32_t, std::vector<uint32_t>> row_map;
		for (const auto& pt : all_points) {
			row_map[pt.row].push_back(pt.column);  // row对应Y，column对应Z
		}

		// 填充垂直线（Z方向）
		std::vector<IJ> filled;
		for (const auto& [y, z_values] : row_map) {
			if (z_values.empty()) continue;
			auto [min_z, max_z] = std::minmax_element(z_values.begin(), z_values.end());
			for (uint32_t z = *min_z; z <= *max_z; ++z) {
				filled.push_back({ y, z });  // IJ结构：row=Y, column=Z
			}
		}

		// 计算X坐标并生成最终结果
		for (const auto& yz : filled) {
			const int64_t numerator = d - b * yz.row - c * yz.column;
			if (a == 0) {
				throw std::runtime_error("Invalid plane coefficient a is zero in YZ projection");
			}

			const double x = static_cast<double>(numerator) / a;
			const uint32_t column = static_cast<uint32_t>(std::round(x));

			if (column > MAX_COORD ||
				yz.row > MAX_COORD ||
				yz.column > MAX_COORD) {
				std::cerr << "[YZ]column: " << column << " exceeds max coord at ("
					<< column << ", " << yz.row << ", " << yz.column<< ")\n";
				throw std::runtime_error("Coordinate exceeds level limit");
			}

			result.push_back({ yz.row, column, yz.column }); // row=Y, column=X, layer=Z
		}
	}
// XZ平面投影处理
	const bool is_xz_plane = (max_abs == abs_b && diff_abc) ||
		(abs_b == abs_c && abs_b > abs_a) ||
		(abs_b > abs_c && abs_c == abs_a) ||
		(abs_b == abs_c && abs_c == abs_a && abs_b != 0);

	if (is_xz_plane) {
		// 投影到XZ平面（使用Col作为X坐标，layer作为Z坐标）
		const IJ p1_xz{ p1.column, p1.layer };  // X -> column, Z -> column
		const IJ p2_xz{ p2.column, p2.layer };
		const IJ p3_xz{ p3.column, p3.layer };

		// 生成边界点
		auto edge12 = bresenham2D(p1_xz, p2_xz);
		auto edge13 = bresenham2D(p1_xz, p3_xz);
		auto edge23 = bresenham2D(p2_xz, p3_xz);

		// 合并并去重
		std::vector<IJ> all_points;
		all_points.reserve(edge12.size() + edge13.size() + edge23.size());
		all_points.insert(all_points.end(), edge12.begin(), edge12.end());
		all_points.insert(all_points.end(), edge13.begin(), edge13.end());
		all_points.insert(all_points.end(), edge23.begin(), edge23.end());

		std::sort(all_points.begin(), all_points.end(), [](const IJ& a, const IJ& b) {
			return std::tie(a.row, a.column) < std::tie(b.row, b.column);
			});

		auto last = std::unique(all_points.begin(), all_points.end(),
			[](const IJ& a, const IJ& b) { return a.row == b.row && a.column == b.column; });
		all_points.erase(last, all_points.end());

		// 按列组织数据（X坐标）
		std::map<uint32_t, std::vector<uint32_t>> col_map;
		for (const auto& pt : all_points) {
			col_map[pt.row].push_back(pt.column);  // row对应X，column对应Z
		}

		// 填充垂直线（Z方向）
		std::vector<IJ> filled;
		for (const auto& [x, z_values] : col_map) {
			if (z_values.empty()) continue;
			auto [min_z, max_z] = std::minmax_element(z_values.begin(), z_values.end());
			for (uint32_t z = *min_z; z <= *max_z; ++z) {
				filled.push_back({ x, z });  // IJ结构：row=X, column=Z
			}
		}

		// 计算Y坐标并生成最终结果
		for (const auto& xz : filled) {
			const int64_t numerator = d - a * xz.row - c * xz.column;
			if (b == 0) {

				throw std::runtime_error("Invalid plane coefficient b is zero in XZ projection");
			}

			const double y = static_cast<double>(numerator) / b;
			const uint32_t row = static_cast<uint32_t>(std::abs(std::round(y)));

			if (row > MAX_COORD ||
				xz.row > MAX_COORD ||  // X坐标检查
				xz.column > MAX_COORD) { // Z坐标检查
				std::cerr << "[XZ]column: " << row << " exceeds max coord at ("
					<< row << ", " << xz.row << ", " <<  xz.column << ")\n";
				throw std::runtime_error("Coordinate exceeds level limit");
			}

			result.push_back({ row, xz.row, xz.column }); // row=Y, column=X, layer=Z
		}
	/*	for (auto& retIJH : result)
		{
			cout << "[final]IJH:(" << retIJH.row << "," << retIJH.column << "," << retIJH.layer << ")\n";
		}*/
	}
	return result;
}

/**
*****************************************************************************
*  @brief   倾斜摄影网格化
			需要的实现：
*           1.  localGridConfig ：局部格网配置
*           2.  localGrid：确定局部剖分基础网格边界坐标
*           3.  localRowColHeiNumber ：已知局部三维空间坐标，求局部的行列层号
*           4.  triangularGrid：单个三角面片网格化
*           5.  getLocalTileLatLon：已知局部网格编码获取网格的经纬高度
*           6.  mortonEncode_3D_LUT： 根据行列层号通过查找表法计算mordon码

*  @author  AnYiZong
*
*****************************************************************************
*/
// 三角形网格化
vector<string> triangular_single(const Triangle& triangle, uint8_t level, const BaseTile& baseTile) {
	const uint32_t max_coord = (1U << level) - 1;
	auto convert_vertex = [&](const PointLBHd& v) {
		//三角面片转行列层号
		//level = LEVEL;
		//该函数debug后，没有算术溢出现象
		auto [I, J, H] = localRowColHeiNumber(level, v.Lng, v.Lat, v.Hgt, baseTile);
		return IJH{ I, J, H };
		};

	try {
		vector<IJH> vertices = {
			convert_vertex(triangle.vertex1),
			convert_vertex(triangle.vertex2),
			convert_vertex(triangle.vertex3)
		};

		auto grid_cells = triangularGrid(vertices[0], vertices[1], vertices[2],level);

		//debug_2
		for (const auto& grid_cell : grid_cells)
		{
			if (grid_cell.row > max_coord ||
				grid_cell.column > max_coord ||
				grid_cell.layer > max_coord) {
				cout << "Error：triangularGrid_result__IJH>Max: (" << grid_cell.row << ", " << grid_cell.column << ", " << grid_cell.layer << ")\n";
			}
		}
		set<string> codes;
		for (const auto& cell : grid_cells) {
			std::string code = IJH2DQG_str(cell.row, cell.column, cell.layer, level);
			codes.insert(code);
		}

		return vector<string>(codes.begin(), codes.end());
	}
	catch (const exception& e) {
		// 错误处理：记录日志或返回空结果
		//cerr << "Error processing triangle: " << e.what() << endl;
		return {};
	}
}

//倾斜模型网格化
// 三角形网格化函数
vector<string> triangular_multiple(const vector<Triangle>& triangles, uint8_t level, const BaseTile& baseTile) {
	set<string> resultCodes;

	for (const auto& triangle : triangles) {
		vector<string> triangleCodes = triangular_single(triangle, level, baseTile);
		resultCodes.insert(triangleCodes.begin(), triangleCodes.end());
	}

	return vector<string>(resultCodes.begin(), resultCodes.end());
}



/**
*****************************************************************************
*  @brief   倾斜摄影建筑物格网化
			需要的实现：
*           1.  localGridConfig ：局部格网配置
*           2.  localGrid：确定局部剖分基础网格边界坐标
*           3.  localRowColHeiNumber ：已知局部三维空间坐标，求局部的行列层号
*           4.  triangularGrid：单个三角面片网格化
*           5.  getLocalTileLatLon：已知局部网格编码获取网格的经纬高度
*           6.  mortonEncode_3D_LUT： 根据行列层号通过查找表法计算mordon码

*  @author  AnYiZong
*
*****************************************************************************
*/

Pagoda pagoda1 = {
	{108.9564, 34.2228}, // A
	{108.9625, 34.2227}, // B
	{108.9565, 34.2170}, // C
	{108.9624, 34.2169}  // D
};

vector<PointLBHd> pagodaTriangular(const std::vector<PointLBHd>& pointArray, uint8_t level, const std::vector<int>& index) {
	vector<map<string, double>> result;
	auto config = localGridConfig(pagoda1.A, pagoda1.B, pagoda1.C, pagoda1.D);
	auto baseTile = localGrid(pagoda1.A, pagoda1.B, pagoda1.C, pagoda1.D, 11, 0);

	vector<IJH> rowColResult;
	for (const auto& point : pointArray) {
		auto rowCol = localRowColHeiNumber(level, point.Lng, point.Lat, point.Hgt, baseTile);
		rowColResult.push_back(rowCol);
	}

	//2. 生成所有三角网格
	vector<IJH> allTriangular;
	for (size_t i = 0; i < index.size(); i += 3) {
		auto triangle = triangularGrid(
			rowColResult[index[i]],
			rowColResult[index[i + 1]],
			rowColResult[index[i + 2]],
			level
		);
		allTriangular.insert(allTriangular.end(), triangle.begin(), triangle.end());
	}


	// 3.去重
	vector<IJH> uniqTriangular;
	uniqTriangular = getUnique(allTriangular);

	// 4.计算 Morton 编码
	vector<std::string> triangularCode;
	for (const auto& point : uniqTriangular) {
		uint64_t code = mortonEncode_3D_LUT(point.row, point.column, point.layer);
		std::string codeStr = std::to_string(code);
		// 补零操作
		if (codeStr.length() < static_cast<size_t>(level)) {
			codeStr = std::string(level - codeStr.length(), '0') + codeStr;
		}
		triangularCode.push_back(codeStr);
	}
	// 5.生成 lineBoundary
	std::vector<PointLBHd> lineBoundary;
	for (const auto& code : triangularCode) {
		auto latLonHei = getLocalTileLatLon(code, baseTile);
		lineBoundary.push_back({ latLonHei.longitude, latLonHei.latitude, latLonHei.height });
	}

	return lineBoundary;
}


/**
*****************************************************************************
*  @brief   倾斜摄影空域网格化
*           需要的实现：
* struct Point_LBd;
* struct pointLBHd;
* struct IJH;
* struct BaseTile;
* struct LatLonHei;
* uint64_t mortonEncode_3D_LUT(uint64_t y, uint64_t x, uint64_t z);
* IJH localRowColHeiNumber(int level, double longitude, double latitude, double height, const BaseTile& baseTile);
* BaseTile localGrid(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D, int level, double height);
* LocalGridConfig localGridConfig(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D);
* std::vector<IJH> bresenham3D(const IJH& point1, const IJH& point2);
* std::vector<IJH> getUnique(const std::vector<IJH>& points);
*  @author  AnYiZong
*
*****************************************************************************
*/

AirClear  airClear1 = {
	{108.9564, 34.2228}, // A
	{108.9625, 34.2227}, // B
	{108.9565, 34.2170}, // C
	{108.9624, 34.2169}  // D
};


//std::vector<std::string> airSpaceGrid(const std::vector<PointLBHd>& data, uint8_t level) {
//
//	// 获取 localGridConfig 和 baseTile
//	auto config = localGridConfig(airClear1.A, airClear1.B, airClear1.C, airClear1.D);
//	auto baseTile = localGrid(airClear1.A, airClear1.B, airClear1.C, airClear1.D, config.idLevel, 0.0);
//
//	// 计算顶点的行列层
//	std::vector<IJH> buildPointRCH;
//	for (const auto& point : data) {
//		auto rowCol = localRowColHeiNumber(level, point.Lng, point.Lat, point.Hgt, baseTile);
//		buildPointRCH.push_back(rowCol);
//	}
//
//	// 计算三维线段并合并结果
//	std::vector<IJH> resultRCH;
//	for (size_t i = 0; i < buildPointRCH.size() - 1; ++i) {
//		auto line = bresenham3D(buildPointRCH[i], buildPointRCH[i + 1]);
//		resultRCH.insert(resultRCH.end(), line.begin(), line.end());
//	}
//
//	// 连接最后一个点和第一个点
//	auto endLine = bresenham3D(buildPointRCH.back(), buildPointRCH.front());
//	resultRCH.insert(resultRCH.end(), endLine.begin(), endLine.end());
//
//	// 去重
//	auto unique = getUnique(resultRCH);
//
//	// 计算 Morton 编码
//	std::vector<std::string> lineCode;
//	for (const auto& point : unique) {
//		uint64_t code = mortonEncode_3D_LUT(point.row, point.column, point.layer);
//		std::string codeStr = std::to_string(code);
//
//		// 补零操作
//		if (codeStr.length() < static_cast<size_t>(level)) {
//			codeStr = std::string(level - codeStr.length(), '0') + codeStr;
//		}
//		lineCode.push_back(codeStr);
//	}
//
//	return lineCode;
//}
//




/**
*****************************************************************************
*  @brief   倾斜摄影建筑物网格化
*           需要的实现：
* struct Point_LBd;
* struct pointLBHd;
* struct IJH;
* struct BaseTile;
* struct LatLonHei;
* uint64_t mortonEncode_3D_LUT(uint64_t y, uint64_t x, uint64_t z);
* IJH localRowColHeiNumber(int level, double longitude, double latitude, double height, const BaseTile& baseTile);
* BaseTile localGrid(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D, int level, double height);
* LocalGridConfig localGridConfig(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D);
* std::vector<IJH> bresenham3D(const IJH& point1, const IJH& point2);
* std::vector<IJH> getUnique(const std::vector<IJH>& points);
*  @author  AnYiZong
*
*****************************************************************************
*/

BuildingArea  buildingArea1 =
{
	{108.9564, 34.2228}, // A
	{108.9625, 34.2227}, // B
	{108.9565, 34.2170}, // C
	{108.9624, 34.2169}  // D
};
std::vector<LatLonHei> buildingsGrid(const std::vector<PointLBHd>& data, uint8_t level) {

	// 获取 localGridConfig 和 baseTile
	auto config = localGridConfig(buildingArea1.A, buildingArea1.B, buildingArea1.C, buildingArea1.D);
	auto baseTile = localGrid(buildingArea1.A, buildingArea1.B, buildingArea1.C, buildingArea1.D, config.idLevel, 0.0);

	// 计算顶点的行列层
	std::vector<IJH> buildPointRCH;
	for (const auto& point : data) {
		auto rowCol = localRowColHeiNumber(level, point.Lng, point.Lat, point.Hgt, baseTile);
		buildPointRCH.push_back(rowCol);
	}

	// 将顶点分块
	std::vector<std::vector<IJH>> point;
	size_t loop = (buildPointRCH.size() + 3) / 4;
	for (size_t i = 0; i < loop; ++i) {
		point.push_back(std::vector<IJH>(
			buildPointRCH.begin() + i * 4,
			buildPointRCH.begin() + std::min((i + 1) * 4, buildPointRCH.size())
		));
	}

	// 计算所有面片
	std::vector<IJH> faceArray;
	for (const auto& block : point) {
		std::vector<IJH> resultRCH;
		for (size_t j = 0; j < block.size() - 1; ++j) {
			auto line = bresenham3D(block[j], block[j + 1]);
			resultRCH.insert(resultRCH.end(), line.begin(), line.end());
		}
		auto endLine = bresenham3D(block.back(), block.front());
		resultRCH.insert(resultRCH.end(), endLine.begin(), endLine.end());

		// 去重
		auto unique = getUnique(resultRCH);

		// 按行填充列
		std::vector<IJH> filled;
		for (const auto& u : unique) {
			for (uint32_t c = u.column; c <= u.column; ++c) {
				filled.push_back({ u.row, c, u.layer });
			}
		}
		faceArray.insert(faceArray.end(), filled.begin(), filled.end());
	}

	// 将结果展开为所有层的 Morton 编码
	std::vector<IJH> newResult = faceArray;
	for (const auto& item : faceArray) {
		for (uint32_t h = item.layer - 1; h >= 0; --h) {
			newResult.push_back({ item.row, item.column, h });
		}
	}

	// 计算 Morton 编码
	std::vector<std::string> lineCode;
	for (const auto& point : newResult) {
		uint64_t code = mortonEncode_3D_LUT(point.row, point.column, point.layer);
		std::string codeStr = std::to_string(code);

		// 补零操作
		if (codeStr.length() < static_cast<size_t>(level)) {
			codeStr = std::string(level - codeStr.length(), '0') + codeStr;
		}
		lineCode.push_back(codeStr);
	}

	// 计算边界
	std::vector<LatLonHei> lineBoundary;
	for (const auto& code : lineCode) {
		auto latLonHei = getLocalTileLatLon(code, baseTile);
		lineBoundary.push_back(latLonHei);
	}

	return lineBoundary;
}

///**
//*****************************************************************************
//*  @brief   2d倾斜模型线网格化
//*           需要的实现：
//* IJH localRowColHeiNumber(int level, double longitude, double latitude, double height, const BaseTile& baseTile);
//* BaseTile localGrid(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D, int level, double height);
//* LocalGridConfig localGridConfig(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D);
//* std::vector<IJH> bresenham3D(const IJH& point1, const IJH& point2);
//* std::vector<IJH> getUnique(const std::vector<IJH>& points);
//*  @author  AnYiZong
//*
//*****************************************************************************
//*/
//
//std::vector<PointLBHd> lineGrid(const std::vector<PointLBHd>& pointArray, int level) {
//    // 获取局部格网配置和基础瓦片
//    auto config = localGridConfig(airClear1.A, airClear1.B, airClear1.C, airClear1.D);
//    auto baseTile = localGrid(airClear1.A, airClear1.B, airClear1.C, airClear1.D, config.idLevel, 0);
//
//    // 转换顶点为行列号
//    std::vector<IJ> rowColResult;
//    for (const auto& point : pointArray) {
//        auto RowCol = localRowColHeiNumber(level, point.Lng, point.Lat, point.Hgt, baseTile);
//        rowColResult.push_back({ RowCol.row, RowCol.column });
//    }
//
//    // 计算二维线段网格
//    std::vector<IJ> result;
//    for (size_t i = 0; i < rowColResult.size() - 1; ++i) {
//        auto line = bresenham2D(rowColResult[i], rowColResult[i + 1]);
//        result.insert(result.end(), line.begin(), line.end());
//    }
//    auto endLine = bresenham2D(rowColResult.back(), rowColResult.front());
//    result.insert(result.end(), endLine.begin(), endLine.end());
//
//    // 去重
//    std::vector<IJ> unique;
//
//    unique = getUnique(result);
//
//    // Morton 编码
//    std::vector<std::string> lineCode;
//    for (const auto& item : unique) {
//        uint64_t mortonCode = mortonEncode_3D_LUT(item.row, item.column, 0);
//        lineCode.push_back(std::to_string(mortonCode));
//    }
//
//    // 转换为边界坐标
//    std::vector<PointLBHd> lineBoundary;
//    for (const auto& code : lineCode) {
//        auto boundaryPoint = getLocalTileLatLon(code, baseTile);
//        boundaryPoint.code = code; // Assuming pointLBHd has a Morton field
//        lineBoundary.push_back({ boundaryPoint.longitude,boundaryPoint.latitude,boundaryPoint.height });
//    }
//
//    return lineBoundary;
//}
///**
//*****************************************************************************
//*  @brief   3d倾斜模型线网格化
//*           需要的实现：
//* IJH localRowColHeiNumber(int level, double longitude, double latitude, double height, const BaseTile& baseTile);
//* BaseTile localGrid(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D, int level, double height);
//* LocalGridConfig localGridConfig(const Point_LBd& A, const Point_LBd& B, const Point_LBd& C, const Point_LBd& D);
//* std::vector<IJH> bresenham3D(const IJH& point1, const IJH& point2);
//* std::vector<IJH> getUnique(const std::vector<IJH>& points);
//*  @author  AnYiZong
//*
//*****************************************************************************
//*/
//
//std::vector<PointLBHd> lineGrid3D(const std::vector<PointLBHd>& pointArray, int level) {
//    // 获取局部格网配置和基础瓦片
//    auto config = localGridConfig({ 108.9564, 34.2228 }, { 108.9625, 34.2227 }, { 108.9565, 34.2170 }, { 108.9624, 34.2169 });
//    auto baseTile = localGrid({ 108.9564, 34.2228 }, { 108.9625, 34.2227 }, { 108.9565, 34.2170 }, { 108.9624, 34.2169 }, config.idLevel, 0);
//
//    // 转换顶点为行列层号
//    std::vector<IJH> rowColResult;
//    for (const auto& point : pointArray) {
//        auto RowCol = localRowColHeiNumber(level, point.Lng, point.Lat, point.Hgt, baseTile);
//        rowColResult.push_back({ RowCol.row, RowCol.column, RowCol.layer });
//    }
//
//    // 计算三维线段网格
//    std::vector<IJH> result;
//    for (size_t i = 0; i < rowColResult.size() - 1; ++i) {
//        auto line = bresenham3D(rowColResult[i], rowColResult[i + 1]);
//        result.insert(result.end(), line.begin(), line.end());
//    }
//    auto endLine = bresenham3D(rowColResult.back(), rowColResult.front());
//    result.insert(result.end(), endLine.begin(), endLine.end());
//
//    // 去重
//    std::unordered_set<IJH> unique(result.begin(), result.end());
//
//    // Morton 编码
//    std::vector<std::string> lineCode;
//    for (const auto& item : unique) {
//        uint64_t mortonCode = mortonEncode_3D_LUT(item.row, item.column, item.layer);
//        lineCode.push_back(std::to_string(mortonCode));
//    }
//
//    // 转换为边界坐标
//    std::vector<PointLBHd> lineBoundary;
//    for (const auto& code : lineCode) {
//        auto boundaryPoint = getLocalTileLatLon(code, baseTile);
//        boundaryPoint.code = code; // Assuming LatLonHei has a Morton field
//        lineBoundary.push_back({ boundaryPoint.longitude, boundaryPoint.latitude, boundaryPoint.latitude });
//    }
//
//    return lineBoundary;
//}

/// @brief 倾斜模型坐标点转换
/// @param dataArray1 
/// @param dataArray2 
/// @return 
vector<PointLBHd> getPointData(const vector<array<double, 3>>& dataArray1, const vector<int>& dataArray2) {
	vector<PointLBHd> pointArray;
	for (const auto& data : dataArray1) {
		double dist = distance(data[0], data[1]);
		double angle = azimuth(data[0], data[1]);
		PointLBd lngLat = getLngLat(108.9594, 34.2196, angle, dist);
		pointArray.push_back({ lngLat.Lng, lngLat.Lat, data[2] });
	}
	return pointArray;
}

