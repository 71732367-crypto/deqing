/*
Copyright (c) [2025] [AnYiZong]
All rights reserved.
No part of this code may be copied or distributed in any form without permission.
*/

#include <dqg/DQG3DBuffer.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <array>
#include <set>
#include <climits>

/// @brief 计算最小外接正方体的经纬高范围
/// @param lon
/// @param lat
/// @param height
/// @param R
/// @return
BaseTile getBoundingBox(double lon, double lat, double height, double R) {
    double latRad = toRadians(lat); // 纬度转弧度
    double deltaLon = toDegrees(R / (R_E * cos(latRad))); // 经度变化量 (°)
    double deltaLat = toDegrees(R / R_E); // 纬度变化量 (°)

    // BaseTile order: west, south, east, north, top, bottom
    return {
        lon - deltaLon, // west
        lat - deltaLat, // south
        lon + deltaLon, // east
        lat + deltaLat, // north
        height + R,     // top
        height - R      // bottom
    };
}

/// @brief 3D点缓冲区网格
/// @param points
/// @param level
/// @param radius
/// @param basetile
/// @return
vector<vector<LatLonHei>> getPointsBuffer(const vector<PointLBHd>& points,
    uint8_t level,
    double radius,
    const BaseTile& basetile)
{
    vector<vector<LatLonHei>> result;
    for (const auto& p : points) {
        // 计算基准行列层
        IJH center = localRowColHeiNumber(level, p.Lng, p.Lat, p.Hgt, basetile);

        // 计算缓冲范围
        BaseTile range = getBoundingBox(p.Lng, p.Lat, p.Hgt, radius);

        // 计算步长
        double lonStep = (basetile.east - basetile.west) / (1 << level);
        double latStep = (basetile.north - basetile.south) / (1 << level);
        double hgtStep = (basetile.top - basetile.bottom) / (1 << level);

        // 计算各方向扩展量
        int dWest = ceil((p.Lng - range.west) / lonStep);
        int dEast = ceil((range.east - p.Lng) / lonStep);
        int dSouth = ceil((p.Lat - range.south) / latStep);
        int dNorth = ceil((range.north - p.Lat) / latStep);
        int dBelow = ceil((p.Hgt - range.bottom) / hgtStep);
        int dAbove = ceil((range.top - p.Hgt) / hgtStep);

        // 生成候选网格（使用有符号索引并裁剪边界，避免无符号下溢）
        vector<IJH> candidates;
        int64_t maxIndex = (1LL << level) - 1;
        int64_t startC = std::max<int64_t>(0, static_cast<int64_t>(center.column) - dWest);
        int64_t endC = std::min<int64_t>(maxIndex, static_cast<int64_t>(center.column) + dEast);
        int64_t startR = std::max<int64_t>(0, static_cast<int64_t>(center.row) - dSouth);
        int64_t endR = std::min<int64_t>(maxIndex, static_cast<int64_t>(center.row) + dNorth);
        int64_t startH = std::max<int64_t>(0, static_cast<int64_t>(center.layer) - dBelow);
        int64_t endH = std::min<int64_t>(maxIndex, static_cast<int64_t>(center.layer) + dAbove);

        for (int64_t c = startC; c <= endC; ++c) {
            for (int64_t r = startR; r <= endR; ++r) {
                for (int64_t h = startH; h <= endH; ++h) {
                    candidates.push_back({ static_cast<uint32_t>(r), static_cast<uint32_t>(c), static_cast<uint32_t>(h) });
                }
            }
        }

        // 精确距离过滤
        vector<LatLonHei> buffer;
        for (const auto& ijh : candidates) {
            LatLonHei TEMP = IJHToLocalTileLatLon(ijh, level, basetile);
            PointLBHd center = { TEMP.longitude,TEMP.latitude,TEMP.height };
            if (distance3D(center, p) <= radius + hgtStep / 2) {
                try {
                    LatLonHei item;
                    item.longitude = center.Lng;
                    item.latitude = center.Lat;
                    item.height = center.Hgt;
                    item.code = IJH2DQG_str(ijh.row, ijh.column, ijh.layer, level);

                    // 计算边界
                    item.west = basetile.west + ijh.column * lonStep;
                    item.east = item.west + lonStep;
                    item.south = basetile.south + ijh.row * latStep;
                    item.north = item.south + latStep;
                    item.bottom = basetile.bottom + ijh.layer * hgtStep;
                    item.top = item.bottom + hgtStep;

                    buffer.push_back(item);
                } catch (const std::exception &e) {
                    // 忽略单个网格错误，继续处理
                    continue;
                }
            }
        }
        result.push_back(buffer);
    }

    return result;
}

/// @brief 三维线缓冲区网格（矩形截面）
/// @param lineReq 三维线坐标序列
/// @param halfWidth 截面矩形的半宽度
/// @param halfHeight 截面矩形的半高度
/// @param level 层级
/// @param baseTile 基准瓦片
/// @return 三维线缓冲区网格编码序列
vector<string> lineBufferFilled(const vector<PointLBHd>& lineReq, double halfWidth, double halfHeight,
                                uint8_t level, const BaseTile& baseTile)
{
    if (lineReq.size() < 2) {
        return {};
    }

    // 取第一点作为局部投影原点
    PointLBHd origin = lineReq[0];

    // 工具函数：经纬度 -> 局部ENU坐标
    auto llhToLocal = [&](const PointLBHd& point) -> array<double, 3> {
        double dx = toRadians(point.Lng - origin.Lng) * R_E * cos(toRadians(origin.Lat));
        double dy = toRadians(point.Lat - origin.Lat) * R_E;
        double dz = point.Hgt - origin.Hgt;
        return {dx, dy, dz};
    };

    // 工具函数：ENU坐标 -> 地理坐标
    auto enuToGeodetic = [&](const array<double, 3>& enu) -> PointLBHd {
        double cosLat = cos(toRadians(origin.Lat));
        double deltaLonRad = enu[0] / (R_E * cosLat);
        double deltaLatRad = enu[1] / R_E;

        return {
            origin.Lng + toDegrees(deltaLonRad),
            origin.Lat + toDegrees(deltaLatRad),
            origin.Hgt + enu[2]
        };
    };

    // 工具函数：向量运算
    auto sub = [](const array<double, 3>& a, const array<double, 3>& b) -> array<double, 3> {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    };

    auto add = [](const array<double, 3>& a, const array<double, 3>& b) -> array<double, 3> {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    };

    auto mul = [](const array<double, 3>& v, double s) -> array<double, 3> {
        return {v[0] * s, v[1] * s, v[2] * s};
    };

    auto norm = [](const array<double, 3>& v) -> array<double, 3> {
        double l = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        return {v[0]/l, v[1]/l, v[2]/l};
    };

    auto cross = [](const array<double, 3>& a, const array<double, 3>& b) -> array<double, 3> {
        return {
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        };
    };

    // 核心函数：计算矩形截面的AABB包围盒
    auto expandRectAABB = [&](const array<double, 3>& center, const array<double, 3>& r,
                             const array<double, 3>& u, double halfW, double halfH) -> array<double, 6> {
        // 4个角点的世界坐标
        array<array<double, 3>, 4> corners = {
            add(add(center, mul(r, halfW)), mul(u, halfH)),
            add(add(center, mul(r, halfW)), mul(u, -halfH)),
            add(add(center, mul(r, -halfW)), mul(u, halfH)),
            add(add(center, mul(r, -halfW)), mul(u, -halfH))
        };

        double xmin = corners[0][0], xmax = corners[0][0];
        double ymin = corners[0][1], ymax = corners[0][1];
        double zmin = corners[0][2], zmax = corners[0][2];

        for (const auto& c : corners) {
            xmin = min(xmin, c[0]); xmax = max(xmax, c[0]);
            ymin = min(ymin, c[1]); ymax = max(ymax, c[1]);
            zmin = min(zmin, c[2]); zmax = max(zmax, c[2]);
        }

        return {xmin, xmax, ymin, ymax, zmin, zmax};
    };

    // 核心函数：将AABB包围盒转换为体素网格
    auto fillAABBToVoxels = [&](const array<double, 6>& aabb, set<string>& result) {
        double xmin = aabb[0], xmax = aabb[1];
        double ymin = aabb[2], ymax = aabb[3];
        double zmin = aabb[4], zmax = aabb[5];

        // 8个角点的ENU坐标
        array<array<double, 3>, 8> cornersENU = {{
            {xmin, ymin, zmin}, {xmax, ymin, zmin}, {xmin, ymax, zmin}, {xmax, ymax, zmin},
            {xmin, ymin, zmax}, {xmax, ymin, zmax}, {xmin, ymax, zmax}, {xmax, ymax, zmax}
        }};

        // 转为地理坐标并计算网格范围
        int64_t minR = LLONG_MAX, maxR = LLONG_MIN;
        int64_t minC = LLONG_MAX, maxC = LLONG_MIN;
        int64_t minH = LLONG_MAX, maxH = LLONG_MIN;

        for (const auto& enu : cornersENU) {
            PointLBHd geo = enuToGeodetic(enu);
            IJH rch = localRowColHeiNumber(level, geo.Lng, geo.Lat, geo.Hgt, baseTile);

            minR = min(minR, static_cast<int64_t>(rch.row));
            maxR = max(maxR, static_cast<int64_t>(rch.row));
            minC = min(minC, static_cast<int64_t>(rch.column));
            maxC = max(maxC, static_cast<int64_t>(rch.column));
            minH = min(minH, static_cast<int64_t>(rch.layer));
            maxH = max(maxH, static_cast<int64_t>(rch.layer));
        }

        // 填充网格，添加边界检查
        int64_t maxCoord = (1LL << level) - 1;
        for (int64_t r = minR; r <= maxR; r++) {
            if (r < 0 || r > maxCoord) continue; // 边界检查
            for (int64_t c = minC; c <= maxC; c++) {
                if (c < 0 || c > maxCoord) continue; // 边界检查
                for (int64_t h = minH; h <= maxH; h++) {
                    if (h < 0 || h > maxCoord) continue; // 边界检查
                    IJH ijh = {static_cast<uint32_t>(r), static_cast<uint32_t>(c), static_cast<uint32_t>(h)};
                    try {
                        string code = IJH2DQG_str(ijh.row, ijh.column, ijh.layer, level);
                        result.insert(code);
                    } catch (const std::exception& e) {
                        // 忽略单个网格编码错误，继续处理其他网格
                        continue;
                    }
                }
            }
        }
    };

    // 坐标转换：将所有点转换为局部ENU坐标
    vector<array<double, 3>> pts;
    for (const auto& p : lineReq) {
        pts.push_back(llhToLocal(p));
    }

    set<string> result;

    // 处理每条线段
    for (size_t i = 0; i < pts.size() - 1; i++) {
        const auto& P0 = pts[i];
        const auto& P1 = pts[i + 1];

        array<double, 3> segVec = sub(P1, P0);
        double segLen = sqrt(segVec[0]*segVec[0] + segVec[1]*segVec[1] + segVec[2]*segVec[2]);

        if (segLen < 1e-6) continue; // 跳过零长度线段

        // 方向向量d
        array<double, 3> d = norm(segVec);

        // 构造稳定的侧向向量r
        array<double, 3> r = cross({0, 0, 1}, d);
        double rLen = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
        if (rLen < 1e-6) {
            // 如果d垂直，则换另一个up向量
            r = cross({1, 0, 0}, d);
            rLen = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
        }
        r = {r[0]/rLen, r[1]/rLen, r[2]/rLen};

        // 上方向向量u
        array<double, 3> u_temp = cross(d, r);
        double uLen = sqrt(u_temp[0]*u_temp[0] + u_temp[1]*u_temp[1] + u_temp[2]*u_temp[2]);
        array<double, 3> u = {u_temp[0]/uLen, u_temp[1]/uLen, u_temp[2]/uLen};

        // 处理P0、P1端点
        array<double, 6> aabb0 = expandRectAABB(P0, r, u, halfWidth, halfHeight);
        fillAABBToVoxels(aabb0, result);

        array<double, 6> aabb1 = expandRectAABB(P1, r, u, halfWidth, halfHeight);
        fillAABBToVoxels(aabb1, result);

        // 沿线段采样，防止遗漏
        // 使用固定采样间隔，确保足够的采样密度
        const double sampleInterval = 1; // 固定采样间隔
        const int numSamples = static_cast<int>(ceil(segLen / sampleInterval));

        for (int s = 0; s <= numSamples; s++) {
            double t = static_cast<double>(s) / numSamples; // t ∈ [0, 1]
            array<double, 3> pointOnSeg = add(P0, mul(d, t * segLen)); // 插值得到采样点

            array<double, 6> aabb = expandRectAABB(pointOnSeg, r, u, halfWidth, halfHeight);
            fillAABBToVoxels(aabb, result);
        }
    }

    return vector<string>(result.begin(), result.end());
}