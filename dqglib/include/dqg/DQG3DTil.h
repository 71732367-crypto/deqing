

#ifndef __DQG3DTIL_HPP__
#define __DQG3DTIL_HPP__
#include <dqg/DQG3DBasic.h>
#include <vector>
#include <string>



/// @brief bresenham2D
std::vector<IJ> bresenham2D(const IJ& p1, const IJ& p2);
/// @brief bresenham3D
std::vector<IJH> bresenham3D(const IJH& p1, const IJH& p2);

/// @brief  根据一个经纬度、距离和方位角计算另一个经纬度
PointLBd getLngLat(double lng, double lat, double brng, double dist);
/// @brief 单个三角面片网格化
vector<IJH> triangularGrid( const IJH& p1, const IJH& p2, const IJH& p3 , const uint8_t level);
/// @brief 单个三角面片网格化
vector<string> triangular_single(const Triangle& points, uint8_t level, const BaseTile& baseTile);
/// @brief 多个三角面片网格化
vector<string> triangular_multiple(const vector<Triangle>& triangles, uint8_t level, const BaseTile& baseTile);
/// @brief 倾斜摄影模型格网化
vector<PointLBHd> pagodaTriangular(const std::vector<PointLBHd>& pointArray, uint8_t level, const std::vector<int>& index);
/// @brief 倾斜摄影空域网格化
std::vector<std::string> airSpaceGrid(const std::vector<PointLBHd>& data, uint8_t level);
/// @brief 倾斜摄影建筑物格网化
std::vector<LatLonHei> buildingsGrid(const std::vector<PointLBHd>& data, uint8_t level);
/// @brief 2d倾斜模型线网格化
//std::vector<PointLBHd> lineGrid(const std::vector<PointLBHd>& pointArray, uint8_t level);
/// @brief 3d倾斜模型线网格化
//std::vector<PointLBHd> lineGrid3D(const std::vector<PointLBHd>& pointArray, uint8_t level);
/// @brief 倾斜模型坐标点转换
vector<PointLBHd> getPointData(const vector<array<double, 3>>& dataArray1, const vector<int>& dataArray2);

/// @brief 三维线网格化为局部网格
std::vector<std::string> lineToLocalCode(const std::vector<PointLBHd>& lineReq, uint8_t level, const BaseTile& baseTile);

#endif // __DQG3DTIL_HPP__