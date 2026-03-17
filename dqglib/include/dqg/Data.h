#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
#include <stdint.h>

using namespace std;


/*---------------------------------------------------*/
/*---------------网格编码结构体-----------------------*/
/*---------------------------------------------------*/

struct code_data
{
    uint32_t          size;	/* Do not touch this field directly! */
    uint64_t* data;	/* Data content is here */
};


/*---------------------------------------------------*/
/*---------------行列号和经纬高结构体-----------------*/
/*---------------------------------------------------*/

struct IJ {
    uint32_t row;
    uint32_t column;

    // 为IJ添加比较运算符
    bool operator<(const IJ& other) const {
        return tie(row, column) < tie(other.row, other.column);
    }
};

struct IJH {
    uint32_t row;
    uint32_t column;
    uint32_t layer;

    bool operator==(const IJH& other) const {
        return tie(row, column, layer) == tie(other.row, other.column, other.layer);
    }

    // 比较运算符
    bool operator<(const IJH& other) const {
        return tie(row, column, layer) < tie(other.row, other.column, other.layer);
    }
};

struct PointLBd {
    double Lng;
    double Lat;
};

struct PointLBHd
{
    double Lng;
    double Lat;
    double Hgt;
};


struct Triangle {
    PointLBHd vertex1;
    PointLBHd vertex2;
    PointLBHd vertex3;
};

struct CoordinateOffset {
    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    
    // 默认构造函数
    CoordinateOffset() = default;
    
    // 带参数的构造函数
    CoordinateOffset(double x, double y, double z) 
        : offsetX(x), offsetY(y), offsetZ(z) {}
};

//行列号八分体号经纬差信息
struct IJ_oct_int {
    uint32_t i;
    uint32_t j;
    uint8_t oct;
    uint8_t level;
};

struct IJH_oct_int {
    uint32_t i;
    uint32_t j;
    uint32_t h;
    uint8_t oct;
    uint8_t level;

};

/// @brief 网格包围盒
struct Gridbox
{
    double west;
    double east;
    double north;
    double south;
    double bottom;
    double top;

    double Lng;
    double Lat;
    double Hgt;

    uint32_t row;
    uint32_t column;
    uint32_t layer;

    uint8_t  level;
    uint16_t  octNum;

    string code;

};

/// @brief BaseTile 
struct BaseTile {
    double west;
    double south;

    double east;
    double north;

    double top;
    double bottom;
};


/// @brief 局部网格包围盒
struct LatLonHei {
    double latitude;
    double longitude;
    double height;

    double west;
    double south;
    double east;
    double north;
    double bottom;
    double top;
    string code;
};
/// @brief 局部网格配置
struct LocalGridConfig {
    std::vector<string> globalCodeSet;
    bool isAcrossDegeneration;
    bool isAcrossOctant;
    std::vector<string> localCode2D;
    uint8_t idLevel;
};

/**
 * @brief 米坐标系三维向量结构体
 *
 * 用于几何计算中的米坐标系点表示（局部坐标系）。
 * 注意：与 PointLBHd 的区别在于坐标系不同：
 * - V3: 米坐标系（x, y, z 单位为米），用于局部几何计算
 * - PointLBHd: 经纬度坐标系（Lng, Lat, Hgt），用于地理坐标表示
 */
struct V3
{
    double x;  // X坐标（米）
    double y;  // Y坐标（米）
    double z;  // Z坐标（米，高度）
};
