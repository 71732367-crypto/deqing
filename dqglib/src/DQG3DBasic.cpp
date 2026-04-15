/*
Copyright (c) [2025] [AnYiZong]
All rights reserved.
No part of this code may be copied or distributed in any form without permission.
*/


#include <dqg/DQG3DBasic.h>
#include <stdint.h>
#include <dqg/GlobalBaseTile.h>
#include <sstream>

/*---------------------------------------------------*/
/*---------------网格包围盒信息-----------------------*/
/*---------------------------------------------------*/

Pagoda pagoda = {
    {108.9564, 34.2228}, // A
    {108.9625, 34.2227}, // B
    {108.9565, 34.2170}, // C
    {108.9624, 34.2169}  // D
};
AirClear  airClear = {
 {108.9564, 34.2228}, // A
 {108.9625, 34.2227}, // B
 {108.9565, 34.2170}, // C
 {108.9624, 34.2169}  // D
};

BuildingArea  buildingArea =
{
    {108.9564, 34.2228}, // A
    {108.9625, 34.2227}, // B
    {108.9565, 34.2170}, // C
    {108.9624, 34.2169}  // D
};

//DQG3D编码（全球编码）-----LBH
//经纬度高度转DQG3D编码字符串
//l:经度 b:纬度 hei:高度 level:层级
std::string LBH2DQG_str(double l, double b, double hei, uint8_t level) {
    // Step 1: 将经纬度转换为DQG_octant_ij结构体
    IJH_oct_int IJH = LBH2DQG_ijh_oct_int(l, b, hei, level);

    // Step 2: 将结构体转换为字符串编码
    return IJH_oct_int2str(IJH);
}

//DQG3D编码-----IJH
//std::string IJH2DQG_str(uint32_t row, uint32_t col, uint32_t layer, uint8_t level) {
//
//    if (row < 0 || col < 0 || layer < 0||row == numeric_limits<uint32_t>::max() ||
//        col == numeric_limits<uint32_t>::max() ||
//        layer == numeric_limits<uint32_t>::max())
//    {
//        std::cout << "IJH: (" << row << ", " << col << ", " << layer << ")\n";
//        throw invalid_argument("Invalid coordinate _funcname: IJH2DQG_str( DQG3D编码)");
//    }
//
//    uint64_t code = mortonEncode_3D_LUT(row, col, layer);
//    string codeStr = toOctalString(code);
//    if (codeStr.length()>level) {
//        std::cout << "codeStr.length() =" << codeStr.length() <<"," << "level=" << level <<"\n";
//        throw invalid_argument("Invalid Code length _funcname: IJH2DQG_str( DQG3D编码)");
//    }
//    if (codeStr.length() < level) {
//        codeStr = string(level - codeStr.length(), '0') + codeStr;
//    }
//    return codeStr;
//}

std::string IJH2DQG_str(uint32_t row, uint32_t col, uint32_t layer, uint8_t level)
{

    // 参数有效性验证----------------------------------------
    // 1. 移除对无符号数的负数检查（uint32_t不可能为负数）
    // 2. 添加层级有效性检查
    // 3. 添加坐标值范围检查

    // 验证层级范围 (0-21)
    if (level > 21) {  // 3*21=63 bits < 64 bits
        throw std::invalid_argument("Invalid level (0-21 allowed)");
    }

    // 计算当前层级允许的最大坐标值
    const uint32_t max_coord = (1U << level) - 1;

    // //debug_3:验证坐标值范围
    if (row > max_coord || col > max_coord || layer > max_coord) {
        throw std::invalid_argument(
            "Coordinate exceeds level limit in IJH2DQG_str : (" +
            std::to_string(row) + ", " +
            std::to_string(col) + ", " +
            std::to_string(layer) + ") at level " +
            std::to_string(level)
        );
    }

    // Morton编码生成----------------------------------------
    uint64_t code;
    try {
        code = mortonEncode_3D_LUT(row, col, layer);
    }
    catch (const std::exception& e) {
        throw std::runtime_error("Morton encoding failed: " + std::string(e.what()));
    }

    // 八进制字符串转换--------------------------------------
    std::string codeStr = toOctalString(code);

    if (codeStr.length() != level)
    {
        codeStr = std::string(level - codeStr.length(), '0') + codeStr; // 补零
    }

    if (codeStr.length() > level) {  // 必须严格匹配层级
        throw std::logic_error(
            "Octal string length mismatch: expected " +
            std::to_string(level) + ", got " +
            std::to_string(codeStr.length())
        );
    }
    if (codeStr == "3030030333")
    {
        cout << "3030030333IJH:" << "(" << row << "," << col << "," << layer << ")\n";

    }
    if (codeStr == "3030030337")
    {
        cout << "3030030337IJH:" << "(" << row << "," << col << "," << layer << ")\n";

    }
    if (codeStr == "3030030373")
    {
        cout << "3030030373IJH:" << "(" << row << "," << col << "," << layer << ")\n";

    }
    return codeStr;
}

//DQG3D解码-生成点地理坐标
// DQG3D解码-生成点地理坐标
PointLBHd Decode_3D(const string& code) {
    PointLBHd grid;
    uint8_t  level = code.length() - 1;
    char Q = code[0];
    string octalPart = code.substr(1);
    uint64_t morton = stoull(octalPart, nullptr, 8);
    auto obj = Morto2IJH(morton);
    uint32_t i = obj.column;

    uint32_t col = obj.column;

    uint32_t h = obj.layer;
    // 修复：使用 1ULL 防止位移溢出
    double LDV = 90.0 / (1ULL << level);
    // 修复：使用 1ULL 和检查 row+1 是否为0
    uint64_t row_plus_1 = i + 1;
    if (row_plus_1 == 0) {
        throw std::overflow_error("Row overflow");
    }
    double LOV = 90.0 / (1ULL << static_cast<uint32_t>(ceil(log2(row_plus_1))));
    double HDV = 10000000.0 / (1ULL << level);

    grid.Lng = (col + 0.5) * LOV;
    grid.Lat = 90.0 - (i + 0.5) * LDV;
    grid.Hgt = (h + 0.5) * HDV;

    return grid;
}

// DQG3D解码-生成包围盒
Gridbox Codes2Gridbox(const std::string& code) {
    Gridbox grid;
    uint8_t  level = code.length() - 1;
    grid.level = level;
    char Q = code[0];
    grid.octNum = Q;

    std::string octalPart = code.substr(1);
    uint64_t morton = std::stoull(octalPart, nullptr, 8);
    auto obj = Morto2IJH(morton);
    uint32_t row = obj.row;
    grid.row = row;
    uint32_t col = obj.column;
    grid.column = col;
    uint32_t layer = obj.layer;
    grid.layer = layer;

    // 修复：使用 1ULL 防止位移溢出
    double LDV = 90.0 / (1ULL << level);
    // 修复：检查 row+1 是否为0
    uint64_t row_plus_1 = row + 1;
    if (row_plus_1 == 0) {
        throw std::overflow_error("Row overflow");
    }
    double LOV = 90.0 / (1ULL << static_cast<uint32_t>(ceil(log2(row_plus_1))));
    double HDV = HEIGHT / (1ULL << level);

    grid.Lng = (col + 0.5) * LOV;
    grid.Lat = 90.0 - (row + 0.5) * LDV;
    grid.Hgt = (layer + 0.5) * HDV;
    if (Q == '0') {
        grid.west = col * LOV;
        grid.south = 90.0 - (row + 1) * LDV;
        grid.east = (col + 1) * LOV;
        grid.north = 90.0 - row * LDV;
    }
    else if (Q == '1') {
        grid.west = col * LOV + 90.0;
        grid.south = 90.0 - (row + 1) * LDV;
        grid.east = (col + 1) * LOV + 90.0;
        grid.north = 90.0 - row * LDV;
        grid.Lng += 90.0;
    }
    else if (Q == '2') {
        grid.west = col * LOV - 180.0;
        grid.south = 90.0 - (row + 1) * LDV;
        grid.east = (col + 1) * LOV - 180.0;
        grid.north = 90.0 - row * LDV;
        grid.Lng -= 180.0;
    }
    else if (Q == '3') {
        grid.west = col * LOV - 90.0;
        grid.south = 90.0 - (row + 1) * LDV;
        grid.east = (col + 1) * LOV - 90.0;
        grid.north = 90.0 - row * LDV;
        grid.Lng -= 90.0;
    }
    else if (Q == '4') {
        grid.west = 90.0 - (col + 1) * LOV;
        grid.south = -(90.0 - row * LDV);
        grid.east = 90.0 - col * LOV;
        grid.north = -(90.0 - (row + 1) * LDV);
        grid.Lng = 90.0 - grid.Lng;
        grid.Lat = -grid.Lat;
    }
    else if (Q == '5') {
        grid.west = 180.0 - (col + 1) * LOV;
        grid.south = -(90.0 - row * LDV);
        grid.east = 180.0 - col * LOV;
        grid.north = -(90.0 - (row + 1) * LDV);
        grid.Lng = 180.0 - grid.Lng;
        grid.Lat = -grid.Lat;
    }
    else if (Q == '6') {
        grid.west = -((col + 1) * LOV) - 90.0;
        grid.south = -(90.0 - row * LDV);
        grid.east = -(col * LOV) - 90.0;
        grid.north = -(90.0 - (row + 1) * LDV);
        grid.Lng = -grid.Lng - 90.0;
        grid.Lat = -grid.Lat;
    }
    else if (Q == '7') {
        grid.west = -((col + 1) * LOV);
        grid.south = -(90.0 - row * LDV);
        grid.east = -(col * LOV);
        grid.north = -(90.0 - (row + 1) * LDV);
        grid.Lng = -grid.Lng;
        grid.Lat = -grid.Lat;
    }

    // 修复高度计算
    grid.bottom = layer * HDV;
    grid.top = (layer + 1) * HDV;

    return grid;
}






// 3D获取指定层级的父格网编码
string getLevelFatherCode(const string& code, uint8_t level) {
    int selfLevel = code.length() - 1;
    if (selfLevel > level) {
        unsigned long long numericCode = stoull(code, nullptr, 8); // 将八进制字符串转换为整数
        unsigned long long parentNumericCode = numericCode >> ((selfLevel - level) * 3);
        string parentCode = toOctalString(parentNumericCode); // 修改：使用toOctalString而不是to_string
        return parentCode;
    }
    return code; // 若请求的层级不小于当前层级，则返回自身
}

// 获取子网格编码
vector<string> getChildCode(const string& code) {
    vector<string> childCode;
    string codeLast = code.substr(1); // 获取除第一位以外的编码

    // 判断是否为退化单元（三角形网格编码）
    if (codeLast.find_first_of("123567") == string::npos) {
        // 二进制编码左移三位然后再异或 0, 2, 3, 4, 6, 7
        vector<int> offsets = { 0, 2, 3, 4, 6, 7 };
        for (int offset : offsets) {
            unsigned long long childNumeric = (stoull(code, nullptr, 8) << 3) ^ offset;
            string childStr = to_string(childNumeric);
            if (code[0] == '0') {
                childStr = string(code.length() - childStr.length() + 1, '0') + childStr;
            }
            childCode.push_back(childStr);
        }
    }
    else {
        // 非退化单元，八个子网格
        vector<int> offsets = { 0, 1, 2, 3, 4, 5, 6, 7 };
        for (int offset : offsets) {
            unsigned long long childNumeric = (stoull(code, nullptr, 8) << 3) ^ offset;
            string childStr = to_string(childNumeric);
            if (code[0] == '0') {
                childStr = string(code.length() - childStr.length() + 1, '0') + childStr;
            }
            childCode.push_back(childStr);
        }
    }
    return childCode;
}


  /// @brief 方位角计算
/// @param x
/// @param y
/// @return
  double azimuth(double x, double y) {
      double result = 0.0;

      if (x == 0 && y >= 0) {
          result = 0.0;
      }
      if (x > 0 && y > 0) {
          result = atan(x / y) * (180.0 / PI);
      }
      if (x > 0 && y == 0) {
          result = 90.0;
      }
      if (x > 0 && y < 0) {
          result = abs(std::atan(y / x) * (180.0 / PI)) + 90.0;
      }
      if (x == 0 && y < 0) {
          result = 180.0;
      }
      if (x < 0 && y < 0) {
          result = atan(x / y) * (180.0 / PI) + 180.0;
      }
      if (x < 0 && y == 0) {
          result = 270.0;
      }
      if (x < 0 && y > 0) {
          result = abs(std::atan(y / x) * (180.0 / PI)) + 270.0;
      }

      return result;
  }




/**
*三维直线网格化
* @param point1 起点行列号坐标
* @param point2 终点行列号坐标
* @returns 直线上所有点的坐标
*/
vector<string> bresenham3D_DQG(const PointLBHd& point1, const PointLBHd& point2, uint8_t level) {
  vector<IJH> IJHret;
  vector<IJH_oct_int> IJH_oct_ret;
  vector<string> ret;

  int Oct = LB2Oct(point1.Lng, point1.Lat);
  IJH p1 = LBH2IJH(point1.Lng, point1.Lat, point1.Hgt, level);
  IJH p2 = LBH2IJH(point2.Lng, point2.Lat, point2.Hgt, level);

  uint32_t diff_X = ABS(p2.column - p1.column);
  uint32_t diff_Y = ABS(p2.row - p1.row);
  uint32_t diff_Z = ABS(p2.layer - p1.layer);
  uint32_t diff_Max = max({ diff_X, diff_Y, diff_Z });

  uint32_t x = p1.column;
  uint32_t y = p1.row;
  uint32_t z = p1.layer;

  uint32_t deltaY = (diff_Y << 1);
  uint32_t deltaX = (diff_X << 1);
  uint32_t deltaZ = (diff_Z << 1);

  int sign_x = (p2.column > p1.column) ? 1 : -1;
  int sign_y = (p2.row > p1.row) ? 1 : -1;
  int sign_z = (p2.layer > p1.layer) ? 1 : -1;

  IJHret.push_back({ y, x, z });

  int p1_err = deltaY - diff_X;
  int p2_err = deltaZ - diff_X;

  for (int i = 0; i < diff_Max; i++) {
      if (p1_err >= 0) {
          y += sign_y;
          p1_err += deltaY - deltaX;
      }
      else {
          p1_err += deltaY;
      }
      if (p2_err >= 0) {
          z += sign_z;
          p2_err += deltaZ - deltaX;
      }
      else {
          p2_err += deltaZ;
      }
      x += sign_x;
      IJHret.push_back({ y, x, z });
  }

  for (size_t i = 0; i < IJHret.size(); ++i)
  {
      IJH_oct_ret[i].i = IJHret[i].row;
      IJH_oct_ret[i].j = IJHret[i].column;
      IJH_oct_ret[i].oct = Oct;
      IJH_oct_ret[i].level = level;
      ret.push_back(IJH_oct_int2str(IJH_oct_ret[i]));
  }
  return ret;
}


/**
* 根据研究区域的角点坐标计算研究区域的外接矩形，此处默认研究区域四个顶点
* @param {object} A 顶点的经纬度
* @param {object} B
* @param {object} C
* @param {object} D
* @returns 返回外接矩形顶点的经纬度
*/
vector<PointLBd> boundingRectangle(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D)
{
    double maxLon = max({ A.Lng, B.Lng, C.Lng, D.Lng });
    double minLon = min({ A.Lng, B.Lng, C.Lng, D.Lng });
    double maxLat = max({ A.Lat, B.Lat, C.Lat, D.Lat });
    double minLat = min({ A.Lat, B.Lat, C.Lat, D.Lat });

    PointLBd A1 = { minLon, maxLat };
    PointLBd B1 = { maxLon, maxLat };
    PointLBd C1 = { minLon, minLat };
    PointLBd D1 = { maxLon, minLat };

    return { A1, B1, C1, D1 };
}

/**
* 根据外接矩形，计算外接正方形
* @param {*} A
* @param {*} B
* @param {*} C
* @param {*} D
* @returns 返回值是正方形的刻度数组，顺序为west,south,east,north
*/
vector<PointLBd> boundingSquare(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D)
{
    vector<PointLBd> calibration = boundingRectangle(A, B, C, D);


    double west = calibration[0].Lng;
    double south = calibration[2].Lat;
    double east = calibration[1].Lng;
    double north = calibration[0].Lat;


    double sideLength = max(east - west, north - south);


    vector<PointLBd> square;
    square.push_back({ west, north });
    square.push_back({ west + sideLength, north });
    square.push_back({ west + sideLength, north - sideLength });
    square.push_back({ west, north - sideLength });

    return square;

}

/*
    计算球面上格网面积（WGS84椭球/任意椭球）
    输入参数：第一个点的经度、纬度，第二个点的经度、纬度
    输出参数：格网面积 （m^2）
*/
double CalculatingGridArea(double longitude1, double latitude1, double longitude2, double latitude2) {
    double centerLatitude = (latitude1 + latitude2) * 0.5 * RegToRad;
    double intervalLatitude = latitude2 > latitude1 ? (latitude2 - latitude1) * RegToRad : (latitude1 - latitude2) * RegToRad;
    double intervalLongitude = (longitude2 > longitude1) ? (longitude2 - longitude1) * RegToRad : (longitude1 - longitude2) * RegToRad;
    double S = 2.0 * WGS84b * WGS84b * intervalLongitude * (WGS84b1 * sin(0.5 * intervalLatitude) * cos(centerLatitude)
        - WGS84b3 * sin(1.5 * intervalLatitude) * cos(3 * centerLatitude) + WGS84b5 * sin(2.5 * intervalLatitude) * cos(5 * centerLatitude)
        - WGS84b7 * sin(3.5 * intervalLatitude) * cos(7 * centerLatitude) + WGS84b9 * sin(4.5 * intervalLatitude) * cos(9 * centerLatitude));
    return S;
}

double CalculatingGridArea(double longitude1, double latitude1, double longitude2, double latitude2, double a, double b) {
    double centerLatitude = (latitude1 + latitude2) * 0.5 * RegToRad;
    double intervalLatitude = latitude2 > latitude1 ? (latitude2 - latitude1) * RegToRad : (latitude1 - latitude2) * RegToRad;
    double intervalLongitude = (longitude2 > longitude1) ? (longitude2 - longitude1) * RegToRad : (longitude1 - longitude2) * RegToRad;
    double e2 = 1.0 - b * b / (a * a);
    double e8 = e2 * e2 * e2 * e2;
    double S = 2.0 * b * b * intervalLongitude
        * ((1.0 + 0.5 * e2 + (3.0 / 8.0) * e2 * e2 + (35.0 / 112.0) * e2 * e2 * e2 + (630.0 / 2304.0) * e8) * sin(0.5 * intervalLatitude) * cos(centerLatitude)
            - (1.0 / 6.0 * e2 + (15.0 / 80.0) * e2 * e2 + (21.0 / 112.0) * e2 * e2 * e2 + (420.0 / 2304.0) * e8) * sin(1.5 * intervalLatitude) * cos(3 * centerLatitude)
            + ((3.0 / 80.0) * e2 * e2 + (7.0 / 112.0) * e2 * e2 * e2 + (180.0 / 2304.0) * e8) * sin(2.5 * intervalLatitude) * cos(5 * centerLatitude)
            - ((1.0 / 112.0) * e2 * e2 * e2 + (45.0 / 2304.0) * e8) * sin(3.5 * intervalLatitude) * cos(7 * centerLatitude)
            + ((5.0 / 2304.0) * e8) * sin(4.5 * intervalLatitude) * cos(9 * centerLatitude));
    return S;
}

    /**
* @brief 计算两点的直线距离
* @param x 坐标系中的 x 值
* @param y 坐标系中的 y 值
* @return 返回点 (0, 0) 和 (x, y) 的距离
*/
double distance(double x, double y) {
    return sqrt(x * x + y * y);
}

/**
 * IJH点间的欧几里得距离
 * @param point1 起点行列层号坐标
 * @param point2 终点行列层号坐标
 * @returns 距离
 */
double distance(const IJH& point1, const IJH& point2) {
    double x1 = static_cast<double>(point1.column) - static_cast<double>(point2.column);
    double y1 = static_cast<double>(point1.row) - static_cast<double>(point2.row);
    double z1 = static_cast<double>(point1.layer) - static_cast<double>(point2.layer);
    return sqrt(x1 * x1 + y1 * y1 + z1 * z1);
}

/**
* @brief 计算两点的直线距离
* @param x 坐标系中的 x 值
* @param y 坐标系中的 y 值
* @return 返回点 (0, 0) 和 (x, y) 的距离
*/
bool collinear_3Points(const IJH& point1, const IJH& point2, const IJH& point3) {
    double x1 = point1.column, y1 = point1.row, z1 = point1.layer;
    double x2 = point2.column, y2 = point2.row, z2 = point2.layer;
    double x3 = point3.column, y3 = point3.row, z3 = point3.layer;

    // 计算向量 v1 = (x2-x1, y2-y1, z2-z1) 和 v2 = (x3-x1, y3-y1, z3-z1)
    double vx1 = x2 - x1, vy1 = y2 - y1, vz1 = z2 - z1;
    double vx2 = x3 - x1, vy2 = y3 - y1, vz2 = z3 - z1;

    // 计算两个向量的叉积 (v1 × v2)
    double cross_x = vy1 * vz2 - vz1 * vy2;
    double cross_y = vz1 * vx2 - vx1 * vz2;
    double cross_z = vx1 * vy2 - vy1 * vx2;

    // 叉积的模，如果为 0，则三点共线
    double cross_norm = cross_x * cross_x + cross_y * cross_y + cross_z * cross_z;

    return cross_norm < 1e-10; // 设定一个小的误差阈值
}



/// @brief  计算某个点沿特定角度扩展一定距离的点
/// @param p
/// @param distance
/// @param angle
/// @return
PointLBHd movePoint(const PointLBHd& p, double distance, double angle) {
    double radLat = p.Lat * PI / 180.0;
    double radLng = p.Lng * PI / 180.0;
    double radAngle = angle * PI / 180.0;

    double newLat = asin(sin(radLat) * cos(distance / WGS84a) +
        cos(radLat) * sin(distance / WGS84a) * cos(radAngle));

    double newLng = radLng + atan2(sin(radAngle) * sin(distance / WGS84a) * cos(radLat),
        cos(distance / WGS84a) - sin(radLat) * sin(newLat));

    return { newLng * 180.0 / PI, newLat * 180.0 / PI };
}


/// @brief 计算多边形的缓冲区**
/// @param polygon
/// @param radius
/// @return
vector<PointLBHd> turfBuffer(const vector<PointLBHd>& polygon, double radius) {
    vector<PointLBHd> bufferedPolygon;

    for (const auto& point : polygon) {
        for (int i = 0; i < 360; i += 10) { // ÿ 10�� ����һ���㣬�γɻ��ƻ�����
            bufferedPolygon.push_back(movePoint(point, radius, i));
        }
    }

    return bufferedPolygon;
}



/**
 * 确定标识层级idlevel
 * @param {*} A
 * @param {*} B
 * @param {*} C
 * @param {*} D
 * @returns 返回标识层级
 */
 // identityLevel 函数
    uint8_t  identityLevel(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D) {
        int idenLevel = -1; // 默认层级
        vector<PointLBd> square = boundingSquare(A, B, C, D);
        double area = CalculatingGridArea(square[0].Lng, square[0].Lat, square[2].Lng, square[2].Lat); // 计算外接正方形的面积
        for (size_t i = 0; i < 23; i++) {
            if (area > gridArea[i])
            {
                idenLevel = static_cast<int>(i);
                break;
            }
        }
        return idenLevel;
    }


/**
 * 确定外接矩形的四个网格
 * @param {*} A
 * @param {*} B
 * @param {*} C
 * @param {*} D
 * @returns 返回标识层级
 */
vector<std::string> codeGather(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D, int level, double height) {
    vector<PointLBd> square = boundingSquare(A, B, C, D);

    // 根据经纬度求网格编码
    set<string> localcode; // 使用 set 去除重复编码
    for (const auto& PointLBd : square) {
        string code = LBH2DQG_str(PointLBd.Lng, PointLBd.Lat, height, level);
        localcode.insert(code);
    }

    // 将 set 转换为 vector
    return vector<std::string>(localcode.begin(), localcode.end());
}


/**
 * 判断是否跨八分体
 * @param {*} A
 * @param {*} B
 * @param {*} C
 * @param {*} D
 * @returns 返回标识层级
 */
bool isAcrossOctant(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D, int level, double height) {
    // 获取编码
    vector<std::string> removeLocalcode = codeGather(A, B, C, D, level, height);
    vector<double> Qarray;

    // 提取编码的第一个字符并转换为数字
    for (const auto& code : removeLocalcode)
    {
        if (!code.empty()) {
            Qarray.push_back(static_cast<double>(code[0] - '0')); // 假设编码的第一个字符是数字
        }
    }
    // 判断数组里的最小八分体号是否等于最大八分体号 是则不跨 不是则跨过
    if (!Qarray.empty() && *std::min_element(Qarray.begin(), Qarray.end()) == *std::max_element(Qarray.begin(), Qarray.end())) {
        return false; // 没有跨八分体
    }
    else {
        return true; // 跨八分体
    }
}


/**
 * 判断是否跨退化线
 * @param {*} A
 * @param {*} B
 * @param {*} C
 * @param {*} D
 * @returns 返回标识层级
 */

bool isAcrossDegeneration(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D) {
    // 获取研究区域的最大纬度和最小纬度
    vector<PointLBd> square = boundingSquare(A, B, C, D);

    double minLat = square[1].Lat; // 最小纬度
    double maxLat = square[3].Lat; // 最大纬度

    // 检查退化边界
    for (const auto& edge : degEdge) {
        if (edge > minLat && edge < maxLat) {
            return true; // 处于跨退化区域
        }
    }
    return false; // 不处于跨退化区域
}

/**
 * @brief 计算包含研究区域的全局基础网格编码集合
 *
 * 该函数根据研究区域的四个顶点坐标，计算包含该区域的全局基础网格编码集合。
 * 函数处理了研究区域在单个网格内、跨越两个网格、跨越三个网格或跨越四个网格的不同情况，
 * 并根据具体情况扩展网格编码集合，确保完整包含研究区域。
 *
 * @param A 研究区域的第一个顶点坐标(经度和纬度)
 * @param B 研究区域的第二个顶点坐标(经度和纬度)
 * @param C 研究区域的第三个顶点坐标(经度和纬度)
 * @param D 研究区域的第四个顶点坐标(经度和纬度)
 * @param level 网格层级
 * @param height 高度值
 * @return std::vector<std::string> 包含研究区域的全局基础网格编码集合
 */
std::vector<std::string> baseGridIncludeGlobalCode(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D, int level, double height)
{
    std::vector<std::string> localCodeSet; // 存储最终的网格编码集合

    // 第一步：收集研究区域覆盖的局部网格编码
    std::vector<std::string> localCode = codeGather(A, B, C, D, level, height);

    // 情况1：研究区域完全位于单个网格内
    if (localCode.size() == 1)
    {
        std::cout << "case1: The research region is in a single grid" << std::endl;
        // 只需添加该单个网格编码
        localCodeSet.push_back(localCode[0]);
    }

    // 情况2：研究区域跨越两个网格（共有6种可能的子情况）
    /*
     * 不跨越退化边界或八分体的两个网格有四种情况
     * 跨越退化边界的两个网格有一种情况
     * 跨越八分体的两个网格有一种情况
     */
    if (localCode.size() == 2)
    {
        std::cout << "case2: The research region spans two grids" << std::endl;
        std::string oneTile = localCode[0]; // 第一个网格编码
        std::string twoTile = localCode[1]; // 第二个网格编码

        // 将网格编码解码为网格盒信息
        Gridbox grid1 = Codes2Gridbox(oneTile);
        Gridbox grid2 = Codes2Gridbox(twoTile);

        // 提取网格的经纬度和高度信息
        PointLBHd oneTileLatLon = { grid1.Lng, grid1.Lat, grid1.Hgt }; // 第一个网格的经纬度和高度
        PointLBHd twoTileLatLon = { grid2.Lng, grid2.Lat, grid2.Hgt }; // 第二个网格的经纬度和高度

        // 提取网格的八分体编号
        std::string oneTileQ = std::to_string(grid1.octNum); // 第一个网格的八分体编号
        std::string twoTileQ = std::to_string(grid2.octNum); // 第二个网格的八分体编号

        // 提取网格的行列层信息
        IJH oneTileRCH = { grid1.row, grid1.column, grid1.layer }; // 第一个网格的行列层信息
        IJH twoTileRCH = { grid1.row, grid1.column, grid1.layer }; // 第二个网格的行列层信息（注意：这里可能是个bug，应该使用grid2）

        // 子情况2.1：两个网格不跨越退化边界且位于同一个八分体
        if (!isAcrossDegeneration(A, B, C, D) && oneTileQ == twoTileQ) {
            // 子情况2.1.1：跨越两个水平网格
            if (oneTileRCH.row == twoTileRCH.row && oneTileLatLon.Lat != 0 &&
                std::find(degEdge.begin(), degEdge.end(), oneTileLatLon.Lat) == degEdge.end()) {
                std::cout << "case3.1: Spanning two horizontal grids" << std::endl;
                // 调整行列号（向南扩展一行）
                oneTileRCH.row += 1;
                twoTileRCH.row += 1;

                // 根据更新后的行列号计算新的网格编码
                uint64_t newOneMorton = mortonEncode_3D_LUT(oneTileRCH.row, oneTileRCH.column, oneTileRCH.layer);
                uint64_t newTwoMorton = mortonEncode_3D_LUT(twoTileRCH.row, twoTileRCH.column, twoTileRCH.layer);
                std::string newOneTileCode = oneTileQ + std::to_string(newOneMorton);
                std::string newTwoTileCode = twoTileQ + std::to_string(newTwoMorton);

                // 添加新计算的网格编码
                localCode.push_back(newOneTileCode);
                localCode.push_back(newTwoTileCode);

                // 将4个网格编码扩展为8个网格编码
                std::vector<std::string> codeArray = localCode;
                for (auto& code : codeArray) {
                    char lastCode = code.back();
                    std::string preCode = code.substr(0, code.length() - 1);
                    // 根据最后一位数字确定对应的上层网格编号
                    if (lastCode == '0') lastCode = '4';
                    else if (lastCode == '1') lastCode = '5';
                    else if (lastCode == '2') lastCode = '6';
                    else if (lastCode == '3') lastCode = '7';

                    // 添加上层网格编码
                    std::string newCode = preCode + lastCode;
                    localCode.push_back(newCode);
                }
                localCodeSet = localCode; // 更新最终网格编码集合
            }
        }

        // 子情况2.1.2：跨越两个水平网格，位于八分体边缘或退化纬度线上
        if (!isAcrossDegeneration(A, B, C, D) && oneTileQ == twoTileQ) {
            if ((oneTileRCH.row == twoTileRCH.row && oneTileLatLon.Lat == 0) ||
                (oneTileRCH.row == twoTileRCH.row && std::find(degEdge.begin(), degEdge.end(), oneTileLatLon.Lat) != degEdge.end())) {
                std::cout << "case3.2: Spanning two horizontal grids at the edge of octants or degenerate latitude line" << std::endl;

                // 调整行列号（向北扩展一行）
                IJH newOneTile = { oneTileRCH.row - 1, oneTileRCH.column, oneTileRCH.layer };
                IJH newTwoTile = { twoTileRCH.row - 1, twoTileRCH.column, twoTileRCH.layer };

                // 根据更新后的行列号计算新的网格编码
                uint64_t newOneMorton = mortonEncode_3D_LUT(newOneTile.row, newOneTile.column, newOneTile.layer);
                uint64_t newTwoMorton = mortonEncode_3D_LUT(newTwoTile.row, newTwoTile.column, newTwoTile.layer);

                std::string newOneTileCode = oneTileQ + std::to_string(newOneMorton);
                std::string newTwoTileCode = twoTileQ + std::to_string(newTwoMorton);

                // 在编码列表开头添加新计算的网格编码
                localCode.insert(localCode.begin(), newOneTileCode);
                localCode.insert(localCode.begin(), newTwoTileCode);

                // 将4个网格编码扩展为8个网格编码
                std::vector<std::string> codeArray = localCode;
                for (const auto& code : codeArray) {
                    char lastCode = code.back();
                    std::string preCode = code.substr(0, code.length() - 1);

                    // 根据最后一位数字确定对应的上层网格编号
                    if (lastCode == '0') lastCode = '4';
                    else if (lastCode == '1') lastCode = '5';
                    else if (lastCode == '2') lastCode = '6';
                    else if (lastCode == '3') lastCode = '7';

                    localCode.push_back(preCode + lastCode);
                }

                localCodeSet = localCode; // 更新最终网格编码集合
            }
        }

        // 子情况2.1.3：跨越两个垂直网格（未实现）
        // 子情况2.1.4：跨越两个垂直网格，位于八分体右边缘（未实现）
        // 子情况2.2：跨越退化边界的两个网格（未实现）
        // 子情况2.3：跨越八分体的两个网格（未实现）
    }

    // 情况3：研究区域跨越三个网格（未实现）

    // 情况4：研究区域跨越四个网格
    if (localCode.size() == 4)
    {
        // 将4个网格编码扩展为8个网格编码
        std::vector<std::string> codeArray = localCode; // 复制原始编码

        for (const auto& code : codeArray)
        {
            char lastCode = code.back();
            std::string preCode = code.substr(0, code.length() - 1);

            // 根据最后一位数字确定对应的上层网格编号
            if (lastCode == '0') lastCode = '4';
            else if (lastCode == '1') lastCode = '5';
            else if (lastCode == '2') lastCode = '6';
            else if (lastCode == '3') lastCode = '7';

            // 添加上层网格编码
            std::string newCode = preCode + lastCode;
            localCode.push_back(newCode);
        }
        localCodeSet = localCode; // 更新最终网格编码集合
    }

    // 返回包含研究区域的全局基础网格编码集合
    return localCodeSet;
}


/**
 * 确定局部剖分基础网格边界坐标
 * @param {*} A
 * @param {*} B
 * @param {*} C
 * @param {*} D
 * @param {*} level
 * @param {*} height
 * @returns 局部基础格网的边界坐标
 */
BaseTile localGrid(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D, int level, double height)
{
    BaseTile BaseTile;
    std::vector<std::string> baseGridCode = baseGridIncludeGlobalCode(A, B, C, D, level, height);

    std::vector<double> baseWest, baseSouth, baseEast, baseNorth, baseTop, baseBottom;

    for (const auto& code : baseGridCode) {
        Gridbox gridbox = Codes2Gridbox(code);
        baseWest.push_back(gridbox.west);
        baseSouth.push_back(gridbox.south);
        baseEast.push_back(gridbox.east);
        baseNorth.push_back(gridbox.north);
        baseTop.push_back(gridbox.top);
        baseBottom.push_back(gridbox.bottom);
    }
    if ((baseGridCode.size() == 8 && isAcrossDegeneration(A, B, C, D)) ||
        (baseGridCode.size() == 8 && isAcrossDegeneration(A, B, C, D) &&
            isAcrossOctant(A, B, C, D, level, height))) {
        // 取非退化线的边界作为基础格网东西边界
        BaseTile.west = baseWest[2];
        BaseTile.south = *std::min_element(baseSouth.begin(), baseSouth.end());
        BaseTile.east = baseEast[3];
        BaseTile.north = *std::max_element(baseNorth.begin(), baseNorth.end());
        BaseTile.bottom = *std::min_element(baseBottom.begin(), baseBottom.end());
        BaseTile.top = *std::max_element(baseTop.begin(), baseTop.end());
    }
    else {
        BaseTile.west = *std::min_element(baseWest.begin(), baseWest.end());
        BaseTile.south = *std::min_element(baseSouth.begin(), baseSouth.end());
        BaseTile.east = *std::max_element(baseEast.begin(), baseEast.end());
        BaseTile.north = *std::max_element(baseNorth.begin(), baseNorth.end());
        BaseTile.bottom = *std::min_element(baseBottom.begin(), baseBottom.end());
        BaseTile.top = *std::max_element(baseTop.begin(), baseTop.end());

        if (baseWest[0] * baseWest[2] < 0) {
            BaseTile.west = -(360 - *std::max_element(baseWest.begin(), baseWest.end()));
            BaseTile.east = *std::min_element(baseEast.begin(), baseEast.end());
        }
    }
    return BaseTile;
}


/**
 * @brief 获取基础网格(BaseTile)
 *
 * 该函数通过四个地理坐标点确定一个合适的基础瓦片，用于后续的局部网格计算。
 * 首先计算适合输入点的网格层级，然后验证层级有效性，最后调用 localGrid 函数生成基础瓦片。
 *
 * @param A 第一个地理坐标点(经度和纬度)
 * @param B 第二个地理坐标点(经度和纬度)
 * @param C 第三个地理坐标点(经度和纬度)
 * @param D 第四个地理坐标点(经度和纬度)
 * @return BaseTile 包含西、南、东、北边界以及顶部和底部高度的基础瓦片结构体
 * @throws std::runtime_error 当计算得到的层级无效时抛出异常
 */
BaseTile getBaseTile(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D) {
    // 计算适合输入点的网格层级
    int level = identityLevel(A, B, C, D);

    // 验证层级是否在有效范围内(0-22)
    if (level < 0 || level > 22) {
        throw std::runtime_error("[getBaseTile]Invalid identity level computed for input points.");
    }

    // 调用 localGrid 函数生成基础瓦片，高度参数设为0.0
    return localGrid(A, B, C, D, level, 0.0);
}




/**
 * 已知局部三维空间坐标，求局部的行列层号
 * @param {*} level 局部格网剖分等级
 * @param {*} longitude 经度
 * @param {*} latitude 纬度
 * @param {*} height 高度
 * @returns 局部格网的行列层号
 */
    //IJH localRowColHeiNumber(int level, double longitude, double latitude, double height, const BaseTile& baseTile) {
    //    IJH rchNumber;

    //    // 计算纬差
    //    double LDV = (baseTile.north - baseTile.south) / std::pow(2, level);
    //    // 计算行号
    //    rchNumber.row = static_cast<long long>(std::floor((baseTile.north - latitude) / LDV));

    //    // 计算经差
    //    double LOV = (baseTile.east - baseTile.west) / std::pow(2, level);
    //    // 计算列号
    //    rchNumber.column = static_cast<long long>(std::floor((longitude - baseTile.west) / LOV));

    //    // 跨越180度经线的情况
    //    if (baseTile.west < -180 && longitude > 0) {
    //        rchNumber.column = static_cast<long long>(std::floor(((longitude - baseTile.west) - 360) / LOV));
    //    }

    //    // 计算高差
    //    double HDV = baseTile.top / std::pow(2, level);
    //    rchNumber.layer = static_cast<long long>(std::floor(height / HDV));

    //    return rchNumber;
    //}

// 已知局部三维空间坐标，求局部的行列层号
IJH localRowColHeiNumber(uint8_t level, double longitude, double latitude, double height, const BaseTile& baseTile) {
    IJH rchNumber;

    // 处理纬度部分（LDV：Latitude Division Value）
    double LDV = (baseTile.north - baseTile.south) / std::pow(2.0, level);
    rchNumber.row = static_cast<uint32_t>(std::floor((baseTile.north - latitude) / LDV));

    // 经度部分（LOV：Longitude Division Value）
    double LOV = (baseTile.east - baseTile.west) / std::pow(2.0, level);
    rchNumber.column = static_cast<uint32_t>(std::floor((longitude - baseTile.west) / LOV));

    // 与 JS 保持一致：如果 west < -180 且 longitude > 0，说明瓦片跨越180°
    if (baseTile.west < -180.0 && longitude > 0.0) {
        rchNumber.column = static_cast<uint32_t>(std::floor((longitude - baseTile.west - 360.0) / LOV));
    }

    double HDV = 78125.0 / std::pow(2.0, level);

    // 【引入防线下溢】：计算相对于底部的物理高度
    double relativeHeight = height - baseTile.bottom;

    // 核心防线：绝对不允许相对高度小于 0，防止强转 uint32_t 时变成 42亿导致崩溃
    if (relativeHeight < 0.0) {
        relativeHeight = 0.0;
    }

    rchNumber.layer = static_cast<uint32_t>(std::floor(relativeHeight / HDV));

    // 限制最大层号
    if (rchNumber.layer >= (1u << level)) {
        rchNumber.layer = (1u << level) - 1;
    }
    return rchNumber;
}


/**
 * 已知局部三维坐标获取网格编码
 * @param {string} code 局部格网编码
 * @returns 局部格网的行列层号
 **/
string getLocalCode(uint8_t level, double longitude, double latitude, double height, const BaseTile& baseTile)
{
    IJH ijh = localRowColHeiNumber(level, longitude, latitude, height, baseTile);

    uint64_t localCode = mortonEncode_3D_LUT(ijh.row, ijh.column, ijh.layer);

    // 转换二进制编码为八进制
    string octalCode = toOctalString(localCode);
    // 进行补0操作
    // 进行补0操作
    if (octalCode.length() != static_cast<size_t>(level)) {
        octalCode.insert(0, level - octalCode.length(), '0'); // 补0操作
    }
    return octalCode;
}




    /**
     * 已知局部网格编码获取行列号
     * @param {string} code 局部格网编码
     * @returns 局部格网的行列层号
     **/
    IJH getLocalTileRHC(const string& code) {
        // 从八进制字符串解析为十进制整数
        long long morton = stoll(code, nullptr, 8);
        return Morto2IJH(morton);
    }

    /**
     * 已知网格编码获取网格的经纬高度
     * @param {string} code 局部格网编码
     * @returns 局部网格的经纬高度
     */
    LatLonHei getLocalTileLatLon(const string& code, const BaseTile& baseTile) {
        IJH objRowColHei = getLocalTileRHC(code);
        int level = code.length();

        // 提取行列高
        long long row = static_cast<long long>(objRowColHei.row);
        long long col = static_cast<long long>(objRowColHei.column);
        long long hei = static_cast<long long>(objRowColHei.layer);

        LatLonHei latLonHeiObj;
        // 计算纬差
        double LDV = (baseTile.north - baseTile.south) / pow(2, level);
        // 计算经差
        double LOV = (baseTile.east - baseTile.west) / pow(2, level);
        // 计算高差
    double HDV = 78125.0 / std::pow(2.0, level);

        latLonHeiObj.latitude = baseTile.north - (row + 0.5) * LDV;
        latLonHeiObj.longitude = (col + 0.5) * LOV + baseTile.west;

        // 处理跨越180度经线的情况
        if (latLonHeiObj.longitude < -180) {
            latLonHeiObj.longitude += 360;
        }

        latLonHeiObj.height = baseTile.bottom+(hei + 0.5) * HDV;
        latLonHeiObj.west = col * LOV + baseTile.west;

        // 处理跨越180度经线的情况
        if (latLonHeiObj.west < -180) {
            latLonHeiObj.west += 360;
        }

        latLonHeiObj.south = baseTile.north - (row + 1) * LDV;
        latLonHeiObj.east = (col + 1) * LOV + baseTile.west;

        // 处理跨越180度经线的情况
        if (latLonHeiObj.east <= -180) {
            latLonHeiObj.east += 360;
        }

        latLonHeiObj.north = baseTile.north - row * LDV;
        latLonHeiObj.bottom = baseTile.bottom+hei * HDV;
        latLonHeiObj.top = baseTile.bottom+(hei + 1) * HDV;
        latLonHeiObj.code = code;

        return latLonHeiObj;
    }

    /**
 * 已知网格编码合集获取网格的边界信息合集
 * @param {string} code 局部格网编码
 * @returns 局部网格的经纬高度
 */
    vector<LatLonHei> getLocalTilesLatLon(const vector<string>& codes, const BaseTile& baseTile) {
        vector<LatLonHei> latLonHeiList;
        latLonHeiList.reserve(codes.size()); // 预分配空间，提高性能

        for (const auto& code : codes) {
            latLonHeiList.push_back(getLocalTileLatLon(code, baseTile));
        }

        return latLonHeiList;
    }

/**
 * @brief 计算局部格网配置信息
 *
 * 该函数通过四个地理坐标点确定局部格网的完整配置信息，包括识别最佳网格层级、
 * 计算全局网格编码集合、检测是否跨越特殊边界等。这些配置信息对于后续的局部网格
 * 计算、编码生成和空间分析至关重要。
 *
 * @param A 研究区域的第一个顶点坐标(经度和纬度)
 * @param B 研究区域的第二个顶点坐标(经度和纬度)
 * @param C 研究区域的第三个顶点坐标(经度和纬度)
 * @param D 研究区域的第四个顶点坐标(经度和纬度)
 * @return LocalGridConfig 包含局部格网计算所需的所有配置参数的结构体
 *
 * @note 输入的四个点应该形成一个四边形，代表研究区域的边界
 */
LocalGridConfig localGridConfig(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D) {
    // 创建配置结构体
    LocalGridConfig config;

    // 1. 计算适合研究区域的网格识别层级
    config.idLevel = identityLevel(A, B, C, D);

    // 2. 获取包含研究区域的全局基础网格编码集合
    config.globalCodeSet = baseGridIncludeGlobalCode(A, B, C, D, config.idLevel, 0);

    // 3. 检测研究区域是否跨越退化边界
    config.isAcrossDegeneration = isAcrossDegeneration(A, B, C, D);

    // 4. 检测研究区域是否跨越八分体边界
    config.isAcrossOctant = isAcrossOctant(A, B, C, D, config.idLevel, 0);

    // 5. 收集研究区域的局部二维网格编码
    config.localCode2D = codeGather(A, B, C, D, config.idLevel, 0);

    // 返回完整配置
    return config;
}


    /**
     * 局部行列号转全局行列号
     * @param {string} localCode 局部格网编码
     * @returns 全局格网的行列层号
     */
    IJH localRCHtoGlobalRCH(const string& localCode, const LocalGridConfig& config) {
        IJH GlobalRowColHei;

        // 根据局部网格编码求局部网格的行列号
        IJH LocalRowColHei = getLocalTileRHC(localCode);

        // 求左上角网格，在全局的行列号
        long long LBgridMotrn = stoll(config.localCode2D[0].substr(1), nullptr, 8);
        IJ LBgridRowCol = Morto2IJ(LBgridMotrn);
        long long LBgridRow = LBgridRowCol.row;
        long long LBgridCol = LBgridRowCol.column;

        // 局部剖分的等级
        int localLevel = localCode.length();

        // 计算全局网格的行列号
        GlobalRowColHei.row = LocalRowColHei.row + LBgridRow * pow(2, localLevel - 1);
        GlobalRowColHei.column = floor(LocalRowColHei.column / 2) + LBgridCol * pow(2, localLevel - 1);

        // 四个网格跨退化，但是不跨八分体
        if (config.localCode2D.size() == 4 && config.isAcrossDegeneration && !config.isAcrossOctant)
        {
            GlobalRowColHei.row = floor(LocalRowColHei.row / 2) + LBgridCol * pow(2, localLevel - 1) + (localLevel - 1);
        }

        // 四个网格跨退化跨八分体
        if (config.localCode2D.size() == 4 && config.isAcrossDegeneration && config.isAcrossOctant) {
            if ((LocalRowColHei.row <= pow(2, localLevel - 1) - 1) && (LocalRowColHei.column <= pow(2, localLevel - 1) - 1)) {
                GlobalRowColHei.row = LocalRowColHei.row + LBgridRow * pow(2, localLevel - 1);
                GlobalRowColHei.column = floor(LocalRowColHei.column / 2) + LBgridCol * pow(2, localLevel - 1) + (localLevel - 1);
                GlobalRowColHei.layer = LocalRowColHei.layer;
            }
            else if ((LocalRowColHei.row <= pow(2, localLevel - 1) - 1) && (LocalRowColHei.column > pow(2, localLevel - 1) - 1)) {
                GlobalRowColHei.row = LocalRowColHei.row + LBgridRow * pow(2, localLevel - 1);
                GlobalRowColHei.column = floor(LocalRowColHei.column / 2) - (localLevel - 1);
                GlobalRowColHei.layer = LocalRowColHei.layer;

            }
        }

        GlobalRowColHei.layer = LocalRowColHei.layer; // 确保高度被设置

        return GlobalRowColHei;
    }



    /**   localcode转Globalcode
     * @param {string} localCode
     * @returns 全球格网编码
     */
    string localToGlobal(const string& localCode, const LocalGridConfig& config) {
        string globalCode; // 返回的全球网格编码
        int localLevel = localCode.length();
        // 对应的全球剖分的等级
        int globalLevel = config.localCode2D[0].length() - 1 + localLevel;

        // 研究区域处于一个格网中
        if (config.localCode2D.size() == 1) {
            globalCode = config.globalCodeSet[0] + localCode;
        }

        // 研究区域处于正常的四个网格中
        if ((config.localCode2D.size() == 4 && !config.isAcrossDegeneration) ||
            (config.localCode2D.size() == 2 && !config.isAcrossDegeneration)) {

            char parentCode = localCode[0];
            string childCode = localCode.substr(1);

            // 根据父编码选择全局编码
            if (parentCode >= '0' && parentCode <= '7') {
                long long parentCodeValue = stoll(config.globalCodeSet[parentCode - '0'], nullptr, 8);
                long long childCodeValue = stoll(childCode, nullptr, 8);

                // 计算合并后的值
                long long mergedValue = (parentCodeValue << (childCode.length() * 3)) ^ childCodeValue;

                // 转换为八进制字符串
                globalCode = toOctalString(mergedValue);
            }

            // 补0操作
            if (globalCode.length() != globalLevel) {
                globalCode = string(globalLevel - globalCode.length(), '0') + globalCode;
            }
        }

        //研究区域位于四个网格而且位于退化区域
      /*...........................................*/

        //研究区域位于四个网格且位于退化区域且跨八分体
      /*..............................................*/
        //研究区域位于三个网格或者两个网格的退化区域
       /*....................................*/
        return globalCode;
    }


    /**
     * @brief 将行列高索引转换为局部编码
     * @param obj 包含行、列、层信息的IJH结构体
     * @param level 网格层级
     * @return 局部网格编码字符串
     */
    std::string rchToCode(const IJH& obj, uint8_t level) {
        uint64_t R = static_cast<uint64_t>(obj.row);
        uint64_t C = static_cast<uint64_t>(obj.column);
        uint64_t H = static_cast<uint64_t>(obj.layer);

        uint64_t localCode_n = mortonEncode_3D_LUT(R, C, H);
        std::string localCode = toOctalString(localCode_n);

        // 进行补0操作
        if (localCode.length() != static_cast<size_t>(level)) {
            localCode = std::string(level - localCode.length(), '0') + localCode;
        }

        return localCode;
    }

/// @brief 生成带互操作标识的局部网格编码
/// @details 将区域上下文附加到局部编码，避免跨区域码值冲突
std::string toInteropLocalCode(const std::string& localCode, uint8_t level)
{
    return getProjectRegionId()+":"+localCode;
}


/// @brief 解析并校验互操作局部网格编码
/// @details 校验协议版本、区域标识、层级和八进制编码格式
bool parseInteropLocalCode(const std::string& interopCode,
                           std::string& localCode,
                           uint8_t& level,
                           std::string& error) {
    // 预期格式: LGC1:{regionId}:{level}:{localCode}
    std::vector<std::string> parts;
    std::stringstream ss(interopCode);
    std::string part;
    while (std::getline(ss, part, ':')) parts.push_back(part);

    if (parts.size() != 4 || parts[0] != "LGC1") {
        error = "invalid interop format";
        return false;
    }

    if (parts[1] != getProjectRegionId()) {
        error = "regionId mismatch";
        return false;
    }

    int lv = 0;
    try { lv = std::stoi(parts[2]); }
    catch (...) {
        error = "invalid level";
        return false;
    }

    if (lv < 1 || lv > 30) {
        error = "level out of range";
        return false;
    }

    const std::string& code = parts[3];
    if (static_cast<int>(code.size()) != lv) {
        error = "code length mismatch";
        return false;
    }

    for (char ch : code) {
        if (ch < '0' || ch > '7') {
            error = "code is not octal";
            return false;
        }
    }

    localCode = code;
    level = static_cast<uint8_t>(lv);
    error.clear();
    return true;
}




    /// @brief 空域网格化
    /// @param A
    /// @param B
    /// @param C
    /// @param D
    void newSub(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D)
    {

        // 定义目标网格
        BaseTile targetTile;
        targetTile.west = 118.55650371053235;
        targetTile.east = 118.61513154262803;
        targetTile.north = 31.164584240103924;
        targetTile.south = 31.11180404133692;
        targetTile.bottom = 0;
        targetTile.top = 300;

        int Level = identityLevel(A, B, C, D);
        BaseTile baseTile = localGrid(A, B, C, D, Level, 0);

        const uint8_t level = 11; // 测试局部level

        cout << "基础格网配置: " << baseTile.north << ", " << baseTile.south << ", "
            << baseTile.east << ", " << baseTile.west << endl;
        vector<LatLonHei> temp;
        temp.reserve(1790304);
        // 计算纬差
        double LDV = (baseTile.north - baseTile.south) / pow(2, level);

        // 计算行号
        int R_start = static_cast<int>((baseTile.north - targetTile.north) / LDV);
        int R_end = static_cast<int>((baseTile.north - targetTile.south) / LDV);

        // 计算经差
        double LOV = (baseTile.east - baseTile.west) / pow(2, level);

        // 计算列号
        int C_start = static_cast<int>((targetTile.west - baseTile.west) / LOV);
        int C_end = static_cast<int>((targetTile.east - baseTile.west) / LOV);

        // 跨越180度经线的情况
        if (baseTile.west < -180 && baseTile.east > 0) {
            C_end = static_cast<int>(((targetTile.east - baseTile.west) - 360) / LOV);
            C_start = static_cast<int>(((targetTile.west - baseTile.west) - 360) / LOV);
        }

        // 计算高差
    double HDV = 78125.0 / std::pow(2.0, level);
    double relativeBottom = targetTile.bottom - baseTile.bottom;
    if (relativeBottom < 0.0) relativeBottom = 0.0;
    int H_start = static_cast<int>(std::floor(relativeBottom / HDV));

    double relativeTop = targetTile.top - baseTile.bottom;
    if (relativeTop < 0.0) relativeTop = 0.0;
    int H_end = static_cast<int>(std::floor(relativeTop / HDV));
        for (int r = R_start; r <= R_end; r++) {
            for (int c = C_start; c <= C_end; c++) {
                for (int h = H_start; h <= H_end; h++) {
                    long long R = static_cast<long long>(r);
                    long long C = static_cast<long long>(c);
                    long long H = static_cast<long long>(h);

                    LatLonHei latLonHeiObj;
                    // 局部网格行列号转为的八进制编码
                    uint64_t localCode_n = mortonEncode_3D_LUT(r, c, h);
                    string localCode = toOctalString(localCode_n);

                    // 进行补0操作
                    if (localCode.length() != static_cast<size_t>(level)) {
                        localCode.insert(0, level - localCode.length(), '0'); // 补0操作
                    }
                    // 补充经纬高信息
                    latLonHeiObj.latitude = baseTile.north - (R + 0.5) * LDV;
                    latLonHeiObj.longitude = (C + 0.5) * LOV + baseTile.west;

                    if (latLonHeiObj.longitude < -180) {
                        latLonHeiObj.longitude += 360;
                    }

                    latLonHeiObj.height = baseTile.bottom + (H + 0.5) * HDV;

                    // 计算边界
                    latLonHeiObj.west = C * LOV + baseTile.west;
                    if (latLonHeiObj.west < -180) {
                        latLonHeiObj.west += 360;
                    }

                    latLonHeiObj.south = baseTile.north - (R + 1) * LDV;
                    latLonHeiObj.east = (C + 1) * LOV + baseTile.west;
                    if (latLonHeiObj.east <= -180) {
                        latLonHeiObj.east += 360;
                    }

                    latLonHeiObj.north = baseTile.north - R * LDV;
                    latLonHeiObj.bottom = baseTile.bottom + H * HDV;
                    latLonHeiObj.top = baseTile.bottom + (H + 1) * HDV;
                    latLonHeiObj.code = localCode;

                    temp.push_back(latLonHeiObj);
                }
            }
        }
        // 输出调试信息
        cout << "生成的网格数据数量: " << temp.size() << endl;
    }


    /// @brief 根据研究区格网行列号求格网坐标
    /// @param ijh
    /// @param level
    /// @param baseTile
    /// @return

    LatLonHei IJHToLocalTileLatLon(const IJH& ijh, uint32_t level, const BaseTile& baseTile) {
        LatLonHei latLonHeiObj;

        // 计算纬度、经度、高度的差值
        double LDV = (baseTile.north - baseTile.south) / pow(2, level);
        double LOV = (baseTile.east - baseTile.west) / pow(2, level);
    double HDV = 78125.0 / std::pow(2.0, level);

        // 计算中心点坐标
        latLonHeiObj.latitude = baseTile.north - (ijh.row + 0.5) * LDV;
        latLonHeiObj.longitude = (ijh.column + 0.5) * LOV + baseTile.west;
        if (latLonHeiObj.longitude < -180) latLonHeiObj.longitude += 360;

        latLonHeiObj.height = baseTile.bottom + (ijh.layer + 0.5) * HDV;

        // 计算格网的四个边界
        latLonHeiObj.west = ijh.column * LOV + baseTile.west;
        if (latLonHeiObj.west < -180) latLonHeiObj.west += 360;

        latLonHeiObj.south = baseTile.north - (ijh.row + 1) * LDV;
        latLonHeiObj.east = (ijh.column + 1) * LOV + baseTile.west;
        if (latLonHeiObj.east <= -180) latLonHeiObj.east += 360;

        latLonHeiObj.north = baseTile.north - ijh.row * LDV;
        latLonHeiObj.bottom = baseTile.bottom + ijh.layer * HDV;
        latLonHeiObj.top = baseTile.bottom + (ijh.layer + 1) * HDV;

        return latLonHeiObj;
    }

/**
 * @brief 将输入的编码集聚合到多尺度编码集
 *
 * 该函数实现了八叉树编码的聚合功能：当一个父节点的所有8个子节点都存在时，
 * 会将这些子节点替换为父节点，从而实现编码的粗化（向上聚合）。
 * 聚合过程会迭代进行，直到无法再进行聚合为止，同时确保聚合后的编码不会低于指定的最小层级。
 *
 * @param inputCodes 输入的编码集合
 * @param minLevel 允许的最小编码层级（编码字符串的最小长度）
 * @return 聚合后的多尺度编码集合
 */
std::unordered_set<std::string> aggregateToMultiScaleCodes(
    const std::unordered_set<std::string>& inputCodes,
    int minLevel)
{
    if (inputCodes.empty()) {
        return {};
    }

    std::unordered_set<std::string> currentCodes = inputCodes;
    bool aggregated = true;
    constexpr size_t kFullOctreeChildren = 8;  // 八叉树节点的子节点总数

    while (aggregated) {
        aggregated = false;

        // 构建父编码到子编码的映射
        std::unordered_map<std::string, std::unordered_set<std::string>> parentToChildren;
        for (const auto& code : currentCodes) {
            // 跳过低于或等于最小层级的编码
            if (static_cast<int>(code.length()) <= minLevel) {
                continue;
            }
            // 使用 getLevelFatherCode 获取直接父网格编码
            // level = code.length() - 2，因为 code.length() - 1 是当前层级
            std::string parentCode = getLevelFatherCode(code, static_cast<uint8_t>(code.length() - 2));
            if (!parentCode.empty() && parentCode != code) {
                parentToChildren[parentCode].insert(code);
            }
        }

        std::unordered_set<std::string> codesToRemove;  // 待移除的子编码
        std::unordered_set<std::string> codesToAdd;     // 待添加的父编码

        // 检查每个父节点是否拥有所有8个子节点
        for (const auto& [parentCode, children] : parentToChildren) {
            if (children.size() == kFullOctreeChildren) {
                // 如果是，标记所有子节点为移除，并将父节点标记为添加
                for (const auto& child : children) {
                    codesToRemove.insert(child);
                }
                codesToAdd.insert(parentCode);
                aggregated = true;  // 标记本轮有聚合发生
            }
        }

        // 执行编码的移除和添加操作
        for (const auto& code : codesToRemove) {
            currentCodes.erase(code);
        }
        for (const auto& code : codesToAdd) {
            currentCodes.insert(code);
        }
    }

    return currentCodes;
}

/**
 * @brief 将输入的编码向量聚合到多尺度编码向量（重载版本）
 *
 * 这是一个便捷的重载版本，接受向量作为输入，内部将其转换为无序集合后调用
 * 集合版本的aggregateToMultiScaleCodes函数，最后再将结果转换回向量返回。
 *
 * @param inputCodes 输入的编码向量
 * @param minLevel 允许的最小编码层级（编码字符串的最小长度）
 * @return 聚合后的多尺度编码向量
 */
std::vector<std::string> aggregateToMultiScaleCodes(
    const std::vector<std::string>& inputCodes,
    int minLevel)
{
    std::unordered_set<std::string> codeSet(inputCodes.begin(), inputCodes.end());
    std::unordered_set<std::string> resultSet = aggregateToMultiScaleCodes(codeSet, minLevel);
    return std::vector<std::string>(resultSet.begin(), resultSet.end());
}
