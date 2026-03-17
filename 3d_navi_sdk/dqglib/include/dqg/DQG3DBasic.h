#ifndef DQG_3D_BASIC_HPP
#define DQG_3D_BASIC_HPP

#include <dqg/GeoParam.h>
#include <dqg/DQGMathBasic.h>
#include <unordered_set>
#include <unordered_map>

#define LEVEL 9

/// @brief 3D 网格编码
/// @return 网格编码字符串
std::string LBH2DQG_str(double l, double b, double hei, uint8_t level);
std::string IJH2DQG_str(uint32_t row, uint32_t col, uint32_t layer, uint8_t level);
/// @brief 3D 网格解码
PointLBHd Decode_3D(const std::string& code);

/// @brief 3D 网格解码存入包围盒
Gridbox Codes2Gridbox(const std::string& code);

/// @brief 获取指定级别的父网格编码
std::string getLevelFatherCode(const std::string& code, uint8_t level);

/// @brief 获取所有子网格编码
std::vector<std::string> getChildCode(const std::string& code);

/// @brief 计算局部网格的配置
LocalGridConfig localGridConfig(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D);

/// @brief 角度转换为弧度
inline double toRadians(double degrees) {
    return degrees * PI / 180.0;
}

/// @brief 弧度转换为角度
inline double toDegrees(double radians) {
    return radians * 180.0 / PI;
}

/// @brief 计算 IJH 点间的欧几里得距离
double distance(const IJH& point1, const IJH& point2);

/// @brief 计算二维平面上两点间的直线距离
double distance(double x, double y);

/// @brief 计算三维欧氏距离
inline double distance3D(const PointLBHd& p1, const PointLBHd& p2) {
    double dLng = (p2.Lng - p1.Lng) * R_E * cos(toRadians(p1.Lat)) * PI / 180.0;
    double dLat = (p2.Lat - p1.Lat) * R_E * PI / 180.0;
    double dHgt = p2.Hgt - p1.Hgt;
    return sqrt(dLng * dLng + dLat * dLat + dHgt * dHgt);
}

/// @brief 计算两点间的球面距离（Haversine 公式）
inline double haversineDistance(const PointLBHd& p1, const PointLBHd& p2) {
    double dLat = toRadians(p2.Lat - p1.Lat);
    double dLng = toRadians(p2.Lng - p1.Lng);

    double a = sin(dLat / 2) * sin(dLat / 2) +
        cos(toRadians(p1.Lat)) * cos(toRadians(p2.Lat)) * sin(dLng / 2) * sin(dLng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return WGS84a * c;
}

/// @brief 计算二维平面上两点间的欧几里得距离
inline double euclideanDistance2D(double x, double y) {
    return sqrt(x * x + y * y);
}

/// @brief 检测三个点是否共线
bool collinear_3Points(const IJH& point1, const IJH& point2, const IJH& point3);

/// @brief 计算方位角
double azimuth(double x, double y);

/// @brief 计算某个点沿特定角度扩展一定距离的点
PointLBHd movePoint(const PointLBHd& p, double distance, double angle);

/// @brief 计算多边形的缓冲区
std::vector<PointLBHd> turfBuffer(const std::vector<PointLBHd>& polygon, double radius);

/// @brief 计算 3D 线段经过的 DQG 编码
std::vector<std::string> senham3D_DQG(const PointLBHd& point1, const PointLBHd& point2, uint8_t level);

/// @brief 识别研究区的最佳网格级别
uint8_t identityLevel(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D);

/// @brief 判断是否跨越八分体
bool isAcrossOctant(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D, int level, double height);

/// @brief 判断是否跨越退化边界
bool isAcrossDegeneration(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D);

/// @brief 八进制编码转换为字符串
std::string toOctalString(uint64_t number);


/// @brief 计算局部网格索引
IJH localRowColHeiNumber(uint8_t level, double longitude, double latitude, double height, const BaseTile& baseTile);

/// @brief 计算局部网格编码
std::string getLocalCode(uint8_t level, double longitude, double latitude, double height, const BaseTile& baseTile);

/// @brief 计算局部网格的行列高
IJH getLocalTileRHC(const std::string& code);

/// @brief 确定局部剖分基础网格边界坐标
BaseTile localGrid(
    const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D, int level, double height);

BaseTile getBaseTile(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D);


/// @brief 计算局部网格的边界信息
LatLonHei getLocalTileLatLon(const std::string& code, const BaseTile& baseTile);
/// @brief 计算局部网格的边界信息合集
vector<LatLonHei> getLocalTilesLatLon(const vector<string>& codes, const BaseTile& baseTile);
/// @brief 局部网格索引转换为全局网格索引
IJH localRCHtoGlobalRCH(const std::string& localCode, const LocalGridConfig& config);

/// @brief 局部编码转换为全局编码
std::string localToGlobal(const std::string& localCode, const LocalGridConfig& config);

/// @brief 计算网格区域中心点的经纬度
LatLonHei IJHToLocalTileLatLon(const IJH& ijh, uint32_t level, const BaseTile& baseTile);

/// @brief 将行列高索引转换为局部编码
std::string rchToCode(const IJH& obj, uint8_t level);

/// @brief 多尺度网格编码聚合（unordered_set版本）
/// @param inputCodes 输入的网格编码集合
/// @param minLevel 最小聚合层级，编码长度小于等于此值的不再向上聚合（默认为1）
/// @return 聚合后的多尺度网格编码集合
/// @details 递归判断父格网的子格网数量是否等于8，若是则聚合为父格网编码，
///          直到没有任何父格网的子格网数量达到8为止
std::unordered_set<std::string> aggregateToMultiScaleCodes(
    const std::unordered_set<std::string>& inputCodes,
    int minLevel = 1);

/// @brief 多尺度网格编码聚合（vector版本）
/// @param inputCodes 输入的网格编码向量
/// @param minLevel 最小聚合层级，编码长度小于等于此值的不再向上聚合（默认为1）
/// @return 聚合后的多尺度网格编码向量
std::vector<std::string> aggregateToMultiScaleCodes(
    const std::vector<std::string>& inputCodes,
    int minLevel = 1);

#endif
