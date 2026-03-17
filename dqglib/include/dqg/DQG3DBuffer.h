
#include <dqg/DQG3DPolygon.h>
#include <stdexcept>



/// @brief 计算最小外接正方体的经纬高范围
BaseTile getBoundingBox(double lon, double lat, double alt, double R);

/// @brief 3D点缓冲区网格
vector<vector<LatLonHei>> getPointsBuffer(const vector<PointLBHd>& points,
    uint8_t level,
    double radius,
    const BaseTile& basetile);

/// @brief  计算两点连线与指定平面的交点
PointLBHd calculateIntersection(const PointLBHd& A, const PointLBHd& B, double planeValue, const string& flag);



/// @brief 生成线段与网格的交点列表
void generateGridIntersections(const PointLBHd& p1, const PointLBHd& p2,
    const BaseTile& startGrid, double diff,
    const string& flag, vector<double>& intersections);

/// @brief  计算线段与网格交点，生成插值点
vector<PointLBHd> lineInsertPoints(const PointLBHd& p1, const PointLBHd& p2, uint8_t level, const BaseTile& baseTile);

/// @brief 计算网格编码，返回所有涉及的网格点

vector<LatLonHei> lineInsertPoints(const vector<PointLBHd>& eachLine, double radius, uint8_t level);


/// @brief 线缓冲区生成

vector<vector<LatLonHei>> getLinesBuffer(const vector<vector<PointLBHd>>& lineReq,
    double radius, uint8_t level);



///@brief 获取多边形缓冲区坐标
vector<vector<PointLBHd>> getBufferPoints(const vector<vector<PointLBHd>>& polygons, double radius);


///@brief 计算多边形缓冲区网格
pair<vector<vector<Gridbox>>, vector<vector<Gridbox>>>
getPolygonsBuffer(uint8_t level, double top, double bottom,
    const vector<vector<PointLBHd>>& s,
    double radius, const BaseTile& baseTile);

/// @brief 三维线缓冲区生成算法
/// 由于线的截面矩形所在的坐标系和网格所在的坐标系不一致的问题，从而不能简单使用网格领域扩充来计算
/// 方法：构建局部投影坐标，求出截面矩形的四个角点坐标，并且求其最小包围盒，将最小包围盒进行网格填充
/// @param lineReq 三维线上点坐标序列（经、纬、高）
/// @param halfWidth 截面矩形的宽度
/// @param halfHeight 截面矩形的高度
/// @param level 层级
/// @param baseTile 基准瓦片
/// @return 三维线缓冲区网格编码序列
vector<string> lineBufferFilled(const vector<PointLBHd>& lineReq, double halfWidth, double halfHeight, 
                                uint8_t level, const BaseTile& baseTile);