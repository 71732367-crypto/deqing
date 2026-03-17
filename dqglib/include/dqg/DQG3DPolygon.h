#ifndef DQG_3D_POLYGON_H
#define DQG_3D_POLYGON_H

#include  <dqg/DQG3DBasic.h>
#include <optional>
#include <thread>
#include <future>
#include <algorithm>



struct Point {
    int x, y;
    double z;
};

struct Edge {
    double x;
    int ymin, ymax;
    double dx;
};


// 扫描线边结构体
struct ScanEdge {
    double x;        // 当前扫描线交点X坐标（对应网格列号）
    int ymin;       // 边起始Y坐标（对应网格行号）
    double dx;      // X方向增量（1/斜率）
    int ymax;       // 边结束Y坐标

    ScanEdge(double x_, int ymin_, double dx_, int ymax_)
        : x(x_), ymin(ymin_), dx(dx_), ymax(ymax_) {
    }
};

// 网格填充结果结构体
struct PolygonFillResult {
    std::vector<Gridbox> filledGrids;  // 填充网格集合
    std::vector<IJ> edgeGrids;         // 边界网格集合
};

inline int getMinY(const std::vector<Point>& vertices) {
    int minY = vertices[0].y;
    for (const auto& p : vertices) {
        if (p.y < minY) minY = p.y;
    }
    return minY;
}

inline int getMaxY(const std::vector<Point>& vertices) {
    int maxY = vertices[0].y;
    for (const auto& p : vertices) {
        if (p.y > maxY) maxY = p.y;
    }
    return maxY;
}


/// @brief 单个多边形面网格化算法
vector<Point> polygonFill(const vector<Point>& vertices);


/// @brief 多边形扫描线算法
PolygonFillResult scanLineFill(
    const vector<IJ>& polygon,    // 多边形顶点（网格行列号）
    uint8_t level,                    // 网格层级
    uint32_t layer,                   // 高度层
    const BaseTile& baseTile           // 基准瓦片信息
);

/// @brief 多个多边形面网格化算法
vector<vector<Gridbox>> getMultiplePolygonGrids(
    uint8_t level,
    const vector<vector<PointLBHd>>& boundaries,
    double top,
    double bottom,
    const BaseTile& baseTile);

/// @brief 立方体区域网格化核心计算函数（支持多线程）
/// @param west 区域西边界
/// @param east 区域东边界  
/// @param north 区域北边界
/// @param south 区域南边界
/// @param bottom 区域底部高度
/// @param top 区域顶部高度
/// @param level 网格精度级别
/// @param baseTile 基准瓦片，用于计算网格分辨率
/// @return std::optional<std::vector<Gridbox>> 成功时返回网格单元向量，失败时返回std::nullopt
std::optional<std::vector<Gridbox>> gridCubeRegion(double west, double east, double north, double south, 
                                                   double bottom, double top, int level, const BaseTile& baseTile);

/// @brief 从 JSON 格式的多边形顶点（[[lon,lat],...]) 生成网格编码集合（立体填充）
std::vector<std::string> getPolygonGridCodes(
    const std::string& polygonJson,
    double top,
    double bottom,
    uint8_t level,
    const BaseTile& baseTile);

/// @brief 从 JSON 格式的多边形顶点（[[lon,lat],...]) 生成仅外表面的网格编码集合（上/下/侧面）
std::vector<std::string> getPolygonSurfaceGridCodes(
    const std::string& polygonJson,
    double top,
    double bottom,
    uint8_t level,
    const BaseTile& baseTile);
#endif // DQG_3D_POLYGON_H
