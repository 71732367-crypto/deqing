#include <dqg/DQG3DPolygon.h>
#include <dqg/DQG3DBasic.h>
#include <stdint.h>
#include <future>
#include <mutex>
#include <algorithm>
#include <vector>
#include <unordered_set>

using namespace std;
/// @brief 单个多边形面网格化算法
/// @param vertices 
/// @return 
vector<Point> polygonFill(const vector<Point>& vertices) {
    vector<Point> pts;
    if (vertices.size() < 3) return pts;
    
    int minY = getMinY(vertices);
    int maxY = getMaxY(vertices);
    double z = vertices[0].z;
    
    vector<vector<Edge>> edgeTable(maxY - minY + 1);
    
    int n = vertices.size();
    for (int i = 0; i < n; ++i) {
        Point p0 = vertices[i];
        Point p1 = vertices[(i + 1) % n];
        
        if (p0.y == p1.y) { 
            int startX = min(p0.x, p1.x);
            int endX = max(p0.x, p1.x);
            for (int x = startX; x <= endX; ++x) {
                pts.push_back({x, p0.y, z});
            }
        } else {
            if (p0.y > p1.y) swap(p0, p1);
            double dx = static_cast<double>(p1.x - p0.x) / (p1.y - p0.y);
            edgeTable[p0.y - minY].push_back({static_cast<double>(p0.x), p0.y, p1.y - 1, dx});
        }
    }
    
    vector<Edge> activeEdgeTable;
    for (int y = minY; y <= maxY; ++y) {
        for (const auto& edge : edgeTable[y - minY]) {
            activeEdgeTable.push_back(edge);
        }
        
        sort(activeEdgeTable.begin(), activeEdgeTable.end(), [](const Edge& a, const Edge& b) {
            return a.x < b.x;
        });
        
        for (size_t i = 0; i < activeEdgeTable.size(); i += 2) {
            int startX = round(activeEdgeTable[i].x);
            int endX = round(activeEdgeTable[i + 1].x);
            for (int x = startX; x <= endX; ++x) {
                pts.push_back({x, y, z});
            }
        }
        
        for (auto& edge : activeEdgeTable) {
            edge.x += edge.dx;
        }
        
        activeEdgeTable.erase(remove_if(activeEdgeTable.begin(), activeEdgeTable.end(), [y](const Edge& edge) {
            return edge.ymax <= y;
        }), activeEdgeTable.end());
    }
    
    return pts;
}

/// @brief 多边形扫描线算法
/// @param polygon 
/// @param level 
/// @param layer 
/// @param baseTile 
/// @return 
PolygonFillResult scanLineFill(
    const vector<IJ>& polygon,    // 多边形顶点（网格行列号）
    uint8_t level,                // 网格层级
    uint32_t layer,               // 高度层
    const BaseTile& baseTile       // 基准瓦片信息
) {
    PolygonFillResult result;
    if (polygon.size() < 3) return result;

    auto [minRow, maxRow] = minmax_element(
        polygon.begin(), polygon.end(),
        [](const IJ& a, const IJ& b) { return a.row < b.row; });
    int yMin = minRow->row;
    int yMax = maxRow->row;

    vector<vector<ScanEdge>> edgeTable(yMax - yMin + 1);
    vector<ScanEdge> activeEdges;

    for (size_t i = 0; i < polygon.size(); ++i) {
        const IJ& p0 = polygon[i];
        const IJ& p1 = polygon[(i + 1) % polygon.size()];

        if (p0.row == p1.row) {
            int startCol = min(p0.column, p1.column);
            int endCol = max(p0.column, p1.column);
            for (int col = startCol; col <= endCol; ++col) {
                result.edgeGrids.push_back({static_cast<uint32_t>(col), p0.row});
            }
            continue;
        }

        const IJ& pStart = (p0.row < p1.row) ? p0 : p1;
        const IJ& pEnd = (p0.row < p1.row) ? p1 : p0;

        double dx = static_cast<double>(pEnd.column - pStart.column) / 
                   (pEnd.row - pStart.row);
        int ymin = pStart.row;
        int ymax = pEnd.row;

        if (ymin - yMin >= 0 && ymin - yMin < edgeTable.size()) {
            edgeTable[ymin - yMin].emplace_back(
                pStart.column, ymin, dx, ymax);
        }
    }

    for (int y = yMin; y <= yMax; ++y) {
        if (y - yMin < edgeTable.size()) {
            activeEdges.insert(activeEdges.end(), 
                edgeTable[y - yMin].begin(), edgeTable[y - yMin].end());
        }

        sort(activeEdges.begin(), activeEdges.end(),
            [](const ScanEdge& a, const ScanEdge& b) { return a.x < b.x; });

        for (size_t i = 0; i < activeEdges.size(); i += 2) {
            if (i + 1 >= activeEdges.size()) break;

            int startCol = static_cast<int>(round(activeEdges[i].x));
            int endCol = static_cast<int>(round(activeEdges[i + 1].x));

            for (int col = startCol; col <= endCol; ++col) {
                Gridbox g;
                g.level = level;
                g.layer = layer;
                g.row = y;
                g.column = col;

                g.Lng = baseTile.west + (col * (baseTile.east - baseTile.west)) / (1 << level);
                g.Lat = baseTile.north - (y * (baseTile.north - baseTile.south)) / (1 << level);

                result.filledGrids.push_back(g);
            }
        }

        for (auto& edge : activeEdges) edge.x += edge.dx;
        activeEdges.erase(
            remove_if(activeEdges.begin(), activeEdges.end(),
                [y](const ScanEdge& e) { return e.ymax <= y; }),
            activeEdges.end());
    }

    return result;
}


/// @brief 多个多边形面网格化算法
/// @param level 
/// @param boundaries 
/// @param top 
/// @param bottom 
/// @param baseTile 
/// @return 
vector<vector<Gridbox>> getMultiplePolygonGrids(
    uint8_t level,
    const vector<vector<PointLBHd>>& boundaries,
    double top,
    double bottom,
    const BaseTile& baseTile)
{
    vector<vector<Gridbox>> results;
    for (const auto& polygon : boundaries) {
        vector<IJ> gridVertices;
        uint32_t maxLayer = 0;
        // 顶点坐标转换
        for (const auto& point : polygon) {
            IJH bottomIJH = localRowColHeiNumber(level, point.Lng, point.Lat, bottom, baseTile);
            gridVertices.push_back({bottomIJH.row, bottomIJH.column});
            
            IJH topIJH = localRowColHeiNumber(level, point.Lng, point.Lat, top, baseTile);
            maxLayer = max(maxLayer, topIJH.layer);
        }
        // 调用扫描线填充
        auto fillResult = scanLineFill(gridVertices, level, HEIGHT,baseTile);
        // 生成三维网格
        unordered_set<string> uniqueCodes;
        vector<Gridbox> currentGrids;

        for (const auto& grid : fillResult.filledGrids) {
            IJH bottomIJH = localRowColHeiNumber(level, grid.Lng, grid.Lat, bottom, baseTile);
            IJH topIJH = localRowColHeiNumber(level, grid.Lng, grid.Lat, top, baseTile);

            for (uint32_t h = bottomIJH.layer; h <= topIJH.layer; ++h) {
                IJH currentIJH{grid.row, grid.column, h};
                // 使用行列号生成局部编码
                string code = IJH2DQG_str(currentIJH.row, currentIJH.column, currentIJH.layer, level);
                
                if (uniqueCodes.insert(code).second) {
                    Gridbox g;
                    g.level = level;
                    g.code = code;
                    
                    // 使用已有坐标转换接口
                    LatLonHei geo = IJHToLocalTileLatLon(currentIJH, level, baseTile);
                    
                    g.west = geo.west;
                    g.east = geo.east;
                    g.north = geo.north;
                    g.south = geo.south;
                    g.bottom = geo.bottom;
                    g.top = geo.top;
                    
                    g.row = currentIJH.row;
                    g.column = currentIJH.column;
                    g.layer = currentIJH.layer;
                    
                    currentGrids.push_back(g);
                }
            }
        }
        results.push_back(currentGrids);
    }

    return results;
}

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
                                                   double bottom, double top, int level, const BaseTile& baseTile) {
    // 计算每个方向的单元尺寸（基于 baseTile 的分辨率）
    uint64_t perDim = (level >= 0) ? (1ULL << level) : 1ULL;
    const double LDV = (baseTile.north - baseTile.south) / static_cast<double>(perDim);
    const double LOV = (baseTile.east - baseTile.west) / static_cast<double>(perDim);
    const double HDV = (baseTile.top - baseTile.bottom) / static_cast<double>(perDim);

    // 计算索引范围（RCH）——仿照 insertCube.js 的方式
    const long R_start = std::max(0L, static_cast<long>(std::floor((baseTile.north - north) / LDV)));
    const long R_end = std::min(static_cast<long>(perDim) - 1, static_cast<long>(std::floor((baseTile.north - south) / LDV)));
    const long C_start = std::max(0L, static_cast<long>(std::floor((west - baseTile.west) / LOV)));
    const long C_end = std::min(static_cast<long>(perDim) - 1, static_cast<long>(std::floor((east - baseTile.west) / LOV)));
    const long H_start = std::max(0L, static_cast<long>(std::floor((bottom - baseTile.bottom) / HDV)));
    const long H_end = std::min(static_cast<long>(perDim) - 1, static_cast<long>(std::floor((top - baseTile.bottom) / HDV)));

    // 计算预估单元总数
    const long long countEstimate = static_cast<long long>(R_end - R_start + 1) * 
                                   static_cast<long long>(C_end - C_start + 1) * 
                                   static_cast<long long>(H_end - H_start + 1);
    
    // 检查参数有效性
    if (R_start > R_end || C_start > C_end || H_start > H_end || countEstimate <= 0) {
        return std::nullopt;
    }

    // 根据数据量决定是否使用多线程
    const bool useMultiThread = countEstimate > 100000; // 超过10万个网格时使用多线程
    const unsigned int numThreads = useMultiThread ? std::min(std::thread::hardware_concurrency(), 8u) : 1u;
    
    std::vector<std::string> allCodes;
    allCodes.reserve(countEstimate);
    
    if (useMultiThread && numThreads > 1) {
        // 多线程处理：按层分割
        std::vector<std::future<std::vector<std::string>>> futures;
        futures.reserve(numThreads);
        
        const long layersPerThread = (H_end - H_start + 1) / numThreads;
        
        for (unsigned int i = 0; i < numThreads; ++i) {
            const long threadH_start = H_start + i * layersPerThread;
            const long threadH_end = (i == numThreads - 1) ? H_end : threadH_start + layersPerThread - 1;
            
            futures.emplace_back(std::async(std::launch::async, [threadH_start, threadH_end, R_start, R_end, C_start, C_end, level]() {
                std::vector<std::string> threadCodes;
                threadCodes.reserve(static_cast<size_t>(threadH_end - threadH_start + 1) * 
                                   static_cast<size_t>(R_end - R_start + 1) * 
                                   static_cast<size_t>(C_end - C_start + 1));
                
                for (long h = threadH_start; h <= threadH_end; ++h) {
                    for (long r = R_start; r <= R_end; ++r) {
                        for (long c = C_start; c <= C_end; ++c) {
                            threadCodes.emplace_back(IJH2DQG_str(static_cast<uint32_t>(r), 
                                                               static_cast<uint32_t>(c), 
                                                               static_cast<uint32_t>(h), 
                                                               static_cast<uint8_t>(level)));
                        }
                    }
                }
                
                return threadCodes;
            }));
        }
        
        // 收集所有线程的结果
        for (auto& future : futures) {
            auto threadResult = future.get();
            allCodes.insert(allCodes.end(), 
                          std::make_move_iterator(threadResult.begin()), 
                          std::make_move_iterator(threadResult.end()));
        }
    } else {
        // 单线程处理
        for (long h = H_start; h <= H_end; ++h) {
            for (long r = R_start; r <= R_end; ++r) {
                for (long c = C_start; c <= C_end; ++c) {
                    allCodes.emplace_back(IJH2DQG_str(static_cast<uint32_t>(r), 
                                                     static_cast<uint32_t>(c), 
                                                     static_cast<uint32_t>(h), 
                                                     static_cast<uint8_t>(level)));
                }
            }
        }
    }
    
    // 转换为Gridbox格式（保持接口兼容性）
    std::vector<Gridbox> gridCells;
    gridCells.reserve(allCodes.size());
    
    for (auto& code : allCodes) {
        Gridbox gridbox;
        gridbox.code = std::move(code);
        gridCells.emplace_back(std::move(gridbox));
    }
    
    return gridCells;
}

//*********************基于js中算法（扫描线）的多边形局部网格化实现************************//
// 最小的 JSON 解析器，用于 [lon，lat] 对数组。预计格式: [[lon,lat],[lon,lat],...]
static bool parsePolygonJson(const std::string& json, std::vector<std::pair<double,double>>& out) {
    out.clear();
    size_t i = 0;
    // skip spaces
    auto skipSpaces = [&](void){ while (i < json.size() && isspace((unsigned char)json[i])) ++i; };
    skipSpaces();
    if (i >= json.size() || json[i] != '[') return false;
    ++i; // skip '['
    skipSpaces();
    while (i < json.size()) {
        if (json[i] == ']') { ++i; break; }
        if (json[i] != '[') return false;
        ++i; skipSpaces();
        // parse lon
        size_t start = i;
        // allow number chars, sign, decimal, exponent
        while (i < json.size() && (isdigit((unsigned char)json[i]) || json[i]=='+' || json[i]=='-' || json[i]=='.' || json[i]=='e' || json[i]=='E')) ++i;
        if (start==i) return false;
        double lon = 0.0;
        try { lon = std::stod(json.substr(start, i-start)); } catch(...) { return false; }
        skipSpaces();
        if (i >= json.size() || json[i] != ',') return false;
        ++i; skipSpaces();
        // parse lat
        start = i;
        while (i < json.size() && (isdigit((unsigned char)json[i]) || json[i]=='+' || json[i]=='-' || json[i]=='.' || json[i]=='e' || json[i]=='E')) ++i;
        if (start==i) return false;
        double lat = 0.0;
        try { lat = std::stod(json.substr(start, i-start)); } catch(...) { return false; }
        skipSpaces();
        if (i >= json.size() || json[i] != ']') return false;
        ++i; skipSpaces();
        out.emplace_back(lon, lat);
        if (i < json.size() && json[i] == ',') { ++i; skipSpaces(); continue; }
    }
    return out.size() >= 3;
}

// 用于存储双精度的坐标点
struct PointDouble {
    double x;
    double y;
};

// 改进版：保守光栅化扫描线填充（确保边缘沾边即包含）
static std::vector<Point> optimizedConservativeScanLineFill(const std::vector<PointDouble>& polygon) {
    if (polygon.size() < 3) return {};

    // 1. 获取浮点级别的上下边界
    double minY = polygon[0].y;
    double maxY = polygon[0].y;
    for (const auto& p : polygon) {
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    // 向外扩展覆盖所有触碰到的整数行
    int yStart = static_cast<int>(std::floor(minY));
    int yEnd   = static_cast<int>(std::floor(maxY));

    struct EdgeDbl { double x; double ymin; double ymax; double dx; };
    std::vector<EdgeDbl> edges;
    edges.reserve(polygon.size());

    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[(i + 1) % polygon.size()];
        if (std::abs(p1.y - p2.y) < 1e-9) continue; // 忽略完全水平的边

        double ymin = std::min(p1.y, p2.y);
        double ymax = std::max(p1.y, p2.y);
        double x_at_ymin = (p1.y < p2.y) ? p1.x : p2.x;
        double dx = (p2.x - p1.x) / (p2.y - p1.y);

        edges.push_back({x_at_ymin, ymin, ymax, dx});
    }

    std::vector<Point> filled;
    // 2. 遍历多边形跨越的每一行网格空间 [y, y+1]
    for (int y = yStart; y <= yEnd; ++y) {
        std::vector<double> inters;
        double gridTopY = static_cast<double>(y);
        double gridBottomY = static_cast<double>(y + 1);

        for (const auto& e : edges) {
            // 判断边是否与当前网格行有交集
            if (e.ymax >= gridTopY && e.ymin <= gridBottomY) {
                // 计算与网格上下边界的交点
                if (e.ymin <= gridTopY && e.ymax >= gridTopY) {
                    inters.push_back(e.x + (gridTopY - e.ymin) * e.dx);
                }
                if (e.ymin <= gridBottomY && e.ymax >= gridBottomY) {
                    inters.push_back(e.x + (gridBottomY - e.ymin) * e.dx);
                }
                // 将包含在当前行内的线段端点也作为极值点加入
                if (e.ymin >= gridTopY && e.ymin <= gridBottomY) {
                    inters.push_back(e.x);
                }
                if (e.ymax >= gridTopY && e.ymax <= gridBottomY) {
                    inters.push_back(e.x + (e.ymax - e.ymin) * e.dx);
                }
            }
        }

        if (inters.empty()) continue;

        // 3. 提取该行网格上的最小和最大 X 跨度
        double minX = *std::min_element(inters.begin(), inters.end());
        double maxX = *std::max_element(inters.begin(), inters.end());

        // X轴向外取整，确保沾边的列也被包含
        int startX = static_cast<int>(std::floor(minX));
        int endX   = static_cast<int>(std::floor(maxX));

        // 填充该行所有被触碰的网格
        for (int x = startX; x <= endX; ++x) {
            filled.push_back({x, y, 0.0});
        }
    }
    return filled;
}
/// @brief 从 JSON 格式的多边形顶点生成局部网格编码集合（立体填充）
std::vector<std::string> getPolygonGridCodes(
    const std::string& polygonJson,
    double top,
    double bottom,
    uint8_t level,
    const BaseTile& baseTile)
{
    std::vector<std::pair<double,double>> coords;
    if (!parsePolygonJson(polygonJson, coords)) return {};

    // 计算当前层级的经纬差和高差
    double LDV = (baseTile.north - baseTile.south) / std::pow(2.0, level);
    double LOV = (baseTile.east - baseTile.west) / std::pow(2.0, level);
    double HDV = 78125.0 / std::pow(2.0, level); // 保持与基础库一致

    // 1. 保留浮点精度转换为网格坐标
    std::vector<PointDouble> gridVerts;
    gridVerts.reserve(coords.size());

    for (const auto& pr : coords) {
        double lon = pr.first;
        double lat = pr.second;

        double exact_row = (baseTile.north - lat) / LDV;
        double exact_col = (lon - baseTile.west) / LOV;

        // 处理跨越180度
        if (baseTile.west < -180.0 && lon > 0.0) {
            exact_col = (lon - baseTile.west - 360.0) / LOV;
        }

        gridVerts.push_back({exact_col, exact_row});
    }

    // 2. 调用全新的保守光栅化算法获取2D面
    auto filled2D = optimizedConservativeScanLineFill(gridVerts);
    if (filled2D.empty()) return {};

    // 3. 高度层级严格向外取整包围 (Z轴保守光栅化)
    double relativeBottom = bottom - baseTile.bottom;
    double relativeTop = top - baseTile.bottom;
    if (relativeBottom < 0.0) relativeBottom = 0.0;
    if (relativeTop < 0.0) relativeTop = 0.0;

    uint32_t bottomLayer = static_cast<uint32_t>(std::floor(relativeBottom / HDV));
    uint32_t topLayer = 0;
    // 顶部使用 ceil 确保只要触碰到上一层，就把该层包进去。减1是因为层号是从底向上计算的边界索引
    if (relativeTop > 0.0) {
        topLayer = static_cast<uint32_t>(std::ceil(relativeTop / HDV)) - 1;
    }
    if (topLayer < bottomLayer || topLayer == static_cast<uint32_t>(-1)) topLayer = bottomLayer;

    // 限制最大层号
    if (topLayer >= (1u << level)) topLayer = (1u << level) - 1;

    // 优化：使用 IJH 结构体和多线程处理，减少内存占用和计算时间
    size_t total2D = filled2D.size();
    size_t numLayers = topLayer - bottomLayer + 1;
    size_t totalEstimated = total2D * numLayers;

    unsigned int numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0) numThreads = 4;

    std::vector<IJH> allIJHs;

    // 如果任务量较小，直接单线程处理
    if (totalEstimated < 10000) {
        allIJHs.reserve(totalEstimated);
        for (const auto& p : filled2D) {
            for (uint32_t h = bottomLayer; h <= topLayer; ++h) {
                allIJHs.push_back({static_cast<uint32_t>(p.y), static_cast<uint32_t>(p.x), h});
            }
        }
    } else {
        // 多线程并行生成 IJH
        std::vector<std::future<std::vector<IJH>>> futures;
        size_t chunkSize = (total2D + numThreads - 1) / numThreads;

        for (unsigned int i = 0; i < numThreads; ++i) {
            size_t start = i * chunkSize;
            size_t end = std::min(start + chunkSize, total2D);
            if (start >= end) break;

            futures.push_back(std::async(std::launch::async, [start, end, &filled2D, bottomLayer, topLayer]() {
                std::vector<IJH> localIJHs;
                localIJHs.reserve((end - start) * (topLayer - bottomLayer + 1));
                for (size_t k = start; k < end; ++k) {
                    const auto& p = filled2D[k];
                    for (uint32_t h = bottomLayer; h <= topLayer; ++h) {
                        localIJHs.push_back({static_cast<uint32_t>(p.y), static_cast<uint32_t>(p.x), h});
                    }
                }
                return localIJHs;
            }));
        }

        // 收集结果
        allIJHs.reserve(totalEstimated);
        for (auto& f : futures) {
            auto vec = f.get();
            allIJHs.insert(allIJHs.end(), vec.begin(), vec.end());
        }
    }

    // 排序并去重 (IJH 结构体比 string 小得多，排序更快)
    std::sort(allIJHs.begin(), allIJHs.end());
    auto last = std::unique(allIJHs.begin(), allIJHs.end());
    allIJHs.erase(last, allIJHs.end());

    // 并行转换为字符串
    size_t finalCount = allIJHs.size();
    std::vector<std::string> out(finalCount);

    if (finalCount < 10000) {
        for (size_t k = 0; k < finalCount; ++k) {
            try {
                out[k] = IJH2DQG_str(allIJHs[k].row, allIJHs[k].column, allIJHs[k].layer, level);
            } catch (...) {}
        }
    } else {
        size_t strChunkSize = (finalCount + numThreads - 1) / numThreads;
        std::vector<std::future<void>> strFutures;

        for (unsigned int i = 0; i < numThreads; ++i) {
            size_t start = i * strChunkSize;
            size_t end = std::min(start + strChunkSize, finalCount);
            if (start >= end) break;

            strFutures.push_back(std::async(std::launch::async, [start, end, &allIJHs, &out, level]() {
                for (size_t k = start; k < end; ++k) {
                    try {
                        out[k] = IJH2DQG_str(allIJHs[k].row, allIJHs[k].column, allIJHs[k].layer, level);
                    } catch (...) {}
                }
            }));
        }
        for (auto& f : strFutures) f.get();
    }

    // 清理可能产生的空字符串（如果发生异常）
    // out.erase(std::remove(out.begin(), out.end(), ""), out.end());

    return out;
}

/// @brief 仅生成多边形体的外部表面网格（上表面、下表面和侧面），不填充内部体积
std::vector<std::string> getPolygonSurfaceGridCodes(
    const std::string& polygonJson,
    double top,
    double bottom,
    uint8_t level,
    const BaseTile& baseTile)
{
    std::vector<std::pair<double,double>> coords;
    if (!parsePolygonJson(polygonJson, coords)) return {};

    double LDV = (baseTile.north - baseTile.south) / std::pow(2.0, level);
    double LOV = (baseTile.east - baseTile.west) / std::pow(2.0, level);
    double HDV = 78125.0 / std::pow(2.0, level);

    std::vector<PointDouble> gridVerts;
    gridVerts.reserve(coords.size());

    for (const auto& pr : coords) {
        double lon = pr.first;
        double lat = pr.second;

        double exact_row = (baseTile.north - lat) / LDV;
        double exact_col = (lon - baseTile.west) / LOV;
        if (baseTile.west < -180.0 && lon > 0.0) exact_col = (lon - baseTile.west - 360.0) / LOV;

        gridVerts.push_back({exact_col, exact_row});
    }

    auto filled2D = optimizedConservativeScanLineFill(gridVerts);

    // 高度层级严格包围
    double relativeBottom = bottom - baseTile.bottom;
    double relativeTop = top - baseTile.bottom;
    if (relativeBottom < 0.0) relativeBottom = 0.0;
    if (relativeTop < 0.0) relativeTop = 0.0;

    uint32_t bottomLayer = static_cast<uint32_t>(std::floor(relativeBottom / HDV));
    uint32_t topLayer = 0;
    if (relativeTop > 0.0) topLayer = static_cast<uint32_t>(std::ceil(relativeTop / HDV)) - 1;
    if (topLayer < bottomLayer || topLayer == static_cast<uint32_t>(-1)) topLayer = bottomLayer;
    if (topLayer >= (1u << level)) topLayer = (1u << level) - 1;

    std::unordered_set<std::string> uniqueCodes;

    // 1) 添加上/下表面网格
    for (const auto& p : filled2D) {
        try {
            std::string topCode = IJH2DQG_str(static_cast<uint32_t>(p.y), static_cast<uint32_t>(p.x), topLayer, level);
            uniqueCodes.insert(topCode);
        } catch(...) {}
        try {
            std::string bottomCode = IJH2DQG_str(static_cast<uint32_t>(p.y), static_cast<uint32_t>(p.x), bottomLayer, level);
            uniqueCodes.insert(bottomCode);
        } catch(...) {}
    }

    // 2) 添加侧面网格 (使用连续坐标插值确保侧边不透风)
    const size_t n = gridVerts.size();
    if (n >= 2) {
        for (size_t i = 0; i < n; ++i) {
            const auto& p0 = gridVerts[i];
            const auto& p1 = gridVerts[(i+1)%n];
            double c0 = p0.x; double r0 = p0.y;
            double c1 = p1.x; double r1 = p1.y;

            // 按照最长轴进行步进，确保在网格级别不会出现断层
            int steps = static_cast<int>(std::max(std::ceil(std::abs(c1 - c0)), std::ceil(std::abs(r1 - r0)))) * 2;
            if (steps == 0) steps = 1;

            for (int s = 0; s <= steps; ++s) {
                double t = static_cast<double>(s) / static_cast<double>(steps);
                int col = static_cast<int>(std::floor(c0 + t * (c1 - c0))); // 向下取整落在对应网格
                int row = static_cast<int>(std::floor(r0 + t * (r1 - r0)));
                for (uint32_t h = bottomLayer; h <= topLayer; ++h) {
                    try {
                        std::string code = IJH2DQG_str(static_cast<uint32_t>(row), static_cast<uint32_t>(col), h, level);
                        uniqueCodes.insert(code);
                    } catch(...) {}
                }
            }
        }
    }

    std::vector<std::string> out;
    out.reserve(uniqueCodes.size());
    for (const auto& s : uniqueCodes) out.push_back(s);
    std::sort(out.begin(), out.end());
    return out;
}