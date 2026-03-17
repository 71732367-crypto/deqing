# dqglib 函数接口文档

## 1. 库基本信息

### 1.1 库名称
**dqglib** - 离散正交四叉格网（Discrete Orthogonal Quadtree Grid）函数库，版本 1.0

### 1.2 依赖项说明

| 依赖库名称 | 用途 | 版本要求 |
|-----------|------|---------|
| OpenSceneGraph | 3D场景图处理 | osgDB, osg, osgGA, osgViewer, osgUtil, osgText |
| TIFF | TIFF图像格式处理 | 必需 |
| ZLIB | 数据压缩 | 必需 |
| PROJ | 坐标系转换和投影 | 必需（需要数据目录） |
| jsoncpp | JSON解析 | 必需 |

### 1.3 编译要求
- CMake 3.10+
- C++20 标准
- 支持的平台：Linux, Windows, macOS

### 1.4 库文件
- **静态库**: `libdqglib.a`
- **头文件目录**: `include/dqg/`

---

## 2. 数据结构定义

### 2.1 基础结构体

```cpp
// 行列号结构体
struct IJ {
    uint32_t row;    // 行号
    uint32_t column; // 列号
};

// 行列高层结构体
struct IJH {
    uint32_t row;    // 行号
    uint32_t column; // 列号
    uint32_t layer;  // 层号
};

// 经纬度结构体
struct PointLBd {
    double Lng;      // 经度（度）
    double Lat;      // 纬度（度）
};

// 经纬高结构体
struct PointLBHd {
    double Lng;      // 经度（度）
    double Lat;      // 纬度（度）
    double Hgt;      // 高度（米）
};

// 三角形结构体
struct Triangle {
    PointLBHd vertex1; // 顶点1
    PointLBHd vertex2; // 顶点2
    PointLBHd vertex3; // 顶点3
};

// 网格包围盒
struct Gridbox {
    double west, east;    // 东西边界
    double north, south;  // 南北边界
    double bottom, top;   // 底顶高度
    double Lng, Lat, Hgt; // 中心坐标
    uint32_t row, column, layer; // 网格索引
    uint8_t level;        // 层级
    uint16_t octNum;      // 八分体号
    string code;          // 网格编码
};

// 基础瓦片
struct BaseTile {
    double west, south;  // 西南角
    double east, north;  // 东北角
    double top, bottom;  // 顶底高度
};

// 米坐标系三维向量
struct V3 {
    double x, y, z;  // 坐标（米）
};
```

---

## 3. 核心编码/解码函数

### 3.1 2D网格编码/解码

#### `LB2DQG_str`
将经纬度转换为DQG 2D网格编码字符串。

**函数原型:**
```cpp
string LB2DQG_str(double l, double b, int level);
```

**参数说明:**
- `l` (double): 经度，单位为度，范围 [-180, 180]
- `b` (double): 纬度，单位为度，范围 [-90, 90]
- `level` (int): 网格层级，范围 [0, 30]，层级越高网格越精细

**返回值:**
- DQG网格编码字符串（八进制表示）

**调用示例:**
```cpp
string code = LB2DQG_str(120.0, 30.0, 9);
```

---

#### `DQG2LB_b`
将DQG 2D网格编码解码为经纬度中心点。

**函数原型:**
```cpp
PointLBd DQG2LB_b(uint64_t DQGCode, int level);
```

**参数说明:**
- `DQGCode` (uint64_t): DQG网格编码（Morton编码）
- `level` (int): 网格层级，范围 [0, 30]

**返回值:**
- `PointLBd`: 网格中心点的经纬度坐标

**调用示例:**
```cpp
PointLBd center = DQG2LB_b(mortonCode, 9);
```

---

### 3.2 3D网格编码/解码

#### `LBH2DQG_str`
将经纬高转换为DQG 3D网格编码字符串。

**函数原型:**
```cpp
std::string LBH2DQG_str(double l, double b, double hei, uint8_t level);
```

**参数说明:**
- `l` (double): 经度，单位为度，范围 [-180, 180]
- `b` (double): 纬度，单位为度，范围 [-90, 90]
- `hei` (double): 高度，单位为米，范围 [-10000000, 10000000]
- `level` (uint8_t): 网格层级，范围 [0, 21]

**返回值:**
- DQG 3D网格编码字符串

**调用示例:**
```cpp
string code = LBH2DQG_str(120.0, 30.0, 100.0, 9);
```

---

#### `IJH2DQG_str`
将行列高索引转换为DQG 3D网格编码字符串。

**函数原型:**
```cpp
std::string IJH2DQG_str(uint32_t row, uint32_t col, uint32_t layer, uint8_t level);
```

**参数说明:**
- `row` (uint32_t): 行号
- `col` (uint32_t): 列号
- `layer` (uint32_t): 层号
- `level` (uint8_t): 网格层级

**返回值:**
- DQG 3D网格编码字符串

---

#### `Decode_3D`
将DQG 3D网格编码解码为经纬高坐标。

**函数原型:**
```cpp
PointLBHd Decode_3D(const std::string& code);
```

**参数说明:**
- `code` (const std::string&): DQG 3D网格编码字符串

**返回值:**
- `PointLBHd`: 网格中心点的经纬高坐标

---

#### `Codes2Gridbox`
将网格编码解码为网格包围盒信息。

**函数原型:**
```cpp
Gridbox Codes2Gridbox(const std::string& code);
```

**参数说明:**
- `code` (const std::string&): DQG网格编码字符串

**返回值:**
- `Gridbox`: 包含网格边界、中心点、索引等完整信息的包围盒

---

## 4. 层级关系函数

### 4.1 父网格查询

#### `getLevelFatherCode`
获取指定层级的父网格编码。

**函数原型:**
```cpp
std::string getLevelFatherCode(const std::string& code, uint8_t level);
```

**参数说明:**
- `code` (const std::string&): 子网格编码
- `level` (uint8_t): 目标父层级，范围 [0, 当前层级-1]

**返回值:**
- 父网格编码字符串

**调用示例:**
```cpp
string parent = getLevelFatherCode("12345", 5);
```

---

#### `getDQGCellFatherBylevel_MTCode`
根据层级获取父网格的Morton编码。

**函数原型:**
```cpp
uint64_t getDQGCellFatherBylevel_MTCode(uint64_t code, uint8_t level);
```

**参数说明:**
- `code` (uint64_t): Morton编码
- `level` (uint8_t): 目标父层级

**返回值:**
- 父网格的Morton编码

---

### 4.2 子网格查询

#### `getChildCode`
获取网格的所有子网格编码。

**函数原型 (DQG3DBasic.h):**
```cpp
std::vector<std::string> getChildCode(const std::string& code);
```

**参数说明:**
- `code` (const std::string&): 父网格编码

**返回值:**
- 子网格编码数组（通常8个子网格）

**调用示例:**
```cpp
vector<string> children = getChildCode("12345");
```

---

#### `getChildCode`
获取网格的所有子网格Morton编码。

**函数原型 (DQG2D.h):**
```cpp
void getChildCode(uint64_t code, code_data* childrencodes);
```

**参数说明:**
- `code` (uint64_t): 父网格Morton编码
- `childrencodes` (code_data*): 输出参数，子网格编码数组

**返回值:**
- 无，通过childrencodes参数返回

---

## 5. 几何计算函数

### 5.1 距离计算

#### `distance`
计算IJH点间的欧几里得距离。

**函数原型:**
```cpp
double distance(const IJH& point1, const IJH& point2);
```

**参数说明:**
- `point1` (const IJH&): 第一个点
- `point2` (const IJH&): 第二个点

**返回值:**
- 两点间的欧几里得距离

---

#### `distance3D`
计算经纬高坐标的三维欧氏距离。

**函数原型:**
```cpp
double distance3D(const PointLBHd& p1, const PointLBHd& p2);
```

**参数说明:**
- `p1` (const PointLBHd&): 第一个点
- `p2` (const PointLBHd&): 第二个点

**返回值:**
- 三维欧氏距离（单位：米）

---

#### `haversineDistance`
使用Haversine公式计算两点间的球面距离。

**函数原型:**
```cpp
double haversineDistance(const PointLBHd& p1, const PointLBHd& p2);
```

**参数说明:**
- `p1` (const PointLBHd&): 第一个点
- `p2` (const PointLBHd&): 第二个点

**返回值:**
- 球面距离（单位：米）

---

### 5.2 角度和方位

#### `azimuth`
计算方位角。

**函数原型:**
```cpp
double azimuth(double x, double y);
```

**参数说明:**
- `x` (double): X方向分量
- `y` (double): Y方向分量

**返回值:**
- 方位角（弧度）

---

#### `movePoint`
计算沿特定角度移动一定距离后的新坐标。

**函数原型:**
```cpp
PointLBHd movePoint(const PointLBHd& p, double distance, double angle);
```

**参数说明:**
- `p` (const PointLBHd&): 起始点
- `distance` (double): 移动距离（米）
- `angle` (double): 方位角（弧度）

**返回值:**
- 移动后的新坐标

---

### 5.3 共线性检测

#### `collinear_3Points`
检测三个点是否共线。

**函数原型:**
```cpp
bool collinear_3Points(const IJH& point1, const IJH& point2, const IJH& point3);
```

**参数说明:**
- `point1` (const IJH&): 第一个点
- `point2` (const IJH&): 第二个点
- `point3` (const IJH&): 第三个点

**返回值:**
- true表示共线，false表示不共线

---

## 6. 线网格化函数

### 6.1 2D线网格化

#### `bresenham2D`
使用Bresenham算法生成2D线段经过的网格。

**函数原型:**
```cpp
std::vector<IJ> bresenham2D(const IJ& p1, const IJ& p2);
```

**参数说明:**
- `p1` (const IJ&): 起始网格行列号
- `p2` (const IJ&): 终止网格行列号

**返回值:**
- 线段经过的网格行列号数组

---

#### `bresenham2D`
使用Bresenham算法生成2D线段经过的DQG编码。

**函数原型:**
```cpp
vector<string> bresenham2D(PointLBd& point1, PointLBd& point2, int level);
```

**参数说明:**
- `point1` (PointLBd&): 起始点经纬度
- `point2` (PointLBd&): 终止点经纬度
- `level` (int): 网格层级

**返回值:**
- 线段经过的DQG编码数组

---

#### `rasterizeAndEncode`
将经纬度线段数组网格化并编码。

**函数原型:**
```cpp
vector<string> rasterizeAndEncode(const vector<PointLBd>& lineLB, int level);
```

**参数说明:**
- `lineLB` (const vector<PointLBd>&): 经纬度线段点数组
- `level` (int): 网格层级

**返回值:**
- 线段经过的DQG编码数组

---

### 6.2 3D线网格化

#### `bresenham3D`
使用3D Bresenham算法生成3D线段经过的网格。

**函数原型:**
```cpp
std::vector<IJH> bresenham3D(const IJH& p1, const IJH& p2);
```

**参数说明:**
- `p1` (const IJH&): 起始网格行列高层号
- `p2` (const IJH&): 终止网格行列高层号

**返回值:**
- 线段经过的网格行列高层号数组

---

#### `senham3D_DQG`
计算3D线段经过的DQG编码。

**函数原型:**
```cpp
std::vector<std::string> senham3D_DQG(const PointLBHd& point1, 
                                      const PointLBHd& point2, 
                                      uint8_t level);
```

**参数说明:**
- `point1` (const PointLBHd&): 起始点经纬高
- `point2` (const PointLBHd&): 终止点经纬高
- `level` (uint8_t): 网格层级

**返回值:**
- 线段经过的DQG 3D编码数组

---

#### `lineToLocalCode`
将3D线网格化为局部网格编码。

**函数原型:**
```cpp
std::vector<std::string> lineToLocalCode(const std::vector<PointLBHd>& lineReq, 
                                          uint8_t level, 
                                          const BaseTile& baseTile);
```

**参数说明:**
- `lineReq` (const std::vector<PointLBHd>&): 线上的点坐标序列
- `level` (uint8_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 局部网格编码数组

---

## 7. 多边形网格化函数

### 7.1 扫描线填充

#### `scanLineFill`
使用扫描线算法填充多边形区域。

**函数原型:**
```cpp
PolygonFillResult scanLineFill(const vector<IJ>& polygon,
                                uint8_t level,
                                uint32_t layer,
                                const BaseTile& baseTile);
```

**参数说明:**
- `polygon` (const vector<IJ>&): 多边形顶点（网格行列号）
- `level` (uint8_t): 网格层级
- `layer` (uint32_t): 高度层
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- `PolygonFillResult`: 包含填充网格和边界网格

---

#### `polygonFill`
单个多边形面网格化算法。

**函数原型:**
```cpp
vector<Point> polygonFill(const vector<Point>& vertices);
```

**参数说明:**
- `vertices` (const vector<Point>&): 多边形顶点

**返回值:**
- 填充的点集合

---

#### `getPolygonGridCodes`
从JSON多边形生成立体填充网格编码。

**函数原型:**
```cpp
std::vector<std::string> getPolygonGridCodes(const std::string& polygonJson,
                                              double top,
                                              double bottom,
                                              uint8_t level,
                                              const BaseTile& baseTile);
```

**参数说明:**
- `polygonJson` (const std::string&): JSON格式多边形顶点 [[lon,lat],...]
- `top` (double): 顶部高度
- `bottom` (double): 底部高度
- `level` (uint8_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 网格编码集合（立体填充）

---

#### `getPolygonSurfaceGridCodes`
从JSON多边形生成仅外表面的网格编码。

**函数原型:**
```cpp
std::vector<std::string> getPolygonSurfaceGridCodes(const std::string& polygonJson,
                                                     double top,
                                                     double bottom,
                                                     uint8_t level,
                                                     const BaseTile& baseTile);
```

**参数说明:**
- `polygonJson` (const std::string&): JSON格式多边形顶点
- `top` (double): 顶部高度
- `bottom` (double): 底部高度
- `level` (uint8_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 外表面网格编码集合（上/下/侧面）

---

#### `getMultiplePolygonGrids`
多个多边形面网格化算法。

**函数原型:**
```cpp
vector<vector<Gridbox>> getMultiplePolygonGrids(uint8_t level,
                                                 const vector<vector<PointLBHd>>& boundaries,
                                                 double top,
                                                 double bottom,
                                                 const BaseTile& baseTile);
```

**参数说明:**
- `level` (uint8_t): 网格层级
- `boundaries` (const vector<vector<PointLBHd>>&): 多个多边形边界
- `top` (double): 顶部高度
- `bottom` (double): 底部高度
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 每个多边形的网格包围盒数组

---

#### `gridCubeRegion`
立方体区域网格化核心计算函数（支持多线程）。

**函数原型:**
```cpp
std::optional<std::vector<Gridbox>> gridCubeRegion(double west,
                                                     double east,
                                                     double north,
                                                     double south,
                                                     double bottom,
                                                     double top,
                                                     int level,
                                                     const BaseTile& baseTile);
```

**参数说明:**
- `west` (double): 区域西边界
- `east` (double): 区域东边界
- `north` (double): 区域北边界
- `south` (double): 区域南边界
- `bottom` (double): 区域底部高度
- `top` (double): 区域顶部高度
- `level` (int): 网格精度级别
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- `std::optional<std::vector<Gridbox>>`: 成功时返回网格单元向量，失败时返回std::nullopt

---

## 8. 三角形网格化函数

### 8.1 单个三角形

#### `triangularGrid`
单个三角面片网格化。

**函数原型:**
```cpp
vector<IJH> triangularGrid(const IJH& p1, 
                            const IJH& p2, 
                            const IJH& p3, 
                            const uint8_t level);
```

**参数说明:**
- `p1` (const IJH&): 第一个顶点
- `p2` (const IJH&): 第二个顶点
- `p3` (const IJH&): 第三个顶点
- `level` (const uint8_t): 网格层级

**返回值:**
- 三角形内部网格索引数组

---

#### `triangular_single`
单个三角面片网格化（使用坐标）。

**函数原型:**
```cpp
vector<string> triangular_single(const Triangle& points, 
                                  uint8_t level, 
                                  const BaseTile& baseTile);
```

**参数说明:**
- `points` (const Triangle&): 三角形（三个顶点）
- `level` (uint8_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 三角形网格编码数组

---

### 8.2 多个三角形

#### `triangular_multiple`
多个三角面片网格化。

**函数原型:**
```cpp
vector<string> triangular_multiple(const vector<Triangle>& triangles, 
                                   uint8_t level, 
                                   const BaseTile& baseTile);
```

**参数说明:**
- `triangles` (const vector<Triangle>&): 三角形数组
- `level` (uint8_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 所有三角形的网格编码数组

---

#### `computePolyhedronGridFill`
三角面片立体模型填充网格计算（核心算法）。

**函数原型:**
```cpp
PolyhedronGridResult computePolyhedronGridFill(const std::vector<std::array<PointLBHd, 3>>& faces,
                                                const BaseTile& baseTile,
                                                int level,
                                                double sampleMultiplier = 2.0,
                                                size_t maxSamples = 0);
```

**参数说明:**
- `faces` (const std::vector<std::array<PointLBHd, 3>>&): 三角面片数组，每个面片包含3个顶点
- `baseTile` (const BaseTile&): 基础网格配置
- `level` (int): 网格层级（0-21），层级越高网格越精细
- `sampleMultiplier` (double): 采样倍率，控制采样密度，默认2.0
- `maxSamples` (size_t): 最大采样数，0表示无限制，用于内存控制

**返回值:**
- `PolyhedronGridResult`: 包含所有相交网格编码的集合及采样统计信息
  - `gridCodes`: 网格编码集合
  - `multiplierUsed`: 实际使用的采样倍率
  - `actualSamples`: 实际采样数量
  - `hitMaxSamples`: 是否达到最大采样数限制
  - `centerCode`: 中心点网格编码

**调用示例:**
```cpp
vector<array<PointLBHd, 3>> faces = {
    {p1, p2, p3},
    {p4, p5, p6}
};
PolyhedronGridResult result = computePolyhedronGridFill(faces, baseTile, 10, 2.0, 0);
```

---

## 9. 缓冲区计算函数

### 9.1 点缓冲区

#### `getBoundingBox`
计算最小外接正方体的经纬高范围。

**函数原型:**
```cpp
BaseTile getBoundingBox(double lon, double lat, double alt, double R);
```

**参数说明:**
- `lon` (double): 经度
- `lat` (double): 纬度
- `alt` (double): 高度
- `R` (double): 缓冲半径

**返回值:**
- 包围盒

---

#### `getPointsBuffer`
3D点缓冲区网格生成。

**函数原型:**
```cpp
vector<vector<LatLonHei>> getPointsBuffer(const vector<PointLBHd>& points,
                                           uint8_t level,
                                           double radius,
                                           const BaseTile& basetile);
```

**参数说明:**
- `points` (const vector<PointLBHd>&): 点坐标数组
- `level` (uint8_t): 网格层级
- `radius` (double): 缓冲半径
- `basetile` (const BaseTile&): 基准瓦片

**返回值:**
- 每个点的缓冲区网格坐标数组

---

### 9.2 线缓冲区

#### `lineInsertPoints`
计算线段与网格交点，生成插值点。

**函数原型:**
```cpp
vector<PointLBHd> lineInsertPoints(const PointLBHd& p1, 
                                    const PointLBHd& p2, 
                                    uint8_t level, 
                                    const BaseTile& baseTile);
```

**参数说明:**
- `p1` (const PointLBHd&): 线段起点
- `p2` (const PointLBHd&): 线段终点
- `level` (uint8_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 插值点数组

---

#### `getLinesBuffer`
线缓冲区生成。

**函数原型:**
```cpp
vector<vector<LatLonHei>> getLinesBuffer(const vector<vector<PointLBHd>>& lineReq,
                                         double radius, 
                                         uint8_t level);
```

**参数说明:**
- `lineReq` (const vector<vector<PointLBHd>>&): 线段数组
- `radius` (double): 缓冲半径
- `level` (uint8_t): 网格层级

**返回值:**
- 每条线的缓冲区网格坐标数组

---

#### `lineBufferFilled`
三维线缓冲区生成算法（立体填充）。

**函数原型:**
```cpp
vector<string> lineBufferFilled(const vector<PointLBHd>& lineReq, 
                                double halfWidth, 
                                double halfHeight,
                                uint8_t level, 
                                const BaseTile& baseTile);
```

**参数说明:**
- `lineReq` (const vector<PointLBHd>&): 三维线上点坐标序列（经、纬、高）
- `halfWidth` (double): 截面矩形的宽度
- `halfHeight` (double): 截面矩形的高度
- `level` (uint8_t): 层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 三维线缓冲区网格编码序列

---

### 9.3 多边形缓冲区

#### `getBufferPoints`
获取多边形缓冲区坐标。

**函数原型:**
```cpp
vector<vector<PointLBHd>> getBufferPoints(const vector<vector<PointLBHd>>& polygons, 
                                         double radius);
```

**参数说明:**
- `polygons` (const vector<vector<PointLBHd>>&): 多边形数组
- `radius` (double): 缓冲半径

**返回值:**
- 缓冲后的多边形坐标数组

---

#### `getPolygonsBuffer`
计算多边形缓冲区网格。

**函数原型:**
```cpp
pair<vector<vector<Gridbox>>, vector<vector<Gridbox>>>
getPolygonsBuffer(uint8_t level, 
                  double top, 
                  double bottom,
                  const vector<vector<PointLBHd>>& s,
                  double radius, 
                  const BaseTile& baseTile);
```

**参数说明:**
- `level` (uint8_t): 网格层级
- `top` (double): 顶部高度
- `bottom` (double): 底部高度
- `s` (const vector<vector<PointLBHd>>&): 多边形数组
- `radius` (double): 缓冲半径
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- pair: 第一个元素为填充网格，第二个元素为表面网格

---

#### `turfBuffer`
计算多边形的缓冲区坐标。

**函数原型:**
```cpp
std::vector<PointLBHd> turfBuffer(const std::vector<PointLBHd>& polygon, 
                                  double radius);
```

**参数说明:**
- `polygon` (const std::vector<PointLBHd>&): 多边形顶点
- `radius` (double): 缓冲半径

**返回值:**
- 缓冲后的多边形顶点数组

---

## 10. 局部网格函数

### 10.1 局部网格配置

#### `localGridConfig`
计算局部网格的配置。

**函数原型:**
```cpp
LocalGridConfig localGridConfig(const PointLBd& A, 
                                 const PointLBd& B, 
                                 const PointLBd& C, 
                                 const PointLBd& D);
```

**参数说明:**
- `A` (const PointLBd&): 区域角点1
- `B` (const PointLBd&): 区域角点2
- `C` (const PointLBd&): 区域角点3
- `D` (const PointLBd&): 区域角点4

**返回值:**
- `LocalGridConfig`: 局部网格配置信息
  - `globalCodeSet`: 全局编码集合
  - `isAcrossDegeneration`: 是否跨越退化边界
  - `isAcrossOctant`: 是否跨越八分体
  - `localCode2D`: 局部2D编码
  - `idLevel`: 标识层级

---

#### `identityLevel`
识别研究区的最佳网格级别。

**函数原型:**
```cpp
uint8_t identityLevel(const PointLBd& A, 
                      const PointLBd& B, 
                      const PointLBd& C, 
                      const PointLBd& D);
```

**参数说明:**
- `A` (const PointLBd&): 区域角点1
- `B` (const PointLBd&): 区域角点2
- `C` (const PointLBd&): 区域角点3
- `D` (const PointLBd&): 区域角点4

**返回值:**
- 最佳网格层级

---

#### `isAcrossOctant`
判断是否跨越八分体。

**函数原型:**
```cpp
bool isAcrossOctant(const PointLBd& A, 
                    const PointLBd& B, 
                    const PointLBd& C, 
                    const PointLBd& D, 
                    int level, 
                    double height);
```

**参数说明:**
- `A-D` (const PointLBd&): 区域角点
- `level` (int): 网格层级
- `height` (double): 高度

**返回值:**
- true表示跨越八分体

---

#### `isAcrossDegeneration`
判断是否跨越退化边界。

**函数原型:**
```cpp
bool isAcrossDegeneration(const PointLBd& A, 
                           const PointLBd& B, 
                           const PointLBd& C, 
                           const PointLBd& D);
```

**参数说明:**
- `A-D` (const PointLBd&): 区域角点

**返回值:**
- true表示跨越退化边界

---

### 10.2 局部网格计算

#### `localGrid`
确定局部剖分基础网格边界坐标。

**函数原型:**
```cpp
BaseTile localGrid(const PointLBd& A, 
                   const PointLBd& B, 
                   const PointLBd& C, 
                   const PointLBd& D, 
                   int level, 
                   double height);
```

**参数说明:**
- `A-D` (const PointLBd&): 区域角点
- `level` (int): 网格层级
- `height` (double): 高度

**返回值:**
- 基础瓦片

---

#### `getBaseTile`
获取基础瓦片。

**函数原型:**
```cpp
BaseTile getBaseTile(const PointLBd& A, 
                     const PointLBd& B, 
                     const PointLBd& C, 
                     const PointLBd& D);
```

**参数说明:**
- `A-D` (const PointLBd&): 区域角点

**返回值:**
- 基础瓦片

---

#### `localRowColHeiNumber`
计算局部网格索引。

**函数原型:**
```cpp
IJH localRowColHeiNumber(uint8_t level, 
                         double longitude, 
                         double latitude, 
                         double height, 
                         const BaseTile& baseTile);
```

**参数说明:**
- `level` (uint8_t): 网格层级
- `longitude` (double): 经度
- `latitude` (double): 纬度
- `height` (double): 高度
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 局部网格行列高索引

---

#### `getLocalCode`
计算局部网格编码。

**函数原型:**
```cpp
std::string getLocalCode(uint8_t level, 
                         double longitude, 
                         double latitude, 
                         double height, 
                         const BaseTile& baseTile);
```

**参数说明:**
- `level` (uint8_t): 网格层级
- `longitude` (double): 经度
- `latitude` (double): 纬度
- `height` (double): 高度
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 局部网格编码

---

#### `getLocalTileRHC`
计算局部网格的行列高。

**函数原型:**
```cpp
IJH getLocalTileRHC(const std::string& code);
```

**参数说明:**
- `code` (const std::string&): 网格编码

**返回值:**
- 局部网格行列高索引

---

#### `getLocalTileLatLon`
计算局部网格的边界信息。

**函数原型:**
```cpp
LatLonHei getLocalTileLatLon(const std::string& code, 
                             const BaseTile& baseTile);
```

**参数说明:**
- `code` (const std::string&): 网格编码
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 局部网格边界信息

---

#### `getLocalTilesLatLon`
计算局部网格的边界信息合集。

**函数原型:**
```cpp
vector<LatLonHei> getLocalTilesLatLon(const vector<string>& codes, 
                                      const BaseTile& baseTile);
```

**参数说明:**
- `codes` (const vector<string>&): 网格编码数组
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 局部网格边界信息数组

---

### 10.3 局部到全局转换

#### `localRCHtoGlobalRCH`
局部网格索引转换为全局网格索引。

**函数原型:**
```cpp
IJH localRCHtoGlobalRCH(const std::string& localCode, 
                        const LocalGridConfig& config);
```

**参数说明:**
- `localCode` (const std::string&): 局部网格编码
- `config` (const LocalGridConfig&): 局部网格配置

**返回值:**
- 全局网格行列高索引

---

#### `localToGlobal`
局部编码转换为全局编码。

**函数原型:**
```cpp
std::string localToGlobal(const std::string& localCode, 
                          const LocalGridConfig& config);
```

**参数说明:**
- `localCode` (const std::string&): 局部网格编码
- `config` (const LocalGridConfig&): 局部网格配置

**返回值:**
- 全局网格编码

---

#### `IJHToLocalTileLatLon`
计算网格区域中心点的经纬度。

**函数原型:**
```cpp
LatLonHei IJHToLocalTileLatLon(const IJH& ijh, 
                                uint32_t level, 
                                const BaseTile& baseTile);
```

**参数说明:**
- `ijh` (const IJH&): 网格行列高索引
- `level` (uint32_t): 网格层级
- `baseTile` (const BaseTile&): 基准瓦片

**返回值:**
- 中心点经纬度

---

#### `rchToCode`
将行列高索引转换为局部编码。

**函数原型:**
```cpp
std::string rchToCode(const IJH& obj, uint8_t level);
```

**参数说明:**
- `obj` (const IJH&): 行列高索引
- `level` (uint8_t): 网格层级

**返回值:**
- 局部编码字符串

---

## 11. 相邻网格查询函数

### 11.1 面相邻

#### `getAdjacentCode`
生成相邻网格的DQG编码。

**函数原型:**
```cpp
string getAdjacentCode(const string& code, 
                       uint64_t row, 
                       uint64_t col, 
                       uint64_t hei);
```

**参数说明:**
- `code` (const string&): 当前网格编码
- `row` (uint64_t): 行偏移量
- `col` (uint64_t): 列偏移量
- `hei` (uint64_t): 层偏移量

**返回值:**
- 相邻网格编码

---

#### `faceAdjacentSearch`
计算给定网格的所有相邻网格编码。

**函数原型:**
```cpp
vector<string> faceAdjacentSearch(const string& code);
```

**参数说明:**
- `code` (const string&): 当前网格编码

**返回值:**
- 相邻网格编码数组（通常6个面相邻）

---

## 12. 编码分组和排序函数

### 12.1 编码分组

#### `multi_Codes`
编码分组处理。

**函数原型:**
```cpp
unordered_map<string, vector<string>> multi_Codes(const vector<string>& arr);
```

**参数说明:**
- `arr` (const vector<string>&): 编码数组

**返回值:**
- 分组后的编码映射

---

#### `decompose_Codes`
分解编码分组。

**函数原型:**
```cpp
vector<string> decompose_Codes(const unordered_map<string, vector<string>>& groups);
```

**参数说明:**
- `groups` (const unordered_map<string, vector<string>>&): 编码分组

**返回值:**
- 分解后的编码数组

---

### 12.2 编码排序

#### `sort_codes`
比较两个编码的排序顺序。

**函数原型:**
```cpp
bool sort_codes(string a, string b);
```

**参数说明:**
- `a` (string): 编码1
- `b` (string): 编码2

**返回值:**
- true表示a应排在b前面

---

#### `sort_in_level`
按层级对编码排序。

**函数原型:**
```cpp
bool sort_in_level(vector<string>& grids_codes);
```

**参数说明:**
- `grids_codes` (vector<string>&): 网格编码数组

**返回值:**
- true表示排序成功

---

#### `sort_in_num`
按数值对编码排序。

**函数原型:**
```cpp
bool sort_in_num(vector<string>& grids_codes);
```

**参数说明:**
- `grids_codes` (vector<string>&): 网格编码数组

**返回值:**
- true表示排序成功

---

## 13. OSG模型提取函数

### 13.1 三角形提取

#### `extractTriangles`
从OSG节点提取三角形（最高效率：根块直提）。

**函数原型:**
```cpp
void extractTriangles(osg::Node* node, vector<Triangle>& triangles);
```

**参数说明:**
- `node` (osg::Node*): OSG节点
- `triangles` (vector<Triangle>&): 输出三角形数组

**返回值:**
- 无，通过triangles参数返回

---

#### `extractTrianglesFromLevel`
从指定层级的文件中提取单个文件中的三角形。

**函数原型:**
```cpp
void extractTrianglesFromLevel(const std::string& folderPath, 
                                int level, 
                                std::vector<Triangle>& triangles);
```

**参数说明:**
- `folderPath` (const std::string&): 文件夹路径
- `level` (int): 层级
- `triangles` (std::vector<Triangle>&): 输出三角形数组

---

#### `extractTrianglesFromLevelFiles`
从指定层级的.osgb文件中提取多个文件中的三角形。

**函数原型:**
```cpp
void extractTrianglesFromLevelFiles(const std::string& folderPath, 
                                    int level, 
                                    std::vector<Triangle>& triangles);
```

**参数说明:**
- `folderPath` (const std::string&): 文件夹路径
- `level` (int): 层级
- `triangles` (std::vector<Triangle>&): 输出三角形数组

---

### 13.2 坐标转换

#### `convertCoordinates`
转换三角形坐标。

**函数原型:**
```cpp
void convertCoordinates(vector<Triangle>& triangles,
                         const string& sourceCRS = "EPSG:4528",
                         const string& targetCRS = "EPSG:4326",
                         double offsetX = 40497564.61263241, 
                         double offsetY = 3377677.966467825,
                         double offsetZ = 53.29796380422264);
```

**参数说明:**
- `triangles` (vector<Triangle>&): 三角形数组
- `sourceCRS` (const string&): 源坐标系，默认EPSG:4528（德清数据）
- `targetCRS` (const string&): 目标坐标系，默认EPSG:4326
- `offsetX` (double): X坐标偏移
- `offsetY` (double): Y坐标偏移
- `offsetZ` (double): Z坐标偏移

**返回值:**
- 无，直接修改triangles

---

#### `convertCoordinatesFromXML`
从数据目录自动读取坐标系和偏移量进行坐标转换。

**函数原型:**
```cpp
void convertCoordinatesFromXML(vector<Triangle>& triangles,
                               const std::string& dataDir,
                               const string& targetCRS = "EPSG:4326");
```

**参数说明:**
- `triangles` (vector<Triangle>&): 三角形数组
- `dataDir` (const std::string&): 数据目录路径
- `targetCRS` (const string&): 目标坐标系，默认EPSG:4326

**返回值:**
- 无，直接修改triangles

---

#### `readSourceCRSFromXML`
从metadata.xml文件中读取源坐标系信息。

**函数原型:**
```cpp
std::string readSourceCRSFromXML(const std::string& dataDir);
```

**参数说明:**
- `dataDir` (const std::string&): 数据目录路径

**返回值:**
- 源坐标系字符串

---

#### `readCoordinateOffsetFromXML`
从metadata.xml文件中读取坐标偏移量信息。

**函数原型:**
```cpp
CoordinateOffset readCoordinateOffsetFromXML(const std::string& dataDir);
```

**参数说明:**
- `dataDir` (const std::string&): 数据目录路径

**返回值:**
- `CoordinateOffset`: 坐标偏移量

---

## 14. 全局基础瓦片函数

### 14.1 初始化

#### `initializeProjectBaseTile`
初始化全局基础瓦片数据。

**函数原型:**
```cpp
void initializeProjectBaseTile(const PointLBd& A, 
                               const PointLBd& B, 
                               const PointLBd& C, 
                               const PointLBd& D);
```

**参数说明:**
- `A` (const PointLBd&): 西南角坐标点
- `B` (const PointLBd&): 西北角坐标点
- `C` (const PointLBd&): 东北角坐标点
- `D` (const PointLBd&): 东南角坐标点

**返回值:**
- 无

---

#### `initializeProjectBaseTileFromConfig`
从JSON配置文件初始化全局基础瓦片数据。

**函数原型:**
```cpp
bool initializeProjectBaseTileFromConfig(const std::string& configFilePath);
```

**参数说明:**
- `configFilePath` (const std::string&): JSON配置文件路径

**返回值:**
- true表示成功初始化，false表示失败

---

### 14.2 查询

#### `getProjectBaseTile`
获取全局基础瓦片数据。

**函数原型:**
```cpp
const BaseTile& getProjectBaseTile();
```

**参数说明:**
- 无

**返回值:**
- 基础瓦片常量引用

---

## 15. 倾斜摄影模型处理函数

### 15.1 塔形模型

#### `pagodaTriangular`
倾斜摄影模型格网化（塔形模型）。

**函数原型:**
```cpp
vector<PointLBHd> pagodaTriangular(const std::vector<PointLBHd>& pointArray, 
                                    uint8_t level, 
                                    const std::vector<int>& index);
```

**参数说明:**
- `pointArray` (const std::vector<PointLBHd>&): 点数组
- `level` (uint8_t): 网格层级
- `index` (const std::vector<int>&): 索引数组

**返回值:**
- 网格化后的点数组

---

### 15.2 空域和建筑

#### `airSpaceGrid`
倾斜摄影空域网格化。

**函数原型:**
```cpp
std::vector<std::string> airSpaceGrid(const std::vector<PointLBHd>& data, 
                                       uint8_t level);
```

**参数说明:**
- `data` (const std::vector<PointLBHd>&): 空域数据
- `level` (uint8_t): 网格层级

**返回值:**
- 网格编码数组

---

#### `buildingsGrid`
倾斜摄影建筑物网格化。

**函数原型:**
```cpp
std::vector<LatLonHei> buildingsGrid(const std::vector<PointLBHd>& data, 
                                       uint8_t level);
```

**参数说明:**
- `data` (const std::vector<PointLBHd>&): 建筑数据
- `level` (uint8_t): 网格层级

**返回值:**
- 网格坐标数组

---

### 15.3 数据转换

#### `getPointData`
倾斜模型坐标点转换。

**函数原型:**
```cpp
vector<PointLBHd> getPointData(const vector<array<double, 3>>& dataArray1, 
                                const vector<int>& dataArray2);
```

**参数说明:**
- `dataArray1` (const vector<array<double, 3>>&): 坐标数组
- `dataArray2` (const vector<int>&): 索引数组

**返回值:**
- 转换后的点数据

---

## 16. 工具函数

### 16.1 角度转换

#### `toRadians`
角度转换为弧度。

**函数原型:**
```cpp
inline double toRadians(double degrees);
```

**参数说明:**
- `degrees` (double): 角度

**返回值:**
- 弧度

---

#### `toDegrees`
弧度转换为角度。

**函数原型:**
```cpp
inline double toDegrees(double radians);
```

**参数说明:**
- `radians` (double): 弧度

**返回值:**
- 角度

---

### 16.2 编码转换

#### `toOctalString`
将Morton编码转换为八进制字符串。

**函数原型:**
```cpp
std::string toOctalString(uint64_t number);
```

**参数说明:**
- `number` (uint64_t): Morton编码

**返回值:**
- 八进制字符串

---

### 16.3 数组去重

#### `getUnique`
IJ数组去重。

**函数原型:**
```cpp
inline vector<IJ> getUnique(const vector<IJ>& points);
```

**参数说明:**
- `points` (const vector<IJ>&): 需要去重的数组

**返回值:**
- 去重后的数组

---

#### `getUnique`
IJH数组去重。

**函数原型:**
```cpp
inline vector<IJH> getUnique(const vector<IJH>& points);
```

**参数说明:**
- `points` (const vector<IJH>&): 需要去重的数组

**返回值:**
- 去重后的数组

---

### 16.4 对象比较

#### `isObjectValueEqual`
检查两个IJH对象的值是否相等。

**函数原型:**
```cpp
inline bool isObjectValueEqual(const IJH& a, const IJH& b);
```

**参数说明:**
- `a` (const IJH&): 第一个对象
- `b` (const IJH&): 第二个对象

**返回值:**
- true表示值相等

---

#### `calculatePlaneEquation`
给定三角形面片的三个顶点，求出平面方程。

**函数原型:**
```cpp
inline void calculatePlaneEquation(const IJH& point1, 
                                   const IJH& point2, 
                                   const IJH& point3,
                                   double& a, 
                                   double& b, 
                                   double& c, 
                                   double& d);
```

**参数说明:**
- `point1` (const IJH&): 第一个点
- `point2` (const IJH&): 第二个点
- `point3` (const IJH&): 第三个点
- `a` (double&): 输出参数，平面方程法向量x分量
- `b` (double&): 输出参数，平面方程法向量y分量
- `c` (double&): 输出参数，平面方程法向量z分量
- `d` (double&): 输出参数，平面方程常数项

**返回值:**
- 无，通过输出参数返回

---

### 16.5 对数计算

#### `log2_d`
计算无符号整数的对数。

**函数原型:**
```cpp
inline unsigned int log2_d(unsigned int n);
```

**参数说明:**
- `n` (unsigned int): 输入值

**返回值:**
- 以2为底的对数

---

#### `log2_ull`
计算64位无符号整数的对数。

**函数原型:**
```cpp
inline int log2_ull(uint64_t n);
```

**参数说明:**
- `n` (uint64_t): 输入值

**返回值:**
- 以2为底的对数

---

### 16.6 Morton编码

#### `mortonEncode_2D_LUT`
使用查找表进行2D Morton编码。

**函数原型:**
```cpp
uint64_t mortonEncode_2D_LUT(uint32_t x, uint32_t y);
```

**参数说明:**
- `x` (uint32_t): X坐标
- `y` (uint32_t): Y坐标

**返回值:**
- 2D Morton编码

---

#### `mortonEncode_3D_LUT`
使用查找表进行3D Morton编码。

**函数原型:**
```cpp
uint64_t mortonEncode_3D_LUT(uint32_t y, uint32_t x, uint32_t z);
```

**参数说明:**
- `y` (uint32_t): Y坐标
- `x` (uint32_t): X坐标
- `z` (uint32_t): Z坐标

**返回值:**
- 3D Morton编码

---

#### `Morto2IJ`
将Morton编码解码为IJ索引。

**函数原型:**
```cpp
IJ Morto2IJ(uint64_t GridMorton);
```

**参数说明:**
- `GridMorton` (uint64_t): Morton编码

**返回值:**
- IJ索引

---

#### `Morto2IJH`
将Morton编码解码为IJH索引。

**函数原型:**
```cpp
IJH Morto2IJH(uint64_t morton);
```

**参数说明:**
- `morton` (uint64_t): Morton编码

**返回值:**
- IJH索引

---

### 16.7 八分体处理

#### `LB2Oct`
求八分体号。

**函数原型:**
```cpp
uint8_t LB2Oct(double Lng, double Lat);
```

**参数说明:**
- `Lng` (double): 经度
- `Lat` (double): 纬度

**返回值:**
- 八分体号（0-7）

---

#### `LBinOctant`
局部和全局经纬度转换。

**函数原型:**
```cpp
PointLBd LBinOctant(double Lng, double Lat, uint8_t octNum);
```

**参数说明:**
- `Lng` (double): 经度
- `Lat` (double): 纬度
- `octNum` (uint8_t): 八分体号

**返回值:**
- 八分体内的局部经纬度

---

#### `LBinGlobal`
局部经纬度转换为全局经纬度。

**函数原型:**
```cpp
void LBinGlobal(double& l, double& b, uint8_t oct);
```

**参数说明:**
- `l` (double&): 输入输出经度
- `b` (double&): 输入输出纬度
- `oct` (uint8_t): 八分体号

**返回值:**
- 无，通过引用参数返回

---

#### `LB2DQG_ij_oct_int`
2D经纬度转IJ和八分体号。

**函数原型:**
```cpp
IJ_oct_int LB2DQG_ij_oct_int(double l, double b, uint8_t level);
```

**参数说明:**
- `l` (double): 经度
- `b` (double): 纬度
- `level` (uint8_t): 层级

**返回值:**
- `IJ_oct_int`: 包含i、j、八分体号、层级的结构体

---

#### `LBH2DQG_ijh_oct_int`
3D经纬高转IJH和八分体号。

**函数原型:**
```cpp
IJH_oct_int LBH2DQG_ijh_oct_int(double l, double b, double hei, uint8_t level);
```

**参数说明:**
- `l` (double): 经度
- `b` (double): 纬度
- `hei` (double): 高度
- `level` (uint8_t): 层级

**返回值:**
- `IJH_oct_int`: 包含i、j、h、八分体号、层级的结构体

---

#### `IJ_oct_int2str`
2D IJ和八分体号转DQG字符串。

**函数原型:**
```cpp
string IJ_oct_int2str(IJ_oct_int IJ);
```

**参数说明:**
- `IJ` (IJ_oct_int): IJ和八分体信息

**返回值:**
- DQG编码字符串

---

#### `IJH_oct_int2str`
3D IJH和八分体号转DQG字符串。

**函数原型:**
```cpp
string IJH_oct_int2str(IJH_oct_int IJ);
```

**参数说明:**
- `IJ` (IJH_oct_int): IJH和八分体信息

**返回值:**
- DQG编码字符串

---

#### `LB2IJ`
经纬度转IJ。

**函数原型:**
```cpp
IJ LB2IJ(double l, double b, uint8_t level);
```

**参数说明:**
- `l` (double): 经度
- `b` (double): 纬度
- `level` (uint8_t): 层级

**返回值:**
- IJ索引

---

#### `LBH2IJH`
经纬高转IJH。

**函数原型:**
```cpp
IJH LBH2IJH(double l, double b, double hei, uint8_t level);
```

**参数说明:**
- `l` (double): 经度
- `b` (double): 纬度
- `hei` (double): 高度
- `level` (uint8_t): 层级

**返回值:**
- IJH索引

---

---

## 17. 调用约定

### 17.1 基本规则
1. **C++命名空间**: 所有函数均在全局命名空间中，使用`dqg`前缀的命名空间宏
2. **异常处理**: 大部分函数不抛出异常，需要通过返回值检查错误
3. **内存管理**: 返回的容器（vector, string等）使用移动语义，避免不必要的拷贝
4. **线程安全**: 大部分函数非线程安全，需在多线程环境下自行加锁

### 17.2 参数传递
- **基本类型**: 使用值传递
- **结构体**: 小结构体使用值传递，大结构体使用const引用
- **容器**: 输入参数使用const引用，输出参数使用引用或返回值

### 17.3 坐标系说明
- **WGS84**: 使用WGS84椭球体参数
- **EPSG:4326**: 地理坐标系（经纬度）
- **高度**: 单位为米，正值表示海拔以上，负值表示海拔以下

### 17.4 网格层级说明
- **2D层级**: 范围 [0, 30]，层级越高网格越精细
- **3D层级**: 范围 [0, 21]，层级越高网格越精细
- **层级与精度关系**: 每增加1层，网格面积/体积减半

---

## 18. 兼容性说明

### 18.1 平台兼容性
| 平台 | 支持状态 | 说明 |
|------|---------|------|
| Linux | ✓ 完全支持 | 需安装相关依赖库 |
| Windows | ✓ 完全支持 | 使用CMake构建 |
| macOS | ✓ 完全支持 | 需安装相关依赖库 |

### 18.2 编译器兼容性
- GCC 9.0+
- Clang 10.0+
- MSVC 2019+

### 18.3 C++标准
- 最低要求: C++20
- 推荐使用: C++20

### 18.4 依赖版本要求
- OpenSceneGraph: 3.6.0+
- TIFF: 4.0.0+
- ZLIB: 1.2.0+
- PROJ: 6.0.0+
- jsoncpp: 1.7.0+

---

## 19. 版本变更记录

### Version 1.0 (当前版本)
- 初始版本
- 支持2D/3D DQG网格编码/解码
- 支持线、多边形、三角形网格化
- 支持缓冲区计算
- 支持OSG模型提取
- 支持局部网格系统
- 支持相邻网格查询

---

## 20. 已知问题列表

### 20.1 已知限制
1. **高精度层级性能**: 网格层级大于18时，计算性能显著下降
2. **大区域内存消耗**: 处理大区域时，内存消耗可能较高，建议使用局部网格系统
3. **PROJ数据路径**: 需确保PROJ数据目录路径正确，否则坐标转换可能失败

### 20.2 注意事项
1. **坐标输入**: 所有经纬度输入应为WGS84/EPSG:4326坐标系
2. **高度范围**: 建议高度范围在±10000公里内，超出范围可能导致精度下降
3. **多边形顶点**: 多边形顶点应按顺序（顺时针或逆时针）排列
4. **线程安全**: 在多线程环境下使用时，需自行处理同步问题

### 20.3 待优化项
1. 添加SIMD优化提高Morton编码/解码性能
2. 支持GPU加速的网格化算法
3. 添加更多的空间索引结构
4. 改进大区域内存管理

---

## 21. 使用示例

### 21.1 基本编码/解码
```cpp
#include <dqg/DQG3DBasic.h>

// 3D编码
string code = LBH2DQG_str(120.0, 30.0, 100.0, 9);

// 3D解码
PointLBHd center = Decode_3D(code);

// 获取网格包围盒
Gridbox box = Codes2Gridbox(code);
```

### 21.2 线网格化
```cpp
#include <dqg/DQG3DTil.h>
#include <dqg/DQG3DBasic.h>

// 3D线网格化
vector<PointLBHd> line = {{120.0, 30.0, 100.0}, {120.1, 30.1, 200.0}};
vector<string> codes = senham3D_DQG(line[0], line[1], 9);
```

### 21.3 多边形填充
```cpp
#include <dqg/DQG3DPolygon.h>
#include <dqg/GlobalBaseTile.h>

// 初始化基础瓦片
initializeProjectBaseTile(A, B, C, D);
BaseTile tile = getProjectBaseTile();

// 多边形填充
string polygonJson = "[[120.0,30.0],[120.1,30.0],[120.1,30.1],[120.0,30.1]]";
vector<string> codes = getPolygonGridCodes(polygonJson, 1000.0, 0.0, 9, tile);
```

### 21.4 三角面片填充
```cpp
#include <dqg/Extractor.h>

// 三角面片立体模型填充
vector<array<PointLBHd, 3>> faces = {
    {{120.0, 30.0, 100.0}, {120.1, 30.0, 100.0}, {120.0, 30.1, 100.0}},
    {{120.0, 30.0, 200.0}, {120.1, 30.0, 200.0}, {120.0, 30.1, 200.0}}
};

BaseTile tile = getProjectBaseTile();
PolyhedronGridResult result = computePolyhedronGridFill(faces, tile, 10, 2.0, 0);

// 获取结果网格编码
for (const auto& code : result.gridCodes) {
    cout << code << endl;
}
```

### 21.5 OSG模型处理
```cpp
#include <dqg/Extractor.h>

// 从.osgb文件提取三角形
string folderPath = "/path/to/model";
vector<Triangle> triangles;
extractTrianglesFromLevelFiles(folderPath, 15, triangles);

// 坐标转换
convertCoordinatesFromXML(triangles, folderPath, "EPSG:4326");

// 网格化
BaseTile tile = getProjectBaseTile();
vector<string> codes = triangular_multiple(triangles, 9, tile);
```

---

## 22. 附录

### 22.1 WGS84椭球体参数
```
长半径 a = 6378137.0 m
短半径 b = 6356752.314245 m
扁率 f = 0.003352810664747
极半径 c = 6399593.62575849 m
平均半径 R_E = 6371000.0 m
```

### 22.2 网格层级与精度
| 层级 | 网格边长（度） | 网格面积（km²） |
|------|---------------|----------------|
| 0    | 45.0          | 21,300,300      |
| 5    | 88.59375      | 23,398,384      |
| 10   | 89.95605      | 5,713,193       |
| 15   | 89.99863      | 892.68          |
| 20   | 89.99999      | 21.79           |
| 22   | 89.99999      | 1.36            |

### 22.3 八分体定义
DQG将地球表面划分为8个八分体，每个八分体包含3个基本三角形：
- 八分体0-7: 覆盖整个地球表面
- 每个八分体内的局部坐标范围：[0, 90]

### 22.4 Morton编码
- 2D Morton编码: 将(x, y)坐标交织成单个整数
- 3D Morton编码: 将(x, y, z)坐标交织成单个整数
- 使用查找表加速编码/解码过程

---

## 23. 联系和支持

如需技术支持或有疑问，请参考：
- CMakeLists.txt 中的构建说明
- 头文件中的详细注释
- 示例代码（如有）

---

**文档版本**: 1.0  
**最后更新**: 2026-01-12  
**库版本**: 1.0
