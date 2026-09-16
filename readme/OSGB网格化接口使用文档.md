# 🏢 OSGB 实景三维网格化与空间多面体填充接口文档

> **最后更新时间**：2026年9月16日

## 1. 概述与核心能力

本模块（`api_multiSource_triangleGrid`）专注于将高精度实景三维数据（倾斜摄影 OSGB 模型、空间三角网、四面体与多面体几何体）转化为统一的 **DQG-3D** 离散空间体素网格，并提供高性能的 PostgreSQL/PostGIS 空间持久化与查询服务。

### 主要特性
- **倾斜摄影分块并行解析**：自动识别 `Block_*`、`Tile_*` 等 OSGB 瓦片分块目录，多线程并行提取三角面片；
- **自动坐标系投影校正**：递归向上查找 `metadata.xml` 文件，自动完成局部大地坐标向 WGS84（`EPSG:4326`）的精确投影；
- **边界防下溢过滤保护**：内置基准瓦片四至（2D Bounds）越界检测，自动丢弃越界三角形，杜绝局部网格索引计算时的无符号整型下溢异常；
- **多层级体素化填充**：支持单个空间三角面片、四面体（三棱锥）以及任意闭合/非闭合多面体的体素网格化；
- **批量入库与冲突去重**：以 1000 条/批的高速事务并发写入 PostgreSQL，使用 `ON CONFLICT (code) DO NOTHING` 保证幂等性；
- **全生命周期时区审计**：操作记录自动入库 `update_log` 表，并强制使用 `Asia/Shanghai` 时区时间戳。

---

## 2. 接口清单汇总

| 接口名称 | HTTP 方法 | 路径 | 功能说明 |
| :--- | :--- | :--- | :--- |
| **OSGB 目录转网格入库** | `POST` | `/api/multiSource/triangleGrid/osgbToGridJson` | 批量将本地 OSGB 实景瓦片解析体素化并入库 |
| **实景数据库状态测试** | `GET` | `/api/multiSource/triangleGrid/testDatabase` | 检查 PostgreSQL 连接与各层级网格表存在状态 |
| **单三角面片网格填充** | `POST` | `/api/multiSource/triangleGrid/fillTriangleWithCubes` | 计算单个空间三维三角形覆盖的所有 DQG 网格 |
| **四面体网格填充** | `POST` | `/api/multiSource/triangleGrid/fillTetraWithCubes` | 计算三维四面体/棱锥立体内部覆盖的所有网格 |
| **任意多面体网格填充** | `POST` | `/api/multiSource/triangleGrid/fillPolyhedronWithCubes`| 计算由多三角面围成的多面体空间体素网格 |

---

## 3. 详细接口规范

### 3.1 OSGB 目录转网格入库 (`/osgbToGridJson`)

#### 请求方式
- `POST /api/multiSource/triangleGrid/osgbToGridJson`
- `Content-Type: application/json`

#### 请求参数 (JSON)

| 字段 | 类型 | 必选 | 说明与约束 |
| :--- | :--- | :--- | :--- |
| `osgbFolder` | `string` | 是 | 存放 OSGB 数据的服务器绝对路径，禁止包含 `..` 或 `~` 特殊路径字符。 |
| `level` | `integer` | 是 | 目标网格层级，有效范围为 `0` ~ `21`（城市实景常用 17~20 级）。 |

#### 请求示例
```json
{
  "osgbFolder": "/app/data/deqing_city_osgb",
  "level": 18
}
```

#### 响应示例 (成功)
```json
{
  "status": "success",
  "message": "OSGB 转网格成功并已存入数据库",
  "data": {
    "totalTriangles": 458920,
    "discardedTriangles": 124,
    "uniqueGrids": 89430,
    "tableName": "osgbgrid_18",
    "elapsedSeconds": 14.5
  }
}
```

---

### 3.2 数据库连接与表状态测试 (`/testDatabase`)

#### 请求方式
- `GET /api/multiSource/triangleGrid/testDatabase`

#### 响应示例 (成功)
```json
{
  "status": "success",
  "message": "数据库连接正常",
  "data": {
    "existingTables": [
      "osgbgrid_14",
      "osgbgrid_16",
      "osgbgrid_18"
    ],
    "updateLogAvailable": true
  }
}
```

---

### 3.3 单三角面片网格填充 (`/fillTriangleWithCubes`)

#### 请求参数 (JSON)
```json
{
  "v1": [119.97015, 30.52329, 50.0],
  "v2": [119.97055, 30.52380, 55.0],
  "v3": [119.97120, 30.52290, 48.0],
  "level": 18
}
```

#### 响应示例
```json
{
  "status": "success",
  "gridCount": 42,
  "gridCodes": [
    "18-10243048",
    "18-10243049"
  ]
}
```

---

### 3.4 四面体体素填充 (`/fillTetraWithCubes`)

#### 请求参数 (JSON)
```json
{
  "points": [
    [119.9700, 30.5230, 20.0],
    [119.9710, 30.5230, 20.0],
    [119.9705, 30.5240, 20.0],
    [119.9705, 30.5235, 60.0]
  ],
  "level": 18
}
```

#### 约束说明
- `points` 数组长度必须严格等于 4，分别代表四面体的 4 个空间三维顶点 `[经度, 纬度, 高度]`。

---

### 3.5 任意不规则多面体体素填充 (`/fillPolyhedronWithCubes`)

#### 请求参数 (JSON)
```json
{
  "triangles": [
    {
      "v1": [119.9700, 30.5230, 20.0],
      "v2": [119.9710, 30.5230, 20.0],
      "v3": [119.9705, 30.5235, 60.0]
    },
    {
      "v1": [119.9710, 30.5230, 20.0],
      "v2": [119.9705, 30.5240, 20.0],
      "v3": [119.9705, 30.5235, 60.0]
    }
  ],
  "level": 18
}
```

---

## 4. 数据库表结构与持久化模型

### 4.1 网格数据表 (`osgbgrid_{level}`)

表名规则通过 `config.json` 的 `custom_config.pg_table_name` 定义（默认为 `osgbgrid_${level}`）：

| 字段名 | 类型 | 约束 | 说明 |
| :--- | :--- | :--- | :--- |
| `code` | `varchar(100)` | `PRIMARY KEY` | 唯一 DQG-3D 空间网格编码 |
| `center` | `geometry(PointZ, 4326)` | `NOT NULL` | PostGIS 空间点几何对象（经、纬、高） |
| `maxlon` | `float8` | `NOT NULL` | 网格包围盒东边界经度 |
| `minlon` | `float8` | `NOT NULL` | 网格包围盒西边界经度 |
| `maxlat` | `float8` | `NOT NULL` | 网格包围盒北边界纬度 |
| `minlat` | `float8` | `NOT NULL` | 网格包围盒南边界纬度 |
| `top` | `float8` | `NOT NULL` | 网格包围盒天顶高度（米） |
| `bottom` | `float8` | `NOT NULL` | 网格包围盒地底高度（米） |
| `x` | `int8` | `NOT NULL` | 局部网格列索引 (Column) |
| `y` | `int8` | `NOT NULL` | 局部网格行索引 (Row) |
| `z` | `int8` | `NOT NULL` | 局部网格层索引 (Layer) |
| `type` | `varchar(50)` | 默认 `'osgb'` | 数据来源标识 |

---

### 4.2 更新日志表 (`update_log`)

记录实景网格数据与各模块更新历史：

| 字段名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `id` | `bigserial PRIMARY KEY` | 自增主键 |
| `module_code` | `varchar(100)` | 模块编码（`6` 表示实景三维数据网格化） |
| `module_name` | `varchar(200)` | 模块名称（如 `"实景三维网格化"`） |
| `update_content` | `text` | 详细操作描述（记录处理三角面数、网格数、耗时等） |
| `create_time` | `timestamp(6)` | 创建时间（自动写入 `CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Shanghai'`） |
| `update_time` | `timestamp(6)` | 更新时间（自动写入 `CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Shanghai'`） |

---

## 5. 关键技术实现细节

### 5.1 越界三角面片防下溢过滤 (Boundary Clipping)

在将实景模型三角面片映射到局部格网坐标（`localRowColHeiNumber`）时，如果顶点经纬度超出基准瓦片范围（`baseTile`），经纬度差值为负数将导致 `uint32_t` 发生严重的整型下溢，计算出极其巨大的行列号破坏数据。

当前系统内置了防御检测：

```cpp
auto isOutside = [&](const PointLBHd& p) {
    return p.Lng < baseTile.west || p.Lng > baseTile.east ||
           p.Lat < baseTile.south || p.Lat > baseTile.north;
};

// 只要任一顶点位于研究区域边界外，即安全过滤丢弃，并统计 localDiscarded 数量
if (isOutside(t.vertex1) || isOutside(t.vertex2) || isOutside(t.vertex3)) {
    localDiscarded++;
    continue;
}
```

> [!NOTE]
> 高度（`Hgt`）不参与越界丢弃判定，避免由于配置文件中高度范围偏小造成合法高空建筑面片被误杀。

---

### 5.2 OSGB 数据规范化目录结构

系统支持的 OSGB 目录规范如下（`metadata.xml` 必须存在于分块同级目录）：

```
/data/osgbdata/
├── metadata.xml                <- 必须包含 SRS 与 SRSOrigin
├── Block_+000_+001/
│   ├── Block_+000_+001.osgb
│   └── Block_+000_+001_L18_1.osgb
├── Tile_+001_+001/
│   ├── Tile_+001_+001.osgb
│   └── Tile_+001_+001_L18_1.osgb
```

`metadata.xml` 核心内容示例：
```xml
<?xml version="1.0" encoding="utf-8"?>
<ModelMetadata version="1">
    <SRS>EPSG:4549</SRS>
    <SRSOrigin>497429.5213, 3377072.0553, 184.3484</SRSOrigin>
</ModelMetadata>
```
