# 🏢 OSGB 实景三维网格化与空间多面体填充接口文档

> **最后更新时间**：2026年9月21日

## 1. 概述与核心能力

本模块（`api_multiSource_triangleGrid`）专注于将高精度实景三维数据（倾斜摄影 OSGB 模型、空间三角网、四面体与多面体几何体）转化为统一的 **DQG-3D** 离散空间体素网格，并提供高性能的 PostgreSQL/PostGIS 空间持久化与查询服务。

### 主要特性
- **OSGB目录发现与分块提取**：向下递归发现任意名称的模型分块，分块之间顺序处理，分块内部最多两个LOD层级并行提取；
- **元数据预检与坐标转换**：分块向上查找最近的 `metadata.xml`，最远到 `/app/osgb_data`（包含该目录）；全部预检通过后，使用明确的源坐标系及偏移量转换到 WGS84（`EPSG:4326`）；
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
| `osgbFolder` | `string` | 是 | `/app/osgb_data` 或其内部存在的目录绝对路径；禁止 `..` 路径分量、`~`、越界路径和数据目录内的符号链接。 |
| `level` | `integer` | 是 | 目标网格层级，有效范围为 `0` ~ `21`（城市实景常用 17~20 级）。 |

#### 请求示例
```json
{
  "osgbFolder": "/app/osgb_data/航图数据/单体",
  "level": 18
}
```

#### 响应示例 (成功)
```json
{
  "status": "success",
  "data": {
    "total_triangles_processed": 458920,
    "total_grid_count": 89430,
    "total_discarded_triangles": 124,
    "discard_reason": "total_discarded_triangles为部分三角形顶点坐标超出了region.json边界或数据异常而被丢弃的数量。",
    "message": "处理完成，数据已存入数据库"
  }
}
```

#### 目录发现与元数据边界

支持以下结构：

```text
/app/osgb_data/
└── 航图数据/单体/
    ├── 通信设施/
    │   ├── metadata.xml
    │   ├── JC001/ → .osgb文件
    │   ├── JC002/ → .osgb文件
    │   └── JC003/ → .osgb文件
    ├── 机巢/
    │   ├── metadata.xml
    │   └── 任意分块名/ → .osgb文件
    └── tongxin.zip
```

可以指定 `/app/osgb_data`、`航图数据/单体`、`通信设施` 或 `JC001` 的完整绝对路径。只处理所指定目录的子树；指定 JC001 不会处理同级 JC002。ZIP 不读取、不自动解压。

发现过程向下多层扫描，不按固定目录名称筛选。第一次遇到直接含 OSGB 文件的目录时建立分块；该分块更深目录里的 LOD 文件归入同一分块。遇到嵌套的独立 `metadata.xml` 时建立新的数据集边界，避免不同元数据的几何混合转换。示例中的各 JC 分别选择层级，并共用通信设施目录的元数据。仅有目录、没有直接模型文件的上层不会被当作一个巨大分块重复处理。

每个分块向上找到最近的元数据即停止，查找范围包括 `/app/osgb_data`，但不访问 `/app/metadata.xml`。子目录自己的元数据优先；最近元数据无效时直接报错，不用上级文件替代。请求路径先规范化并检查真实路径归属，扫描不跟随符号链接。

#### LOD选择及无层级回退

每个分块独立执行：存在 L21 或 L20 时仅选实际存在的 L21、L20；二者都不存在时选最高两个层级，只有一个则选一个；无层级文件如 `TD016.osgb` 在没有可选层级或选中层级总提取结果为空时读取。同层所有分片均提取，两个层级最多两个异步任务。扩展名及层级标记的英文字母不区分大小写，中文名称保留原字节。

模型LOD与请求 `level` 不同：LOD决定读取哪批几何，`level` 决定最终DQG网格精度。未新增空间分支补漏、外部PagedLOD引用加载或父子几何去重；不存在仅凭 `Restructure/RTTL` 名称跳过的规则。部分文件失败而其他文件成功时，不会触发整块无层级回退。

#### 元数据错误响应

元数据必须是可读的普通XML文件，包含唯一非空的 `SRS` 和 `SRSOrigin`。`SRSOrigin` 必须是三个逗号分隔的有限数值，`SRS` 必须能由PROJ建立并归一化到 `EPSG:4326` 的转换。字段允许换行；不再默认采用 `EPSG:4528` 或零偏移量来掩盖缺失字段。厂商专用的ENU等表达式只有在现有PROJ转换支持时才可用。

所有分块预检均在连接数据库、建表、记录数据库日志、模型加载及网格入库之前完成。路径无效、无OSGB文件、找不到元数据或元数据无效时返回 HTTP 400：

```json
{
  "status": "error",
  "message": "OSGB输入预检失败: 未找到metadata.xml，分块: /app/osgb_data/航图数据/单体/通信设施/JC001，查找边界: /app/osgb_data"
}
```

预检成功仅确认输入与转换配置可用，不保证模型采用的坐标约定正确或最终覆盖完整。`total_grid_count` 仍为各分块网格去重数量之和，不等于跨分块数据库实际新增行数。后续数据库失败处理及事务方式沿用原逻辑。

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
