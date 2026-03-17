# OSGB网格化接口使用文档

## 目录

- [1. 概述](#1-概述)
- [2. 接口列表](#2-接口列表)
- [3. 核心接口详细说明](#3-核心接口详细说明)
  - [3.1 OSGB文件转网格JSON接口](#31-osgb文件转网格json接口)
  - [3.2 数据库连接测试接口](#32-数据库连接测试接口)
  - [3.3 三角面片空间网格填充接口](#33-三角面片空间网格填充接口)
  - [3.4 四面体网格填充接口](#34-四面体网格填充接口)
  - [3.5 三角形网格化接口](#35-三角形网格化接口)
- [4. 数据库环境配置](#4-数据库环境配置)
- [5. 数据库表结构](#5-数据库表结构)
- [6. 技术实现细节](#6-技术实现细节)
- [7. 使用示例](#7-使用示例)
- [8. 错误处理](#8-错误处理)

---

## 1. 概述

OSGB网格化功能用于将OSGB格式的三维实景数据转换为DQG三维网格编码，并存储到数据库中。

### 主要特性

- **分块处理**：支持大文件分块处理，避免内存溢出
- **坐标转换**：支持OSGB数据坐标系转换（基于metadata.xml）
- **多层级支持**：支持0-21层级的网格编码
- **并发处理**：利用多线程加速数据处理
- **数据入库**：自动将网格数据存储到PostgreSQL数据库
- **日志记录**：完整的处理过程日志记录，便于监控和追踪

### 技术架构

- **后端框架**：Drogon C++ Web框架
- **数据库**：PostgreSQL + PostGIS（用于空间数据存储）
- **网格系统**：DQG-3D
- **坐标系统**：EPSG:4326（WGS84）

---

## 2. 接口列表

| 接口名称 | HTTP方法 | 路径 | 说明 |
|---------|---------|------|------|
| OSGB文件转网格JSON | POST | `/api/multiSource/triangleGrid/osgbToGridJson` | 将OSGB文件转换为网格编码并存入数据库 |
| 数据库测试 | GET | `/api/multiSource/triangleGrid/testDatabase` | 测试数据库连接并检查层级表状态 |
---

## 3. 核心接口详细说明

### 3.1 OSGB文件转网格JSON接口

#### 接口信息

- **URL**: `/api/multiSource/triangleGrid/osgbToGridJson`
- **方法**: `POST`
- **Content-Type**: `application/json`

#### 请求参数

| 参数名 | 类型 | 必填 | 说明 |
|-------|------|------|------|
| osgbFolder | string | 是 | OSGB文件所在目录的绝对路径 |
| level | int | 是 | 网格层级（0-21） |

#### 请求示例

```json
{
  "osgbFolder": "/data/osgb/project1",
  "level": 18
}
```

#### 处理流程

```
1. 参数验证和安全检查
   ├─ 验证JSON格式
   ├─ 检查必需参数
   ├─ 路径遍历攻击防护
   └─ 路径有效性验证

2. 数据库初始化
   ├─ 创建数据库连接
   ├─ 创建/检查目标表（osgbgrid_{level}）
   └─ 记录处理开始日志

3. 分块策略识别
   ├─ 扫描OSGB目录结构
   ├─ 识别数据块目录（Block_+000_+000等格式）
   └─ 确定分块处理方案

4. 顺序处理每个分块
   ├─ 并行提取层级20和21的三角形数据
   ├─ 坐标系转换（基于metadata.xml）
   ├─ 多线程计算网格编码
   ├─ 生成网格数据记录
   └─ 分批入库（每批1000条）
   └─ 清理内存

5. 统计和响应
   ├─ 统计总处理三角形数
   ├─ 统计生成的网格总数
   └─ 返回处理结果
```

#### 响应示例

**成功响应**

```json
{
  "status": "success",
  "data": {
    "total_triangles_processed": 152000,
    "total_grid_count": 45000,
    "message": "处理完成，数据已存入数据库"
  }
}
```

**错误响应**

```json
{
  "status": "error",
  "message": "指定的目录不存在或不是有效目录"
}
```

#### 注意事项

1. **路径安全**：不允许包含`..`或`~`等特殊字符
2. **目录结构**：支持两种模式
   - 包含分块子目录（如`Block_+000_+000`）
   - 直接在根目录处理
3. **层级范围**：level必须在0-21之间
4. **内存管理**：系统会自动清理内存，支持大文件处理
5. **并发策略**：
   - 分块之间：顺序处理（防止内存溢出）
   - 分块内部：多线程处理（提高速度）

---

### 3.2 数据库连接测试接口

#### 接口信息

- **URL**: `/api/multiSource/triangleGrid/testDatabase`
- **方法**: `GET`
- **参数**: 无

#### 功能说明

该接口用于：
1. 测试数据库连接是否正常
2. 检查所有OSGB网格层级表（osgbgrid_0 ~ osgbgrid_21）的存在情况
3. 统计每个表的记录数量
4. 提供数据库状态总览

#### 响应示例

**成功响应**

```json
{
  "status": "success",
  "message": "数据库层级表检查完成，存在 osgbgrid 相关表",
  "total_records": 125000,
  "any_table_exists": true,
  "tables_info": [
    {
      "level": 0,
      "table_name": "osgbgrid_0",
      "exists": false,
      "record_count": 0
    },
    {
      "level": 18,
      "table_name": "osgbgrid_18",
      "exists": true,
      "record_count": 45000
    },
    {
      "level": 19,
      "table_name": "osgbgrid_19",
      "exists": true,
      "record_count": 80000
    }
  ]
}
```

#### 响应字段说明

| 字段 | 类型 | 说明 |
|-----|------|------|
| status | string | 请求状态（success/error） |
| message | string | 描述信息 |
| total_records | int | 所有层级表的总记录数 |
| any_table_exists | bool | 是否存在任何有效的层级表 |
| tables_info | array | 每个层级表的详细信息 |
| tables_info[].level | int | 网格层级 |
| tables_info[].table_name | string | 表名 |
| tables_info[].exists | bool | 表是否存在 |
| tables_info[].record_count | int | 表的记录数（-1表示查询失败） |

---

## 4. 数据库环境配置

在开始使用OSGB网格化功能之前，需要先配置好PostgreSQL数据库环境。本节将详细介绍数据库的安装、配置和初始化步骤。

### 4.1 系统要求

#### 软件版本要求

| 软件 | 最低版本 | 推荐版本 | 说明 |
|-----|---------|---------|------|
| PostgreSQL | 12.0 | 14.0+ | 关系型数据库 |
| PostGIS | 3.0 | 3.3+ | 空间数据扩展 |

#### 硬件要求

- **内存**：至少4GB（推荐8GB+）
- **磁盘空间**：根据数据量而定（建议预留至少100GB）
- **CPU**：多核处理器（建议4核以上）

### 4.2 数据库安装

#### Ubuntu/Debian系统

```bash
# 1. 更新软件包列表
sudo apt update

# 2. 安装PostgreSQL和PostGIS
sudo apt install -y postgresql postgresql-contrib postgis postgresql-14-postgis-3

# 3. 启动PostgreSQL服务
sudo systemctl start postgresql
sudo systemctl enable postgresql

# 4. 检查服务状态
sudo systemctl status postgresql
```

#### CentOS/RHEL系统

```bash
# 1. 安装PostgreSQL仓库
sudo dnf install -y https://download.postgresql.org/pub/repos/yum/reporpms/EL-9-x86_64/pgdg-redhat-repo-latest.noarch.rpm

# 2. 安装PostgreSQL
sudo dnf install -y postgresql14-server postgresql14-contrib postgis34_14

# 3. 初始化数据库
sudo /usr/pgsql-14/bin/postgresql-14-setup initdb

# 4. 启动PostgreSQL服务
sudo systemctl start postgresql-14
sudo systemctl enable postgresql-14
```

### 4.3 数据库创建


```bash
# 1. 切换到postgres用户
sudo -i -u postgres

# 2. 登录到PostgreSQL
psql

# 在psql命令行中执行以下SQL命令：

-- 3. 创建数据库（根据实际需求修改数据库名）
CREATE DATABASE DbName ENCODING 'UTF8';

-- 4. 连接到创建的数据库
\c DbName

-- 5. 创建数据库用户（如果需要）
CREATE USER usrName WITH PASSWORD 'your_secure_password';

-- 6. 授予用户权限
GRANT ALL PRIVILEGES ON DATABASE DbName TO usrName;

-- 7. 退出psql
\q

# 8. 退出postgres用户
exit
```

### 4.4 应用程序配置

配置应用程序连接到数据库。根据实际部署方式，修改相应的配置文件。

#### 配置文件格式

**config.json 示例**：

```json
{
  "db_clients": [
    {
      "name": "default", //数据库指针名称
      "rdbms": "postgresql",
      "host": "localhost",
      "port": 5432,
      "dbname": "DbName", //数据库名称
      "user": "usrName",
      "passwd": "your_secure_password",
      "is_fast": true,
      "connection_number": 10,
      "filename": ""
    }
  ]
}
```


### 4.5 数据库连接测试

#### 使用应用程序测试

通过调用testDatabase接口测试：

```bash
curl -X GET http://localhost:8080/api/multiSource/triangleGrid/testDatabase
```

预期响应：

```json
{
  "status": "success",
  "message": "数据库层级表检查完成，不存在任何 osgbgrid 层级表",
  "total_records": 0,
  "any_table_exists": false,
  "tables_info": [...]
}
```


## 5. 数据库表结构

### 5.1 osgbgrid_{level} 表

每个网格层级对应一张独立的表，表名为`osgbgrid_0`至`osgbgrid_21`。

#### 表结构

| 字段名 | 类型 | 说明 |
|-------|------|------|
| code | NUMERIC(40,0) | 网格编码（主键） |
| center | GEOMETRY(POINTZ, 4326) | 中心点3D坐标 |
| maxlon | DOUBLE PRECISION | 最大经度 |
| minlon | DOUBLE PRECISION | 最小经度 |
| maxlat | DOUBLE PRECISION | 最大纬度 |
| minlat | DOUBLE PRECISION | 最小纬度 |
| top | DOUBLE PRECISION | 网格顶部高程 |
| bottom | DOUBLE PRECISION | 网格底部高程 |
| x | BIGINT | 网格列索引 |
| y | BIGINT | 网格行索引 |
| z | INTEGER | 网格层索引 |
| type | VARCHAR(50) | 数据类型标识（osgb） |

#### 建表SQL


### 5.2 update_log 表

用于记录OSGB网格化处理过程中的关键事件。

#### 表结构

| 字段名 | 类型 | 说明 |
|-------|------|------|
| id | int8 | 主键（自增） |
| module_code | varchar(100) | 模块编码（6表示实景三维数据网格化） |
| module_name | varchar(200) | 模块名称 |
| update_content | text | 更新内容描述 |
| create_time | timestamp(6) | 创建时间（自动） |
| update_time | timestamp(6) | 更新时间（自动） |

#### 索引

- `idx_update_log_module_time`：基于module_code和create_time的复合索引
- `idx_update_log_update_time`：基于module_code和update_time的复合索引

---

## 6. 技术实现细节

### 6.1 分块处理策略

#### 分块识别

系统自动识别OSGB目录下的分块子目录，支持以下命名模式：

- `Block_+000_+000`
- `Tile_+000_+000`
- `ATile_+011_+004`

使用正则表达式：`^[A-Za-z]+_[+-]\d+_[+-]\d+$`

#### 内存管理

- **分块间顺序处理**：确保一个分块完全处理并清理内存后，再处理下一个
- **分块内并行处理**：使用多线程加速三角形数据处理
- **及时清理**：每个分块处理完成后，立即清理相关数据结构

### 6.2 坐标系转换

#### metadata.xml 查找

系统会自动从当前目录向上查找`metadata.xml`文件：

```cpp
const auto findMetadataDir = [](std::filesystem::path current) -> std::filesystem::path {
    current = std::filesystem::absolute(current);
    while (!current.empty()) {
        if (std::filesystem::exists(current / "metadata.xml"))
            return current;
        if (!current.has_parent_path() || current.parent_path() == current)
            break;
        current = current.parent_path();
    }
    return {};
};
```

#### 坐标转换函数

```cpp
convertCoordinatesFromXML(triangles, metadataDir.string(), "EPSG:4326");
```

### 6.3 网格编码生成

#### 三角形网格化算法

使用`triangularGrid`函数将三角形转换为网格单元：

```cpp
std::vector<IJH> triangleGrids = triangularGrid(p1, p2, p3, gridLevelUint);
```

#### 网格编码生成

```cpp
std::string code = IJH2DQG_str(grid.row, grid.column, grid.layer, gridLevelUint);
```

### 6.4 数据库入库策略

#### 分批入库

为避免单次处理过多数据导致内存溢出，系统采用分批入库策略：

- 批次大小：1000条记录/批次
- 冲突处理：使用`ON CONFLICT (code) DO NOTHING`避免重复插入

#### SQL示例

```sql
INSERT INTO osgbgrid_{level} (code, center, maxlon, minlon, maxlat, minlat,
                            top, bottom, x, y, z, type)
VALUES
    (123, ST_GeomFromEWKT('SRID=4326;POINT(116.395 39.905 51.0)'),
     116.398, 116.392, 39.908, 39.902, 55.5, 45.5, 100, 200, 18, 'osgb'),
    ...
ON CONFLICT (code) DO NOTHING;
```

### 6.5 并发控制

#### 线程数配置

```cpp
const unsigned int numThreads = std::max(1u, std::thread::hardware_concurrency() / 2);
```

- 使用CPU核心数的一半作为线程数
- 最少保留1个线程

#### 数据结构

- 使用`std::unordered_set`存储网格编码，自动去重
- 使用`std::atomic`变量进行线程安全的统计

### 6.6 安全措施

#### 路径遍历防护

```cpp
if (folder.find("..") != std::string::npos || folder.find("~") != std::string::npos) {
    // 返回错误
}
```

#### 参数范围验证

```cpp
if (level < 0 || level > 21) {
    // 返回错误
}
```

#### 坐标有效性检查

```cpp
if (std::isnan(t.vertex1.Lng) || std::isnan(t.vertex1.Lat) || std::isnan(t.vertex1.Hgt)) {
    continue; // 跳过无效坐标
}
```

---

## 7. 使用示例

### 7.1 完整的OSGB处理流程

#### 步骤1：检查数据库连接

```bash
curl -X GET http://localhost:8080/api/multiSource/triangleGrid/testDatabase
```
#### 步骤2：规范化OSGB数据目录
##### 正确的OSGB数据目录结构才能激活分块并行处理逻辑，结构应如下所示：
(注：metadata.xml需要与各分块目录在同一层级)

```/data/osgbdata/
├── metadata.xml
├── Block_+000_+001/
│   ├── Block_+000_+001.osgb
│   ├── Block_+000_+001_L14_1.osgb
│   ├── ...
│   ├── Block_+000_+001_L20_1.osgb
│   ├── Block_+000_+001_L20_2.osgb
│   ├── Block_+000_+001_L21_1.osgb
├── Block_+001_+001/
│   ├── Block_+001_+001.osgb
│   ├── ...
│   ├── Block_+001_+001_L21_11.osgb
│   ├── ...
├── Tile_+085_+012
│   ├── Tile_+085_+012.osgb
│   ├── ...
├── ATile_+085_+012_L21_5.osgb
├── ...
```
##### metadata.xml 示例内容：
```xml
<?xml version="1.0" encoding="utf-8"?>
<ModelMetadata version="1">
    <!--Spatial Reference System-->
    <SRS>EPSG:4549</SRS>
    <!--Origin in Spatial Reference System-->
    <SRSOrigin>497429.52130912244,3377072.0553179393,184.34849130019012</SRSOrigin>
    <Texture>
        <ColorSource>Visible</ColorSource>
    </Texture>
</ModelMetadata>

```

#### 步骤3：提交OSGB网格化请求

```bash
curl -X POST http://localhost:8080/api/multiSource/triangleGrid/osgbToGridJson \
  -H "Content-Type: application/json" \
  -d '{
    "osgbFolder": "/data/osgbdata",
    "level": 14
  }'
```


## 8. 错误处理

### 8.1 常见错误码

| HTTP状态码 | 说明 | 可能原因 |
|-----------|------|---------|
| 400 | 请求参数错误 | 缺少必需参数、参数类型错误、参数范围错误 |
| 500 | 服务器内部错误 | 数据库连接失败、文件读取失败、处理异常 |

### 8.2 错误响应格式

```json
{
  "status": "error",
  "message": "具体错误信息描述"
}
```

### 8.3 常见错误及解决方案

#### 错误1：请求体必须为 JSON

**错误响应**

```json
{
  "status": "error",
  "message": "请求体必须为 JSON"
}
```

**原因**：请求Content-Type未设置为`application/json`

**解决方案**：设置正确的请求头

```bash
curl -X POST http://... \
  -H "Content-Type: application/json" \
  -d '...'
```

#### 错误2：缺少必需参数

**错误响应**

```json
{
  "status": "error",
  "message": "缺少必需参数: osgbFolder 或 level"
}
```

**原因**：请求体中缺少必需的参数

**解决方案**：确保请求体包含所有必需参数

```json
{
  "osgbFolder": "/path/to/osgb",
  "level": 18
}
```

#### 错误3：路径包含非法字符

**错误响应**

```json
{
  "status": "error",
  "message": "路径包含非法字符"
}
```

**原因**：路径中包含`..`或`~`等特殊字符

**解决方案**：使用绝对路径，避免特殊字符

#### 错误4：指定的目录不存在或不是有效目录

**错误响应**

```json
{
  "status": "error",
  "message": "指定的目录不存在或不是有效目录"
}
```

**原因**：目录不存在或不可访问

**解决方案**：
- 检查目录路径是否正确
- 确保目录具有读取权限
- 使用`ls`或`find`命令验证目录存在

#### 错误5：level 必须在 0-21 之间

**错误响应**

```json
{
  "status": "error",
  "message": "level 必须在 0-21 之间"
}
```

**原因**：level参数超出允许范围

**解决方案**：使用0-21之间的整数

#### 错误6：无法连接到数据库

**错误响应**

```json
{
  "status": "error",
  "message": "无法连接到数据库"
}
```

**原因**：数据库连接失败

**解决方案**：
- 检查数据库服务是否运行
- 检查数据库配置是否正确
- 检查网络连接

### 8.4 日志查看

系统会在处理过程中记录详细的日志信息，可以通过以下方式查看：

#### 日志位置

日志会输出到应用程序的标准输出，具体位置取决于部署方式：

- **Docker部署**：使用`docker logs`命令查看
- **直接运行**：查看控制台输出

#### 日志级别

- `LOG_INFO`：正常处理信息
- `LOG_WARN`：警告信息（如坐标转换失败）
- `LOG_ERROR`：错误信息

#### 关键日志示例

```
[INFO] 开始处理OSGB网格化 - 目录: /data/osgb/project1, 网格层级: 18
[INFO] 成功创建或确认表存在: osgbgrid_18
[INFO] 检测到 25 个分块数据待处理
[INFO] 正在顺序处理分块: /data/osgb/project1/Block_+000_+000
[INFO] 识别到有效分块目录: Block_+000_+000
[INFO] 分块 /data/osgb/project1/Block_+000_+000 处理完成并已清理内存
[INFO] 所有分块处理完成，共处理三角形: 152000，处理网格总数: 45000
```

---