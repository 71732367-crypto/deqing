# 🚀 A* 三维低空航路规划算法与接口使用文档

> **最后更新时间**：2026年9月16日

## 1. 概述与航路规划全流程管线

本系统（`deqing_serve`）基于 **DQG-3D** 离散空间四叉格网底座与 **C++20 协程**，构建了面向低空无人机（UAV）运行的高性能三维航路规划引擎。

系统实现了从“宏观二维地理硬障碍绕飞”、“微观三维空间启发式搜索”，到“视线平滑抽稀”与“仿地贴地飞行”的全链路自主航路编排。

### 1.1 航路规划端到端处理流水线 (Pipeline)

```
[ 客户端发起 HTTP POST 请求 (/AstarPathPlane 或 /SmoothAstarPathPlane) ]
                               │
                               ▼
 1. 请求参数解析与合法性校验 (points, speed, workHeight, planeRadius, level, mode/condition)
                               │
                               ▼
 2. 空间基准绑定与仿地作业层分配 (Z_target = 地面海拔 + workHeight, 标记 mustPass=true)
                               │
                               ▼
 3. 起飞上升段与降落垂直段构建 (垂直轴逐格生成, 标记 isVertical=true)
                               │
                               ▼
 4. 数据库活动可见图预处理 (VisibilityGraphPlanner: 绕飞禁飞区/围栏, 动态注入绕障点 mustPass=false)
                               │
                               ▼
 5. 微观三维 A* 协程搜索 (aStarPath: 26邻居扩展, H=1.5启发式加速, Redis异步批量环境核验)
                               │
                               ▼
 6. 视线平滑抽稀 (仅 /SmoothAstarPathPlane: thinPathGreedy 严格在 mustPass 区间内分段拉直)
                               │
                               ▼
 7. 统一线段复检与全航程时间积分 (亚秒级飞行时间推算, 关联动态规则)
                               │
                               ▼
[ 格式化响应返回 (原始网格包围盒序列 或 平滑经纬高折点序列) ]
```

---

## 2. 核心算法设计与关键技术

### 2.1 3D 启发式函数与 H 值倍数放大加速 (H-Weight Optimization)

在标准 3D A* 算法中，启发式估计 $h$ 通常采用空间欧几里得距离：

$$h(x, y, z) = \sqrt{(x - x_{goal})^2 + (y - y_{goal})^2 + (z - z_{goal})^2} 	imes 	ext{gridSize}$$

在复杂城市低空环境中，为了防止 A* 在大范围低风险平原区域产生盲目的“地毯式无向发散”，系统引入了 **H 权重倍数动态放大技术**（`hWeight`）：

```cpp
double hWeight = 1.0;
if (evaluator && routeMode == RouteMode::BALANCED) {
    hWeight = 1.5; // 综合模式下放大 1.5 倍
} else if (evaluator && routeMode == RouteMode::SHORTEST) {
    hWeight = 1.5; // 最短路径模式下放大 1.5 倍
}

auto heuristic = [&](int x, int y, int z) {
    double dx = x - ex, dy = y - ey, dz = z - ez;
    return std::sqrt(dx * dx + dy * dy + dz * dz) * gridSize * hWeight;
};
```

- **效果**：将启发式权重大幅放大至 `1.5`，使搜索树强烈向目标牵引，大幅压减搜索空间状态数，算路耗时降低 60% 以上，遇到障碍时绕行收敛更为迅速。

---

### 2.2 26 邻域三维空间扩展与高度变化惩罚

- **空间连通度**：支持每个三维立方体网格向其周围面相邻（6个）、棱相邻（12个）、顶相邻（8个）共 **26 个三维空间方向** 扩展；
- **升降代价抑制**：在移动代价中额外引入 `efficiency` 权重，对垂直高度变化施加惩罚，避免无人机在巡航途中发生无意义的频繁高频起伏。

---

### 2.3 C++20 协程与异步批量核验 (`GridCheckAwaiter`)

传统的路径规划算法在评估每个邻居的动态环境（风速、雨量、信号、围栏）时容易受限于网络 IO 延迟。

本系统深度结合 **C++20 协程**：
- 在展开当前节点的邻居时，将所有候选网格及其对应的球形缓冲网格汇总为 `candidateListForChecker`；
- 通过 `co_await GridCheckAwaiter{evaluator, candidateListForChecker}` 触发非阻塞异步批查询；
- 底层利用 Redis Pipeline 一次性获取成百上千个网格状态，彻底消除了网络往返等待时间。

---

### 2.4 仿地贴地飞行 (Ground Tracking)

- **地面真高提取**：提取每个规划输入点的地面海拔高程（`groundHeight`）；
- **作业层换算**：航路点绝对目标海拔严格计算为：
  $$Height_{target} = GroundHeight + WorkHeight$$
- 转换得到对应的离散网格层索引（`layer`），确保无人机在地形起伏变化时始终恒定保持距地表 $WorkHeight$ 的作业真高，实现自动仿地飞行。

---

### 2.5 起飞爬升段与降落垂直段构建

系统自动为航线无缝缝合起降阶段：
1. **起飞垂直段 (`verticalPath`)**：
   - 从起点地面层（$originalLayer$）垂直升至作业层（$startWorkLayer$）；
   - 航段索引标记为 `pathIndex: 1`，并赋予布尔标识 `isVertical: true`；
2. **降落垂直段 (`landingPath`)**：
   - 从终点作业高空层垂直下降至终点地面层；
   - 航段索引顺延，并同样标记 `isVertical: true`。

---

### 2.6 贪婪视线平滑抽稀 (`thinPathGreedy`) 与必经点保护

在 `/SmoothAstarPathPlane` 接口中，系统对 A* 生成的阶梯状离散网格折线进行平滑抽稀：

1. **快捷线尝试**：使用 3D DDA 射线检测从前向后跳跃探测，尝试用直线替代中间一段原子折线；
2. **替换前提（安全与代价双核验）**：
   - 快捷线必须几何上无碰撞、穿透的网格及其球形掩码全部合规且不破极端阈值；
   - 快捷直线的累积环境风险代价**不得劣于原原子路径**；
3. **必经点区间锁定 (`mustPassIndices`)**：
   - 抽稀严格在前端指定的必经点区间之间独立进行，绝不跨越必经点合并，确保航路严格途经特定空域中继点与可见图绕障点。

---

### 2.7 地形真高适飞层核验 (`enableTrueHeightCheck`)

当启用 `enableTrueHeightCheck: true` 时：
- 系统读取 GDAL 单例加载的 DEM 高程（`dem.tif`）；
- 严格保证中心航迹经过的每一个网格距地表净空高度处于 **$15	ext{m} \sim 120	ext{m}$** 法定低空适飞区间内；
- 避免因低空障碍或地面建筑物遮挡导致撞地风险。

---

## 3. 两大致电 API 详尽规范

服务提供两个核心航路规划端点，根据业务场景灵活调用：

| 接口端点 | HTTP 方法 | 适用场景 | 返回数据特征 |
| :--- | :--- | :--- | :--- |
| **`/AstarPathPlane`** | `POST` | 航迹仿真、网格调试、空域占用分析、地面监控系统 | 返回沿途每一个立体网格的详细三维几何包围盒、到达时刻、阶段标记等全量元数据 |
| **`/SmoothAstarPathPlane`** | `POST` | 实际无人机飞控执飞、航点上传地面站、大屏三维航迹渲染 | 视线拉直平滑，仅返回关键折转点的空间坐标 `[[lon, lat, height], ...]` |

---

### 3.1 请求体通用规范 (Request Body)

两个接口接收完全相同的请求体结构：

```json
{
  "points": [
    [119.970151, 30.523297, 80.0],
    [119.969987, 30.521755, 90.0],
    [119.970904, 30.520849, 70.0]
  ],
  "planeRadius": 0.75,
  "speed": 15.0,
  "workHeight": 100.0,
  "level": 14,
  "mode": "balanced",
  "startTime": 1776049932000,
  "enableTrueHeightCheck": true,
  "condition": {
    "wdh_11": {
      "windSpeed": {
        "threshold": ">13.8",
        "value": [
          { "range": "[0,5]", "cost": 0.0 },
          { "range": "(5,10]", "cost": 0.3 },
          { "range": "(10,13.8]", "cost": 0.8 }
        ]
      }
    }
  }
}
```

#### 字段约束说明

| 字段名 | 类型 | 是否必选 | 默认值 | 约束与说明 |
| :--- | :--- | :--- | :--- | :--- |
| `points` | `Array<[lon,lat,h]>` | **必选** | 无 | 三维航路点数组，至少包含 2 个点（起点与终点）。经度 $[-180, 180]$，纬度 $[-90, 90]$，高度必须高于基准底高。 |
| `workHeight` | `Double` | **必选** | 无 | 相对地面作业真高（米）。规划巡航海拔 = 地面标高 + `workHeight`。 |
| `speed` | `Double` | 可选 | `15.0` | 飞行巡航速度（米/秒），必须为 $>0$ 的有限数值。用于精确推算航线沿途各网格到达时间戳。 |
| `planeRadius` | `Double` | 可选 | `0.75` | 无人机物理半径（米），$\ge 0$。自动计算 3D 球形防护走廊，享有保底一格缓冲。 |
| `level` | `Integer` | 可选 | `14` | 规划网格层级（通常为 13~16 级，14 级单格尺寸约 18.3 米）。 |
| `mode` / `route_type` | `String` | 可选 | `"original"` | 调优模式：`"shortest"`（最短）、`"safest"`（最安全）、`"balanced"`（均衡最优）、`"original"`（原始无代价 A*）。 |
| `startTime` | `Long/Double` | 可选 | 当前北京时间 | 起飞时刻时间戳（毫秒或秒均可，大于 10 位自动转为秒）。 |
| `enableTrueHeightCheck` | `Boolean` | 可选 | `false` | 是否开启 DEM 真实地形 15~120m 适飞高度检查。 |
| `condition` | `Object` | 可选 | 空对象 | 动态多因子权重与阈值规则覆盖对象（详见《多权重算路.md》）。 |

---

### 3.2 `/SmoothAstarPathPlane` 响应体示例 (平滑航线)

```json
{
  "results": {
    "success": true,
    "path": [
      [119.970151, 30.523297, 80.0],
      [119.970151, 30.523297, 180.0],
      [119.969820, 30.521800, 190.0],
      [119.970904, 30.520849, 170.0],
      [119.970904, 30.520849, 70.0]
    ]
  }
}
```

---

### 3.3 `/AstarPathPlane` 响应体示例 (全量网格序列)

```json
{
  "results": {
    "success": true,
    "path": [
      {
        "center": [119.970151, 30.523297, 80.0],
        "minlon": 119.969241, "maxlon": 119.971061,
        "minlat": 30.522387,  "maxlat": 30.524207,
        "bottom": 70.85,      "top": 89.15,
        "code": "14-10243048",
        "interopCode": "LGC1:deqing:14:14-10243048",
        "arrivalTime": 1776049932,
        "pathIndex": 1,
        "isStart": true,
        "isVertical": true
      },
      {
        "center": [119.970151, 30.523297, 180.0],
        "code": "14-10243053",
        "interopCode": "LGC1:deqing:14:14-10243053",
        "arrivalTime": 1776049938,
        "pathIndex": 1,
        "isVertical": true
      },
      {
        "center": [119.970904, 30.520849, 70.0],
        "code": "14-10253012",
        "interopCode": "LGC1:deqing:14:14-10253012",
        "arrivalTime": 1776049964,
        "pathIndex": 3,
        "isEnd": true,
        "isVertical": true
      }
    ]
  }
}
```

#### 元素属性字段清单

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `center` | `[lon, lat, h]` | 当前网格三维物理中心点大地坐标 |
| `minlon` / `maxlon` | `Double` | 网格水平经度包围盒边界 |
| `minlat` / `maxlat` | `Double` | 网格水平纬度包围盒边界 |
| `bottom` / `top` | `Double` | 网格空间天顶/地底海拔高程（米） |
| `code` | `String` | 原生 DQG-3D 局部网格编码 |
| `interopCode` | `String` | 具备行业跨域互操作标准的统一局部编码（`LGC1:{region}:{level}:{code}`） |
| `arrivalTime` | `Integer` | 飞行器抵达该网格的累计推算绝对时间戳（秒） |
| `pathIndex` | `Integer` | 所属航段序号（从 1 开始递增） |
| `isVertical` | `Boolean` | 是否属于起降阶段的垂直升降航段 |
| `isStart` / `isEnd` | `Boolean` | 是否为航线起点 / 终点网格 |
| `isWaypoint` | `Boolean` | 是否为用户或可见图指定的任务中继折转航点 |

---

### 3.4 失败响应规范

当发生无解、碰撞或参数非法时，接口返回 HTTP `400 Bad Request`，并在 `reason` 字段中给出具体原因：

```json
{
  "results": {
    "success": false,
    "path": [],
    "reason": "缓冲区检查失败: 邻居网格 14-10243049 触及极端阈值限制(threshold)"
  }
}
```

---

## 4. 典型故障排查手册

| 报错信息 | 触发根因 | 排查与解决策略 |
| :--- | :--- | :--- |
| `坐标值不合法` | 经纬度超出 $[-180, 180]$ 或 $[-90, 90]$，或高度低于基准高 | 检查传入航路点经纬度是否反写，确认高度为海拔高度 |
| `缺少必需参数: workHeight` | 未传入相对地面作业高度 | 在 JSON 根节点显式提供 `"workHeight": 100.0` |
| `planeRadius 必须是非负有限数值` | 传入负数或 NaN / Infinity | 确保 `planeRadius >= 0.0` |
| `活动可见图中未找到可用绕行路径` | 起点或终点直接落在了永久禁飞区或硬围栏内部 | 调整起终点位置，将其移出红色禁飞区 |
| `触及极端阈值限制(threshold)` | 航线必经之路上某网格风速/气象超出了设置的硬安全门限 | 调整阈值或更换算路时间，避开恶劣天气峰值 |
| `no_path_found` | 物理狭窄通道被两端禁飞区或障碍物挤占，且缓冲半径过大 | 适当调小 `planeRadius` 或改用更细层级（如 level 15） |

---

## 5. A* 航路规划体系文档关联

为深入了解 A* 引擎周边子系统，请协同参阅以下专题文档：

- 🛡️ **三维安全走廊**：👉 [A星路径规划缓冲区机制说明.md](./A星路径规划缓冲区机制说明.md)（深入了解球形掩码离散生成与保底缓冲）
- ⚖️ **多因子权重与阈值**：👉 [多权重算路.md](./多权重算路.md)（因子数据字典、模式权重矩阵与 threshold 规则）
- 📍 **可见图与拓扑中继点**：👉 [候选点机制说明.md](./候选点机制说明.md)（活动可见图多阶段搜索、必经点锁定与抽稀保护）
- 🌐 **系统全局服务概览**：👉 [项目总体架构.md](./项目总体架构.md)（全量 API 接口清单与部署架构）
