# 3D A* 路径规划自适应球形缓冲区机制说明

> **最后更新时间**：2026年9月16日

## 1. 机制演进与概述

> 💡 关于 3D A* 寻路流水线、H值倍数、仿地飞行及接口返回规范，请参阅主文档：
> 👉 [A星三维航路规划算法与接口使用文档.md](./A星三维航路规划算法与接口使用文档.md)


在低空无人机（UAV）三维航路规划中，航迹若紧贴建筑物、山体或禁飞区边缘，极易因 GPS 漂移、风偏扰动或控制误差引发安全事故。

早期版本采用固定的“26 邻居全通检查”（即只要周围 26 个网格有任意一个不可行，当前节点便不可扩展）。这种粗粒度机制存在三大缺陷：
1. **无法适配不同机型尺寸**：大型无人机（翼展数米）与微型四旋翼均使用相同的固定网格数量，无法按真实物理半径进行防护；
2. **割裂了规划与抽稀判定**：A* 搜索阶段用了 26 邻居过滤，而视线平滑抽稀（Thinning）却未以相同缓冲尺度复检，造成抽稀线贴近障碍或抽稀失败；
3. **高层级网格尺度失真**：随着网格层级（Level）变化，单个网格物理尺寸从数米到上百米变化，固定格数会导致高层级过度保守、低层级保护不足。

当前系统已全面升级为**连续物理尺度自适应的 3D 球形缓冲区机制（Spherical Buffer Mask）**，并在 **A* 节点扩展、贪婪平滑抽稀、线段航路统一校验及多级冲突检测** 中实现了全链路统一复用。

---

## 2. 核心数学模型与掩码生成算法

### 2.1 物理参数与网格尺寸映射

系统在请求中接收无人机真实物理半径 `planeRadius`（单位：**米**，有限非负浮点数，默认 `0.75` 米）。

在特定剖分层级 $L$（如 $L=14$ 时 $\text{gridSize} \approx 18.3\text{m}$，详细尺寸由 `getGridSize(level)` 计算得出）下，首先计算三维外切包围盒半长（单位：网格格数）：

$$\text{extendCell} = \left\lceil \frac{\text{planeRadius}}{\text{gridSize}} \right\rceil$$

### 2.2 3D 球形掩码生成逻辑

系统遍历以中心网格为原点、半径为 $\text{extendCell}$ 的离散三维立方体空间：

$$\forall \Delta x, \Delta y, \Delta z \in [-\text{extendCell}, \text{extendCell}]$$

计算该偏移网格相对中心网格的真实空间物理欧氏距离：

$$\text{distance} = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2} \times \text{gridSize}$$

**空间判定条件**：
- 若 $\text{distance} \le \text{planeRadius}$，则将相对偏移坐标 $(\Delta x, \Delta y, \Delta z)$ 加入当前环境上下文的 `sphericalMask` 缓冲掩码集。

### 2.3 保底一格安全缓冲规则（Minimum 1-Grid Guarantee）

当无人机物理半径较小（例如小型多旋翼半径为 $0.75\text{m}$），在粗粒度网格（如 14 级网格尺寸为 $18.3\text{m}$）下，直接计算出的 $\text{distance} \le \text{planeRadius}$ 可能仅包含中心点自身 $(0, 0, 0)$。

为确保飞行绝对安全，代码中内置了**保底扩展机制**：

```cpp
// 摘自 controller/api_airRoute_Astar.cc
if (context.sphericalMask.empty()) {
    context.sphericalMask.push_back({0, 0, 0});
}

// 与安全防护要求保持一致：只要设置了 planeRadius > 0.0，至少扩展周围一格缓冲区（26邻居）
if (planeRadius > 0.0 && context.sphericalMask.size() == 1) {
    context.sphericalMask.clear();
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                context.sphericalMask.push_back({dx, dy, dz});
            }
        }
    }
}
```

- **当 `planeRadius == 0.0`**：仅检测航迹中心自身网格，不产生额外缓冲。
- **当 `planeRadius > 0.0`**：保证至少覆盖周围 26 个相邻三维网格，并随半径增大动态覆盖更大范围的离散球体。

---

## 3. 缓冲机制在各模块中的全链路复用

### 3.1 A* 启发式搜索中的邻居扩展

在 A* 主循环展开某一候选网格节点时：
1. 取出候选节点的目标网格编码，转换为局部行列高坐标 $(col, row, layer)$；
2. 将 `sphericalMask` 中每一个偏移 $(\Delta x, \Delta y, \Delta z)$ 叠加到该坐标上，生成一组**缓冲区三维网格集合**；
3. 将该网格集合送入规则评估引擎（`GridEvaluator`）与矢量障碍检测器：
   - 检查 Redis 中是否有动态气象/禁飞区超标；
   - 检查是否命中永久/临时电子围栏；
   - 检查是否触及 `threshold` 极端阈值；
4. **全通准入法则**：**只有当该候选点对应的球形掩码内所有网格均可安全通行时**，该候选点才被允许计入 OpenSet；一旦缓冲掩码内有任何一个网格被判定为障碍或超出安全阈值，直接跳过并记录原因（如 `"缓冲区检查失败: 邻居网格 ... 触及极端阈值"`）。

### 3.2 航路平滑抽稀（`SmoothAstarPathPlane` / `thinPathGreedy`）

在抽稀阶段尝试用快捷直线 $(A \rightarrow B)$ 替代原有原子折线段时：
1. 采用 3D DDA 算法遍历直线穿过的所有中心网格；
2. 对直线穿越的每个网格，统一沿用当前的 `sphericalMask` 膨胀并查询通行状态；
3. 保证抽稀后的直线段不仅自身无碰撞，而且其球形安全走廊与障碍物、禁飞区保持完全相同的物理安全间距；
4. 杜绝了“原子折线安全，但拉直后航线切角蹭到障碍物”的经典隐患。

### 3.3 航路时空冲突检测（`api_airRoute_lineConflictCheck`）

在外部系统提交已规划好的航线进行冲突检测时：
- `/api/airRoute/lineConflict/check` 与 `/checkFirst` 接口解析请求体中的 `planeRadius`（默认 0.75m）；
- 使用完全相同的球形扩展逻辑，对整条折线沿途的每个时间步网格进行球形走廊安全性验证。

### 3.4 点位缓冲区冲突检测（`api_airRoute_pointConflictCheck`）

- `/api/airRoute/conflictCheck/pointBuffer` 接口支持按指定经纬高坐标和物理半径 `radius` 生成完整的离散三维球形网格集，并与已有网格编码集做集合求交，快速完成点级防空/防撞预警。

---


---

## 4. 与候选点机制的协同

在 3D A* 寻路过程中，空间候选网格的生成与准入同球形缓冲区深度绑定：
- **三维邻居扩展与掩码映射**：当前节点沿 26 邻域试探产生候选网格，并以候选网格为中心平移展开 sphericalMask 离散球形掩码体素簇；
- **全通准入法则**：借助协程（GridCheckAwaiter）向 Redis 批量查询，唯有掩码内所有网格均无障碍且未触碰 	hreshold 极端阈值时，该候选点才允许推入 OpenSet 优先队列；
- **宏观二维绕障候选点联动**：在 3D 细分搜索前，系统通过 PostgreSQL 活动可见图自动注入绕障候选点（mustPass = false），并在抽稀阶段利用必经点区间锁定保护候选点不被切角消除。

> 关于二维活动可见图候选点加载、32/64/128 多阶段筛选及生命周期的完整技术细节，请参阅专门文档：
> 👉 [候选点机制说明.md](./候选点机制说明.md)

---

## 5. 与真高（AGL）防撞的解耦与协同

系统集成了基于 GeoTIFF/DEM 高程数据的真实对地高度（True Height / Above Ground Level, AGL）校验：
- **真高检查范围**：通常设定为 $15\text{m} \sim 120\text{m}$（低空法定适飞层）；
- **解耦设计**：真高检查仅针对**中心航迹线**经过的网格高度与地表高程差进行判定；
- **核心考量**：**不额外将球形缓冲区的上下层网格高度计入真高评估**。避免由于球形掩码向下方膨胀导致的“飞机在 20 米高度巡航，但缓冲掩码下沿到了 10 米，从而被误判为违规低于 15 米”的误杀问题。

---

## 6. API 调用与参数配置

在 `/AstarPathPlane`、`/SmoothAstarPathPlane` 及 `/api/airRoute/lineConflict/check` 接口中，均可通过顶层 JSON 字段直接声明：

```json
{
  "points": [
    [119.970151, 30.523297, 80.0],
    [119.970904, 30.520849, 70.0]
  ],
  "planeRadius": 1.5,
  "speed": 15.0,
  "workHeight": 100.0,
  "level": 14
}
```

### 参数约束

| 参数名 | 类型 | 默认值 | 约束说明 |
| :--- | :--- | :--- | :--- |
| `planeRadius` | `Double` | `0.75` | 必须为非负有限数值（$\ge 0.0$）。传 `0.0` 表示关闭物理外扩缓冲；传正数时自动匹配离散球形掩码并享有保底一格缓冲。 |
| `workHeight` | `Double` | 无（必需） | 相对地面作业高度（米）。规划中心高程 = 地面真实海拔 + `workHeight`。 |
| `level` | `Integer` | `14` | 规划网格层级（通常为 13~16 级，14级单格约 18米）。 |
