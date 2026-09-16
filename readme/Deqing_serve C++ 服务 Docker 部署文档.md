# 🐳 Deqing_serve C++ 服务 Docker 生产环境部署与运维手册

> **最后更新时间**：2026年9月16日

## 1. 概述与部署架构

`deqing_serve` 采用企业级容器化方案进行构建与部署。基于 **Ubuntu 24.04 LTS**，在容器安全上实现了 **只读文件系统 (`read_only: true`)**、**最小特权原则 (`cap_drop: ALL`)**、**非 root 用户运行 (`appuser`)** 以及精细化的 tmpfs 挂载。

### 1.1 系统组件拓扑

```
[ 宿主机 (Host) ]
       │  (Port: 9990)
       ▼
[ Docker 容器 (deqing_service, Port: 9997) ]
       ├── /app/config.json        (只读挂载: 核心配置)
       ├── /app/region.json        (只读挂载: 基准瓦片与四至)
       ├── /app/weight.json        (只读挂载: 因子权重与阶梯规则)
       ├── /app/geo_data/          (只读挂载: DEM 高程 TIFF 文件)
       ├── /app/logs/              (读写挂载: 服务运行日志)
       ├── /app/data/              (读写挂载: OSGB 实景模型瓦片)
       └── /app/uploads/           (读写挂载: 用户上传目录)
       │
       ├───► PostgreSQL (宿主机/外部数据库: osgbgrid_* 表 / 围栏 / 可见图)
       └───► Redis (时空缓存: 气象 / 通信 / 动态环境数据)
```

---

## 2. 部署目录结构标准

在宿主机生产服务器上，推荐部署目录结构如下：

```
/opt/deqing_serve/
├── docker-compose.yml
├── deployment/
│   ├── config/
│   │   ├── config.json          # Drogon、数据库与连接池配置
│   │   ├── region.json          # 德清作业区四至与基准范围
│   │   └── weight.json          # 算路因子权重与阈值规则
│   ├── geo_data/
│   │   └── local_mask.tif       # 局部 DEM 数字高程模型文件
│   ├── logs/                    # 运行日志输出目录
│   ├── data/                    # OSGB 等静态三维数据目录
│   └── uploads/                 # 文件上传目录
```

---

## 3. 核心配置文件详解

### 3.1 `docker-compose.yml`

```yaml
services:
  deqing_serve:
    image: deqing_serve:latest    # 替换为实际构建或导入的镜像 Tag
    container_name: deqing_service
    restart: unless-stopped

    # 安全配置：只读根文件系统与非 root 用户
    read_only: true
    cap_drop:
      - ALL
    cap_add:
      - NET_BIND_SERVICE
    user: appuser

    # 临时文件系统 (内存盘)
    tmpfs:
      - /tmp:size=64M,mode=1777
      - /run:size=64M,mode=1777
      - /app/tmp:size=32M,mode=1777

    security_opt:
      - no-new-privileges:true
      - apparmor:docker-default

    # 端口映射 (宿主机 9990 -> 容器服务端口 9997)
    ports:
      - "9990:9997"

    environment:
      - TZ=Asia/Shanghai
      - LOG_LEVEL=WARN
      - SECURITY_MODE=ENABLED
      - LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu
      # 指定 DEM 高程 TIFF 文件路径
      - ELEVATION_TIFF_PATH=/app/geo_data/local_mask.tif

    volumes:
      # 配置文件只读挂载
      - ./deployment/config/config.json:/app/config.json:ro
      - ./deployment/config/region.json:/app/region.json:ro
      - ./deployment/config/weight.json:/app/weight.json:ro

      # 高程数据只读挂载
      - ./deployment/geo_data:/app/geo_data:ro

      # 业务读写目录
      - ./deployment/logs:/app/logs:rw
      - ./deployment/data:/app/data:rw
      - ./deployment/uploads:/app/uploads:rw

    networks:
      - deqing_network

    # 容器健康检查
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:9997/"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

networks:
  deqing_network:
    driver: bridge
```

---

### 3.2 `region.json` - 区域与基准瓦片配置

定义德清低空作业区的 WGS84 经纬度四至与高程层：

```json
{
  "region": {
    "name": "德清县研究区域",
    "bounds": {
      "southwest": { "longitude": 119.843672, "latitude": 30.499532 },
      "northwest": { "longitude": 119.843672, "latitude": 30.617562 },
      "northeast": { "longitude": 120.112892, "latitude": 30.617562 },
      "southeast": { "longitude": 120.112892, "latitude": 30.499532 }
    },
    "height": {
      "bottom": 0,
      "top": 600
    }
  }
}
```

---

### 3.3 `config.json` - 框架与中间件配置

```json
{
  "listeners": [
    {
      "address": "0.0.0.0",
      "port": 9997,
      "https": false
    }
  ],
  "db_clients": [
    {
      "name": "default",
      "rdbms": "postgresql",
      "host": "192.168.3.6",
      "port": 5432,
      "dbname": "grid_dev",
      "user": "postgres",
      "passwd": "your_db_password"
    }
  ],
  "redis_clients": [
    {
      "name": "default",
      "host": "127.0.0.1",
      "port": 6379,
      "passwd": ""
    }
  ],
  "app": {
    "threads_num": 4,
    "log": {
      "log_level": "WARN",
      "display_local_time": true
    }
  },
  "custom_config": {
    "pg_table_name": "osgbgrid_${level}",
    "tiff_file_path": "/app/geo_data/local_mask.tif"
  }
}
```

---

## 4. 服务启动流程与启动自检机制

服务在调用 `drogon::app().run()` 之前，会执行两项关键异步自检：

1. **TIFF 高程数据初始化检查**：
   - 系统尝试加载 `custom_config.tiff_file_path` 或环境变量 `ELEVATION_TIFF_PATH` 指定的 DEM 瓦片；
   - 若加载成功，日志提示：`"TIFF 高程文件加载成功！真高防撞系统已激活。"`；
   - 若文件缺失，系统以 `LOG_WARN` 警告并平滑切入降级状态（不阻止服务启动，但真高检查退化）。
2. **Redis 连接异步探活看门狗**：
   - 发送异步 `PING` 命令；
   - 若 3 秒内未收到 Pong 回复，看门狗输出告警，便于运维快速定位缓存网络抖动。

---

## 5. 常用运维与部署命令

### 5.1 首次部署与启动

```bash
# 1. 确保目录存在且设置合适权限 (appuser UID 通常为 1000)
mkdir -p deployment/{logs,data,uploads,geo_data,config}
chmod -R 777 deployment/logs deployment/data deployment/uploads

# 2. 检查 docker-compose 配置正确性
docker compose config

# 3. 启动容器并在后台运行
docker compose up -d

# 4. 观察实时启动日志与自检输出
docker compose logs -f
```

### 5.2 状态监测与健康验证

```bash
# 查看容器健康状态 (应当显示 healthy)
docker compose ps

# 宿主机发起健康探测 (返回 200 OK)
curl -I http://localhost:9990/

# 查看资源占用 (CPU / 内存 / 网络IO)
docker stats deqing_service
```

### 5.3 镜像更新与服务重启

```bash
# 重新加载配置并平滑重启
docker compose restart

# 更新镜像并重新创建容器
docker compose pull
docker compose up -d --force-recreate
```
