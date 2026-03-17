# Deqing\_serve C++ 服务 Docker 部署文档



***


**最后更新**: 2026 年 1 月 12 日

**适用环境**: 生产环境

**维护团队**: CUMTB



***

## 📋 依赖服务



| 服务组件               | 服务器地址           | 端口配置                   | 说明                |
| ------------------ |-----------------|------------------------| ----------------- |
| **C++ 服务**         | HSOT            | PORT (宿主机) → 9997 (容器) | Deqing\_serve 主服务 |
| **PostgreSQL 数据库** | HOST            | PORT                   | 应用数据库             |
| **Redis 缓存**       | HOST            | PORT                   | 数据缓存服务            |





***


## 🖥️ 1. Deqing\_serve C++ 服务部署

### 1.1 准备工作与目录规划

#### 1.1.1 创建部署目录结构

在宿主机服务器上创建以下目录结构：
```
\# 创建主目录

cd path/to/yourprojects

mkdir -p deqing_serve/deployment && cd deqing_serve/deployment

\# 创建子目录

mkdir -p config data logs uploads
```

#### 1.1.2 目录结构说明



``` 项目目录结构
deqing_serve/
│
├── deployment/                # 部署文件目录
│   ├── config/                # 配置文件目录        
│   │   ├── config.json        # 核心数据库及应用配置
│   │   └── region.json        # 项目区域配置
│   ├── data/                  # 静态数据存放目录
│   ├── logs/                  # 日志目录
│   └── uploads/               # 服务缓存目录
├── docker-compose.yml         # Docker Compose 编排文件
└── deqing_serve.tar.gz        # Docker 镜像包
```

#### 1.1.3 设置目录权限 ⚠️

**重要**: 由于容器内使用非 Root 用户 (appuser) 且文件系统为只读模式，必须为数据目录设置正确的写入权限：

``` 设置权限
# 在宿主机Deqing_serve目录执行以下命令

chmod -R 777 deployment/

```

### 1.2 配置文件编写

#### 1.2.1 config.json - 主配置文件

创建并编辑 `config/config.json` 文件：


``` config.json
{
    "listeners": [
        {
            "address": "0.0.0.0",
            "port": 8080,
            "https": false
        }
    ],
    // 数据库配置
    "db_clients": [
        {
            "name": "default",
            "rdbms": "postgresql",
            "host": "xxx.xxx.xxx.xxx",  // 数据库主机地址
            "port": 5432,  // 数据库端口
            "dbname": "xxx",  // 数据库名称
            "user": "postgres",  // 数据库用户名
            "passwd": "postgres",  // 数据库密码
            "is_fast": false,  // 是否使用快速模式
            "number_of_connections": 5,  // 连接池大小
            "timeout": 10.0,  // 连接超时时间
            "auto_batch": false  // 是否启用自动批处理
        }
    ],
    "redis_clients": [
        {
            "name": "default",
            "host": "xxx.xxx.xxx.xxx",  // Redis 主机地址
            "port": 6379,  // Redis 端口
            "username": "",  // Redis 用户名
            "passwd": "",  // Redis 密码
            "db": 0,  // Redis 数据库索引
            "is_fast": false,  // 是否使用快速模式
            "number_of_connections": 5,  // 连接池大小
            "timeout": -1.0  // 连接超时时间
        }
    ],
    "app": {
        "number_of_threads": 0,
        "enable_session": false,
        "session_timeout": 0,
        "session_same_site": "Null",
        "session_cookie_key": "JSESSIONID",
        "session_max_age": -1,
        "document_root": "/app/public",
        "home_page": "index.html",
        "use_implicit_page": true,
        "implicit_page": "index.html",
        "upload_path": "/app/uploads",
        "file_types": [
            "gif",
            "png",
            "jpg",
            "js",
            "css",
            "html",
            "ico",
            "swf",
            "xap",
            "apk",
            "cur",
            "xml",
            "webp",
            "svg"
        ],
        "mime": {},
        "locations": [
            {
                "default_content_type": "text/plain",
                "alias": "",
                "is_case_sensitive": false,
                "allow_all": true,
                "is_recursive": true,
                "filters": []
            }
        ],
        "max_connections": 100000,
        "max_connections_per_ip": 0,
        "load_dynamic_views": false,
        "dynamic_views_path": [
            "/app/views"
        ],
        "dynamic_views_output_path": "/app/build/views",
        "json_parser_stack_limit": 1000,
        "enable_unicode_escaping_in_json": true,
        "float_precision_in_json": {
            "precision": 0,
            "precision_type": "significant"
        },
        "log": {
            "use_spdlog": true,
            "log_path": "",
            "logfile_base_name": "",
            "log_size_limit": 100000000,
            "max_files": 0,
            "log_level": "WARN",   // 日志级别,可选值: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF，生成环境建议设置为WARN或更高
            "display_local_time": true
        },
        "run_as_daemon": false,
        "handle_sig_term": true,
        "relaunch_on_error": false,
        "use_sendfile": true,
        "use_gzip": true,
        "use_brotli": false,
        "static_files_cache_time": 5,
        "idle_connection_timeout": 60,
        "server_header_field": "Drogon",
        "enable_server_header": true,
        "enable_date_header": true,
        "keepalive_requests": 0,
        "pipelining_requests": 0,
        "gzip_static": true,
        "br_static": true,
        "client_max_body_size": "1M",
        "client_max_memory_body_size": "64K",
        "client_max_websocket_message_size": "128K",
        "reuse_port": false,
        "enabled_compressed_request": false,
        "enable_request_stream": false
    },
    "plugins": [
        {
            "name": "drogon::plugin::PromExporter",
            "dependencies": [],
            "config": {
                "path": "/metrics"
            }
        },
        {
            "name": "drogon::plugin::AccessLogger",
            "dependencies": [],
            "config": {
                "use_spdlog": true,
                "log_path": "",
                "log_format": "",
                "log_file": "",
                "log_size_limit": 0,
                "use_local_time": true,
                "log_index": 0
            }
        }
    ],
    "custom_config": {
        "pg_table_name": "osgbgrid_${level}"  // PostgreSQL表名
    }
}

```

#### 1.2.2 region.json - 区域配置文件

创建并编辑 `config/region.json` 文件：

``` region.json
{
  "region": {
    "name": "德清县",  // 区域名称
    // 区域边界坐标
    "bounds": {
      "southwest": {
        "longitude": 119.8436725825645226,
        "latitude": 30.4995322056211791
      },
      "northwest": {
        "longitude": 119.8436725825645226,
        "latitude": 30.6175621573347883
      },
      "northeast": {
        "longitude": 120.1128929038345632,
        "latitude": 30.6175621573347883
      },
      "southeast": {
        "longitude": 120.1128929038345632,
        "latitude": 30.4995322056211791
      }
    }
  }
}
```

#### 1.2.3 docker-compose.yml - 编排配置文件

创建并编辑 `docker-compose.yml` 文件：


``` docker-compose.yml
services:
  deqing_serve:
    # 使用您提供的镜像名称
    image: deqing_serve:1-12-5    # 请替换为实际的标签
    container_name: deqing_service  # 容器名称
    restart: unless-stopped  # 容器重启策略

    # 只读文件系统
    read_only: true  # 将文件系统设置为只读
    # Linux Capabilities 控制
    cap_drop:
      - ALL # 删除所有默认权限
    cap_add:
      - NET_BIND_SERVICE  #
    # 非root用户运行
    user: appuser  # 请确保在Dockerfile中创建了该用户
    # 临时文件系统
    tmpfs:
      - /tmp:size=64M,mode=1777
      - /run:size=64M,mode=1777
      - /app/tmp:size=32M,mode=1777
    # 安全选项
    security_opt:
      - no-new-privileges:true
      - apparmor:docker-default

    # 进程限制
    # pids_limit: 1024

    # 端口映射
    ports:
      - "9990:9997"

    # 环境变量
    environment:
      - TZ=Asia/Shanghai
      - LOG_LEVEL=WARN
      - SECURITY_MODE=ENABLED
      - LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu

    # 核心配置：外部挂载
    volumes:
      # === 配置文件挂载 ===
      # 直接挂载配置文件到app根目录，确保程序能找到
      # [注意] 宿主机必须先存在这些文件，否则会被当成目录创建
      - ./deployment/config/config.json:/app/config.json:ro

      # 单独挂载 region.json 到app根目录
      - ./deployment/config/region.json:/app/region.json:ro

      # === 数据目录挂载 ===
      # [注意] 宿主机的这些目录必须 chmod 777 或者 chown 给对应 UID
      # 日志目录
      - ./deployment/logs:/app/logs:rw

      # 静态数据文件（osgb 文件等）
      - ./deployment/data:/app/data:rw

      # 用户上传文件目录
      - ./deployment/uploads:/app/uploads:rw

      # === 可选：SSL 证书挂载（如果需要 HTTPS） ===
      # - ./deployment/ssl:/app/ssl:ro

    # 网络配置
    networks:
      - deqing_network

    # 健康检查
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:9997/"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

    # # 资源限制
    # deploy:
    #   resources:
    #     limits:
    #       cpus: '2.0'
    #       memory: 2G
    #     reservations:
    #       cpus: '0.5'
    #       memory: 512

# 网络配置
networks:
  deqing_network:
    driver: bridge
# 数据卷配置
volumes:
  postgres_data:
    driver: local
    
```

### 1.4 服务器部署操作

在服务器上执行以下部署步骤：

#### 1.4.1 进入部署目录

```
cd /path/to/deqing_serve
```

#### 1.4.2 停止并清理旧服务


```
\# 停止运行中的容器

docker stop deqing_service 2>/dev/null || true

\# 删除旧容器

docker rm deqing_service 2>/dev/null || true

\# 删除旧镜像

docker rmi deqing_serve:tag 2>/dev/null || true
```

#### 1.4.3 导入新镜像

``` linux
\# 导入Docker镜像

docker load -i deqing-serve-tag.tar.gz

\# 验证镜像导入成功

docker images | grep deqing_serve:tag
```
``` windows-powershell
\# 导入Docker镜像

docker load -i deqing-serve-tag.tar.gz

\# 验证镜像导入成功

docker images | sls "deqing_serve:tag"
```

#### 1.4.4 启动服务


```
\# 使用Docker Compose启动服务

docker-compose up -d

\# 查看服务状态

docker-compose ps

```

#### 1.4.5 验证服务部署



```
\# 查看服务日志

docker logs -f deqing_service

\# 测试服务访问（返回200 OK表示正常）

curl -I http://host:port/

\# 预期输出：

HTTP/1.1 200 OK
content-length: 29
content-type: text/html; charset=utf-8
server: Drogon
accept-range: bytes
expires: Thu, 01 Jan 1970 00:00:00 GMT
last-modified: Mon, 12 Jan 2026 13:58:07 GMT
date: Mon, 12 Jan 2026 14:18:52 GMT

```



***

## 📊 服务监控与维护

### 2.1 日常监控命令



```
\# 查看服务状态

docker-compose ps

\# 查看实时日志

docker-compose logs -f

\# 查看资源使用情况

docker stats deqing_service

\# 查看容器详细信息

docker inspect deqing_service
```

### 2.2 服务维护操作

#### 2.2.1 重启服务


```
\# 重启服务

docker-compose restart

```

#### 2.2.2 日志管理


```
\# 查看日志大小

du -sh logs/

\# 清理日志（保留最近7天）

find logs/ -name "\*.log" -type f -mtime +7 -delete
```


### 2.3 故障排除


**问题 1: 权限被拒绝错误**


```
\# 症状：容器无法写入数据目录

\# 解决方案：检查并修复目录权限

chmod -R 777 deployment/

chown -R 1000:1000 deployment/

```

**问题 2: 配置文件找不到**



```
\# 症状：容器启动失败，提示配置文件不存在

\# 解决方案：检查挂载路径和文件权限

ls -la config/

ls -la config/\*.json
```

**问题 3: 服务无法访问**



```
\# 症状：curl访问返回连接拒绝

\# 解决方案：检查端口映射和防火墙

docker-compose ps

netstat -tulpn | grep 9996

ufw status
```



***


## 📋 附录

###  性能优化建议

1. **资源调整**: 根据实际负载调整 CPU 和内存限制
2. **日志级别**: 生产环境使用 WARN 或 ERROR 级别日志