# 基于 Ubuntu 24.04 LTS，提供稳定性和长期支持
# 安全等级：企业级
# 优化措施：构建阶段安全加固 + 精细化权限控制 + 启动脚本安全增强
# 修复说明：修正Ubuntu 24.04库包名称问题

# ================================
# 构建阶段
# ================================
FROM ubuntu:24.04 AS builder

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_BUILD_TYPE=Release
ENV MAKEFLAGS=-j$(nproc)

# 更新包管理器并安装构建依赖
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    # 基础构建工具\
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    curl \
    unzip \
    # C++ 编译器支持\
    g++ \
    gcc \
    # Drogon 框架依赖\
    libjsoncpp-dev \
    libhiredis-dev \
    libssl-dev \
    zlib1g-dev \
    # PostgreSQL 客户端库\
    libpq-dev \
    postgresql-client \
    # TBB库 (Intel Threading Building Blocks)\
    libtbb-dev \
    # OpenSceneGraph 依赖\
    libopenscenegraph-dev \
    libopenthreads-dev \
    # 图像处理库\
    libtiff-dev \
    libjpeg-dev \
    libpng-dev \
    # 地理投影库基础依赖 (手动构建PROJ)\
    libsqlite3-dev \
    sqlite3 \
    libcurl4-openssl-dev \
    libgeos-dev \
    libgeos++-dev \
    # UUID库\
    uuid-dev \
    # 其他工具\
    ca-certificates \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 复制源码到工作目录
WORKDIR /app
COPY . .

# 创建必要的目录和配置模板
RUN mkdir -p /app/configs /app/data && \
    echo '{"region":{"name":"默认区域","bounds":{"southwest":{"longitude":0,"latitude":0},"northwest":{"longitude":0,"latitude":0},"northeast":{"longitude":0,"latitude":0},"southeast":{"longitude":0,"latitude":0}}}' > /app/configs/region.json.template

# 手动构建 PROJ 库 (解决CMake配置文件问题)
RUN wget https://download.osgeo.org/proj/proj-9.2.1.tar.gz && \
    tar -xzf proj-9.2.1.tar.gz && \
    cd proj-9.2.1 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTING=OFF -DENABLE_CURL=OFF -DBUILD_PROJSYNC=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /app && \
    rm -rf proj-9.2.1 proj-9.2.1.tar.gz

# 在容器中下载并构建 Drogon 框架
RUN git config --global http.version HTTP/1.1 && \
    git config --global http.postBuffer 524288000 && \
    git clone --depth 1 https://github.com/an-tao/drogon.git /app/drogon && \
    cd /app/drogon && \
    git submodule update --init --recursive --depth 1 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# 构建项目
RUN mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local && \
    make -j$(nproc) && \
    make install

# ================================
# 构建阶段安全加固 - 关键步骤
# ================================
# 1. 保存需要保留的配置文件和目录
RUN cd /app && \
    # 保存配置文件和目录到临时位置\
    mkdir -p /tmp/keep && \
    cp -r config.json /tmp/keep/ 2>/dev/null || true && \
    cp -r configs /tmp/keep/ 2>/dev/null || true && \
    # 彻底清理源代码\
    rm -rf * && \
    rm -rf /app/drogon && \
    rm -rf /app/build && \
    # 恢复需要保留的配置文件和目录\
    mv /tmp/keep/* /app/ || true && \
    rm -rf /tmp/keep && \
    # 清理其他临时文件\
    rm -rf /tmp/* && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 2. 清理构建工具和依赖
RUN apt-get remove --purge -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    curl \
    unzip \
    g++ \
    gcc \
    && apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 3. 验证清理结果
RUN ls -la /app && \
    echo "构建阶段安全加固完成 - 源代码已清理"

# ================================
# 运行阶段
# ================================
# ================================
# 运行阶段
# ================================
FROM ubuntu:24.04 AS runtime

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# 安装基础运行时依赖
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    libjsoncpp25 \
    libhiredis1.1.0 \
    libssl3 \
    libpq5 \
    libtbbmalloc2 \
    libopenscenegraph161 \
    libopenthreads21 \
    libtiff6 \
    libjpeg-turbo8 \
    libpng16-16 \
    libsqlite3-0 \
    libgeos-c1t64 \
    libgeos3.12.1t64 \
    uuid-runtime \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 创建非 root 用户
RUN groupadd -r appuser && \
    useradd -r -g appuser -d /app -s /bin/bash appuser

WORKDIR /app

# --- 创建健康检查所需的默认页面 ---
RUN echo "Deqing Serve Health Check OK" > index.html && \
    chown appuser:appuser index.html && \
    chmod 644 index.html
# ----------------------------------------

# 复制文件
COPY --from=builder /usr/local/bin/deqing_serve /usr/local/bin/
COPY --from=builder /usr/local/lib/ /usr/local/lib/
COPY --from=builder /usr/local/include/ /usr/local/include/
COPY --from=builder /usr/local/share/ /usr/local/share/

# 预置配置文件
COPY --from=builder /app/config.json ./config.json.template
COPY --from=builder /app/config.json ./config.json
COPY --from=builder /app/configs ./configs
RUN cp ./configs/region.json.template ./region.json

# 确保data目录存在
RUN mkdir -p ./data

# 设置库路径环境变量
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu

# ================================
# 精细化权限控制
# ================================
RUN mkdir -p /app/uploads /app/logs /app/tmp

RUN \
    # 配置文件 - 只读
    chown root:root /app/configs && \
    chmod 555 /app/configs && \
    chown root:root /app/config.json /app/region.json /app/config.json.template && \
    chmod 444 /app/config.json /app/region.json /app/config.json.template && \
    # 数据目录 - appuser可读写
    chown -R appuser:appuser /app/data /app/uploads /app/logs /app/tmp && \
    chmod 700 /app/data /app/uploads /app/logs /app/tmp && \
    # 工作目录 - root所有，但允许其他人进入和读取
    chown root:root /app && \
    chmod 755 /app

RUN ls -la /app && \
    echo "权限设置完成"

# ================================
# 启动脚本
# ================================
RUN echo '#!/bin/bash' > /app/start.sh && \
    echo '' >> /app/start.sh && \
    echo '# 安全启动脚本' >> /app/start.sh && \
    # 1. Root检查
    echo 'if [ "$(id -u)" -eq 0 ]; then' >> /app/start.sh && \
    echo '    echo "⚠️  Switching to appuser..."' >> /app/start.sh && \
    echo '    exec su - appuser -c "/app/start.sh"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo '' >> /app/start.sh && \
    # 2. 配置文件 config.json 检查
    echo 'if [ ! -f /app/config.json ]; then' >> /app/start.sh && \
    echo '    echo "❌  错误：config.json 未找到"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'else' >> /app/start.sh && \
    echo '    echo "ℹ️  信息: config.json 存在"' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo '' >> /app/start.sh && \
    # 3. 配置文件 region.json 检查
    echo 'if [ ! -f /app/region.json ]; then' >> /app/start.sh && \
    echo '    echo "❌  错误：region.json 未找到"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'else' >> /app/start.sh && \
    echo '    echo "ℹ️  信息: 配置文件 region.json 检查存在"' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo '' >> /app/start.sh && \
    # 4. 目录权限检查
    echo 'if [ ! -w /app/data ]; then' >> /app/start.sh && \
    echo '    echo "❌  错误：数据目录不可写"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo '' >> /app/start.sh && \
    # 5. LD_LIBRARY_PATH 检查
    echo 'if [ -z "$LD_LIBRARY_PATH" ]; then' >> /app/start.sh && \
    echo '    export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo '' >> /app/start.sh && \
    # 6. 启动应用
    echo 'echo "✅  Deqing Serve 正在启动..."' >> /app/start.sh && \
    echo 'cd /app' >> /app/start.sh && \
    echo 'exec /usr/local/bin/deqing_serve' >> /app/start.sh && \
    chown appuser:appuser /app/start.sh && \
    chmod 500 /app/start.sh

# 更新库缓存
RUN ldconfig

# 切换用户
USER appuser

# 暴露端口
EXPOSE 9997

# 启动命令
CMD ["/app/start.sh"]
# 添加安全标签
LABEL maintainer="CUMTB_307A" \
      description="Deqing Serve - 基于 Ubuntu 24.04 LTS"