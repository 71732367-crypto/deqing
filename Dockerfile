# 基于 Ubuntu 24.04 LTS，提供稳定性和长期支持
# 安全等级：企业级
# 优化措施：构建阶段安全加固 + 精细化权限控制 + 启动脚本安全增强
# 修复说明：切换清华大学(TUNA)镜像源，并增加 apt-get 网络容错机制

# ================================
# 构建阶段
# ================================
FROM ubuntu:24.04 AS builder

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_BUILD_TYPE=Release
ENV MAKEFLAGS=-j$(nproc)

# 🚀 替换为清华大学 (TUNA) 镜像源
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources

# 更新包管理器并安装构建依赖 (增加 --fix-missing 容错)
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends --fix-missing \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    unzip \
    curl \
    ca-certificates \
    g++ \
    gcc \
    libjsoncpp-dev \
    libhiredis-dev \
    libssl-dev \
    zlib1g-dev \
    libpq-dev \
    postgresql-client \
    libtbb-dev \
    libopenscenegraph-dev \
    libopenthreads-dev \
    libtiff-dev \
    libjpeg-dev \
    libpng-dev \
    libsqlite3-dev \
    sqlite3 \
    libcurl4-openssl-dev \
    libgeos-dev \
    libgeos++-dev \
    uuid-dev \
    libgdal-dev \
    gdal-bin \
    libproj-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 复制源码到工作目录
WORKDIR /app
COPY . .

# 创建必要的目录和配置模板
RUN mkdir -p /app/configs /app/data && \
    echo '{"region":{"name":"默认区域","bounds":{"southwest":{"longitude":0,"latitude":0},"northwest":{"longitude":0,"latitude":0},"northeast":{"longitude":0,"latitude":0},"southeast":{"longitude":0,"latitude":0}}}' > /app/configs/region.json.template

# 【注意：这里已经删除了手动下载和编译 PROJ 9.2.1 的冗长步骤】
# 系统现在会直接使用上面通过 apt-get 安装的、与 GDAL 版本完美匹配的 libproj-dev

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
RUN cd /app && \
    mkdir -p /tmp/keep && \
    cp -r config.json /tmp/keep/ 2>/dev/null || true && \
    cp -r configs /tmp/keep/ 2>/dev/null || true && \
    rm -rf * && \
    rm -rf /app/drogon && \
    rm -rf /app/build && \
    mv /tmp/keep/* /app/ || true && \
    rm -rf /tmp/keep && \
    rm -rf /tmp/* && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get remove --purge -y \
    build-essential cmake git pkg-config wget curl unzip g++ gcc \
    && apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN ls -la /app && \
    echo "构建阶段安全加固完成 - 源代码已清理"

# ================================
# 运行阶段
# ================================
FROM ubuntu:24.04 AS runtime

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# 🚀 替换为清华大学 (TUNA) 镜像源
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources && \
    sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources

# 安装基础运行时依赖 (增加 --fix-missing 容错)
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends --fix-missing \
    ca-certificates \
    curl \
    libjsoncpp-dev \
    libhiredis-dev \
    libssl-dev \
    libpq5 \
    libtbb-dev \
    libopenscenegraph-dev \
    libtiff-dev \
    libjpeg-turbo8 \
    libpng-dev \
    libsqlite3-0 \
    libgeos-dev \
    uuid-runtime \
    libgdal-dev \
    gdal-bin \
    libproj-dev \
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
    chown root:root /app/configs && \
    chmod 555 /app/configs && \
    chown root:root /app/config.json /app/region.json /app/config.json.template && \
    chmod 444 /app/config.json /app/region.json /app/config.json.template && \
    chown -R appuser:appuser /app/data /app/uploads /app/logs /app/tmp && \
    chmod 700 /app/data /app/uploads /app/logs /app/tmp && \
    chown root:root /app && \
    chmod 755 /app

# ================================
# 启动脚本
# ================================
RUN echo '#!/bin/bash' > /app/start.sh && \
    echo '' >> /app/start.sh && \
    echo '# 安全启动脚本' >> /app/start.sh && \
    echo 'if [ "$(id -u)" -eq 0 ]; then' >> /app/start.sh && \
    echo '    echo "⚠️  Switching to appuser..."' >> /app/start.sh && \
    echo '    exec su - appuser -c "/app/start.sh"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo 'if [ ! -f /app/config.json ]; then' >> /app/start.sh && \
    echo '    echo "❌  错误：config.json 未找到"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo 'if [ ! -f /app/region.json ]; then' >> /app/start.sh && \
    echo '    echo "❌  错误：region.json 未找到"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo 'if [ ! -w /app/data ]; then' >> /app/start.sh && \
    echo '    echo "❌  错误：数据目录不可写"' >> /app/start.sh && \
    echo '    exit 1' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
    echo 'if [ -z "$LD_LIBRARY_PATH" ]; then' >> /app/start.sh && \
    echo '    export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu' >> /app/start.sh && \
    echo 'fi' >> /app/start.sh && \
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