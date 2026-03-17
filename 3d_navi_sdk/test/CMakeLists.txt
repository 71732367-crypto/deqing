cmake_minimum_required(VERSION 3.16)
project(test_redis)

# 设置C++标准（与主工程保持一致，使用C++20）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 解决jsoncpp头文件路径问题（添加jsoncpp的头文件搜索路径）
include_directories(/usr/include/jsoncpp)

# 查找hiredis的头文件和库文件（替代find_package(hiredis)，适配Ubuntu的apt安装）
# 1. 添加hiredis的头文件路径（Ubuntu下hiredis头文件在/usr/include/hiredis或/usr/include）
include_directories(/usr/include/hiredis /usr/include)
# 2. 查找hiredis库文件
find_library(HIREDIS_LIB hiredis PATHS /usr/lib/x86_64-linux-gnu /usr/lib)

# 查找Drogon库（CMake会自动关联所有依赖：trantor、jsoncpp、brotli、zlib、yaml-cpp等）
find_package(Drogon REQUIRED)

# 添加可执行文件
add_executable(test_redis test_redis.cpp)

# 链接依赖库（Drogon::Drogon + 手动找到的hiredis库）
target_link_libraries(test_redis PRIVATE
        Drogon::Drogon
        ${HIREDIS_LIB}
)
