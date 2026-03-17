# 设置最低CMake版本要求和项目名称
cmake_minimum_required(VERSION 3.10)
project(deqing_serve CXX)

# ##############################################################################
# 编译器和C++标准配置
# ##############################################################################

# 强制使用 C++20，Drogon 框架需要 C++20 支持，使用了c++协程等特性
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ standard to use" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 检测编译器支持的协程标志，并仅对 C++ 应用该标志
include(CheckCXXCompilerFlag)
set(COROUTINES_FLAG "")
check_cxx_compiler_flag("-fcoroutines" HAS_CORO_FLAG)
if(HAS_CORO_FLAG)
    set(COROUTINES_FLAG "-fcoroutines")
else()
    check_cxx_compiler_flag("-fcoroutines-ts" HAS_CORO_TS_FLAG)
    if(HAS_CORO_TS_FLAG)
        set(COROUTINES_FLAG "-fcoroutines-ts")
    endif()
endif()

if(COROUTINES_FLAG)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${COROUTINES_FLAG}>")
endif()

# C++20 已经在上面强制设置，跳过自动检测
include(CheckIncludeFileCXX)

check_include_file_cxx(coroutine HAS_COROUTINE)
if(NOT HAS_COROUTINE)
    message(WARNING "Compiler does not support C++20 coroutines, but we are using C++20 anyway")
endif()

message(STATUS "Using C++20 standard (required by Drogon framework)")

# ##############################################################################
# 可执行文件定义和源文件列表
# ##############################################################################

# 定义项目可执行文件及其源文件
add_executable(${PROJECT_NAME} main.cc
        controller/api_multiSource_basicGrid.cc  # 基础网格接口
        controller/api_multiSource_Data3dGrid.cc  # 三维数据网格接口
        controller/api_multiSource_geometricGrid.cc  # 几何网格接口（包含pointGrid和lineGrid功能）
        controller/api_multiSource_triangleGrid.cc  # 三角形网格接口
        # A*航路规划接口
        # 新版A*航路规划接口
        controller/api_multiSource_airSpace.cc
        controller/api_airRoute_pointConflictCheck.cc
        controller/api_airRoute_Astar.cc
        controller/api_airRoute_lineConflictCheck.cc
        GridEvaluatorLib/GridEvaluator.cc
        GridEvaluatorLib/conditionCheck.cc
        GridEvaluatorLib/GridEvaluator.h
        #空域相关接口
)

# ##############################################################################
# 外部库依赖配置
# ##############################################################################

# ##############################################################################
# Using drogon source code locally to ensure Redis support
# add_subdirectory(drogon)
# target_link_libraries(${PROJECT_NAME} PRIVATE drogon)
#
# 临时使用系统安装的Drogon包，但需要检查Redis支持
# 先显式查找SQLite3，避免Drogon和PROJ库创建同名目标冲突
find_package(SQLite3 REQUIRED)
# 手动创建SQLite3_lib目标，防止Drogon和PROJ重复创建
if(NOT TARGET SQLite3_lib)
    add_library(SQLite3_lib INTERFACE IMPORTED)
    set_target_properties(SQLite3_lib PROPERTIES 
        INTERFACE_INCLUDE_DIRECTORIES "${SQLite3_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "SQLite::SQLite3")
endif()
# 重定义add_library命令，防止重复创建SQLite3_lib目标
macro(add_library name)
    if(${name} STREQUAL "SQLite3_lib" AND TARGET SQLite3_lib)
        message(STATUS "SQLite3_lib target already exists, skipping recreation")
    else()
        _add_library(${name} ${ARGN})
    endif()
endmacro()
find_package(Drogon CONFIG REQUIRED)

# 查找 TBB 库（C++20 并行算法需要）
find_package(TBB REQUIRED)

# 查找 PostgreSQL 库
find_package(PostgreSQL REQUIRED)

# 查找 GEOS 库（用于多边形几何运算）
find_package(GEOS REQUIRED)

# 移除hiredis依赖 - 使用redis-cli或HTTP API替代
# 如果需要Redis功能，可以使用redis-cli命令行工具或HTTP API

# 链接项目所需的库文件
target_link_libraries(${PROJECT_NAME} PRIVATE Drogon::Drogon dqglib TBB::tbb jsoncpp ${PostgreSQL_LIBRARIES} geos geos_c)

# ##############################################################################

# 验证C++标准版本是否满足要求
if (CMAKE_CXX_STANDARD LESS 20)
    message(FATAL_ERROR "C++20 or higher is required for Drogon framework")
else ()
    message(STATUS "Using C++20")
endif ()

# ##############################################################################
# 源码目录扫描（当前被注释掉的部分）
# ##############################################################################

# 注释掉不存在的目录，避免构建错误
# aux_source_directory(controllers CTL_SRC)
# aux_source_directory(filters FILTER_SRC)
# aux_source_directory(plugins PLUGIN_SRC)
# 注意：手动排除 GridData.cc，因为它与手动定义的 GridData.h 不匹配
file(GLOB MODEL_SRC
        "models/*.cc"
)
list(REMOVE_ITEM MODEL_SRC "models/GridData.cc")

# ##############################################################################
# 视图处理（当前被注释掉的部分）
# ##############################################################################

# 注释掉 views 创建，因为 views 目录不存在
# drogon_create_views(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/views
#         ${CMAKE_CURRENT_BINARY_DIR})
# use the following line to create views with namespaces.
# drogon_create_views(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/views
#                     ${CMAKE_CURRENT_BINARY_DIR} TRUE)
# use the following line to create views with namespace CHANGE_ME prefixed
# and path namespaces.
# drogon_create_views(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/views
#                     ${CMAKE_CURRENT_BINARY_DIR} TRUE CHANGE_ME)

# ##############################################################################
# 包含目录和源文件配置
# ##############################################################################

# 设置项目的包含目录
target_include_directories(${PROJECT_NAME}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/models
        ${CMAKE_CURRENT_SOURCE_DIR}/GridEvaluatorLib
        dqglib/include
        ${PostgreSQL_INCLUDE_DIRS}
        ${GEOS_INCLUDE_DIRS})

# 添加项目源文件
target_sources(${PROJECT_NAME}
        PRIVATE
        ${SRC_DIR}
        # ${CTL_SRC}  # controllers 目录不存在，暂时注释
        # ${FILTER_SRC}  # filters 目录不存在，暂时注释
        # ${PLUGIN_SRC}  # plugins 目录不存在，暂时注释
        ${MODEL_SRC})

# ##############################################################################
# 动态加载视图相关配置（可选）
# ##############################################################################

# uncomment the following line for dynamically loading views
# set_property(TARGET ${PROJECT_NAME} PROPERTY ENABLE_EXPORTS ON)

# ##############################################################################
# 子目录构建配置
# ##############################################################################

# 添加dqglib子目录进行构建
add_subdirectory(dqglib)

# 条件性地添加测试子目录
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
else()
    message(STATUS "No test/CMakeLists.txt found; skipping add_subdirectory(test)")
endif()

# ##############################################################################
# 安装规则配置
# ##############################################################################

# Install rules so `make install` works in the Docker build
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

# Install config files if present
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.json")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config.json" DESTINATION share/${PROJECT_NAME})
endif()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml" DESTINATION share/${PROJECT_NAME})
endif()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/region.json")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/region.json" DESTINATION share/${PROJECT_NAME})
endif()

# Install data directory if present
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data/" DESTINATION share/${PROJECT_NAME}/data)
endif()