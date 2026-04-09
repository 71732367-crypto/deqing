cmake_minimum_required(VERSION 3.10)
project(deqing_serve CXX)

link_directories("D:/vcpkg/installed/x64-windows/debug/lib")
link_directories("D:/vcpkg/installed/x64-windows/lib")

if(MSVC)
    add_compile_options("/utf-8")
    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-DNOMINMAX)
    add_definitions(-DJSON_DLL) #  解决 LNK2001 的 Json::null 报错
endif()

# ##############################################################################
# 编译器和C++标准配置
# ##############################################################################

# 强制使用 C++20
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ standard to use" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 检测编译器支持的协程标志
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

message(STATUS "Using C++20 standard (required by Drogon framework)")

# ##############################################################################
# 可执行文件定义和源文件列表
# ##############################################################################

add_executable(${PROJECT_NAME} main.cc
        controller/api_multiSource_basicGrid.cc
        controller/api_multiSource_Data3dGrid.cc
        controller/api_multiSource_geometricGrid.cc
        controller/api_multiSource_triangleGrid.cc
        controller/api_multiSource_airSpace.cc
        controller/api_airRoute_pointConflictCheck.cc
        controller/api_airRoute_Astar.cc
        controller/api_airRoute_lineConflictCheck.cc
        GridEvaluatorLib/GridEvaluator.cc
        GridEvaluatorLib/conditionCheck.cc
        GridEvaluatorLib/GridEvaluator.h
        controller/LineToGrids.cpp
        controller/LineToGrids.h
)

# ##############################################################################
# 外部库依赖配置
# ##############################################################################

# 1. 先查找 SQLite3 基础库
find_package(SQLite3 REQUIRED)

# 2. 安全地创建 SQLite3_lib 目标（防止 Drogon 和 PROJ 冲突）
# 使用 if(NOT TARGET) 保护，而不是重写 add_library 命令
if(NOT TARGET SQLite3_lib)
    add_library(SQLite3_lib INTERFACE IMPORTED)
    set_target_properties(SQLite3_lib PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${SQLite3_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "SQLite::SQLite3")
    message(STATUS "Created SQLite3_lib interface target")
endif()

# 3. 查找核心框架和依赖
find_package(Drogon CONFIG REQUIRED)
find_package(TBB REQUIRED)
find_package(PostgreSQL REQUIRED)
find_package(GEOS REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(unofficial-sqlite3 CONFIG REQUIRED)


# 链接项目所需的库
# 建议使用 JsonCpp::jsoncpp (vcpkg 标准用法) 替代裸写的 jsoncpp

target_link_libraries(${PROJECT_NAME} PRIVATE
        dqglib
        Drogon::Drogon
        TBB::tbb
        PostgreSQL::PostgreSQL
        GEOS::geos
        GEOS::geos_c
        unofficial::sqlite3::sqlite3
        OpenSSL::SSL
        OpenSSL::Crypto
        JsonCpp::JsonCpp
)

# ##############################################################################
# 包含目录和源文件配置
# ##############################################################################

target_include_directories(${PROJECT_NAME}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/models
        ${CMAKE_CURRENT_SOURCE_DIR}/GridEvaluatorLib
        dqglib/include
        ${PostgreSQL_INCLUDE_DIRS}
        ${GEOS_INCLUDE_DIRS}
)

# 处理 models 目录下的源文件
file(GLOB MODEL_SRC "models/*.cc")
if(MODEL_SRC)
    list(REMOVE_ITEM MODEL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/models/GridData.cc")
    target_sources(${PROJECT_NAME} PRIVATE ${MODEL_SRC})
endif()

# ##############################################################################
# 子目录构建配置
# ##############################################################################

add_subdirectory(dqglib)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
endif()

# ##############################################################################
# 安装规则配置
# ##############################################################################

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

# 安装配置文件
foreach(CONF_FILE config.json config.yaml region.json)
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${CONF_FILE}")
        install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/${CONF_FILE}" DESTINATION share/${PROJECT_NAME})
    endif()
endforeach()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data/" DESTINATION share/${PROJECT_NAME}/data)
endif()

