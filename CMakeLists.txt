# 设置最低CMake版本要求和项目名称
cmake_minimum_required(VERSION 3.10)
project(deqing_serve CXX)

# ##############################################################################
# 编译器和C++标准配置
# ##############################################################################

set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ standard to use" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

include(CheckIncludeFileCXX)
check_include_file_cxx(coroutine HAS_COROUTINE)
if(NOT HAS_COROUTINE)
    message(WARNING "Compiler does not support C++20 coroutines, but we are using C++20 anyway")
endif()

message(STATUS "Using C++20 standard (required by Drogon framework)")

# ##############################################################################
# 子目录优先构建（必须在 find_package 之前，避免 SQLite3_lib 目标重复创建）
# dqglib 依赖 PROJ，PROJ 依赖 SQLite3，需要让它先占用 SQLite3_lib 目标名
# ##############################################################################

add_subdirectory(dqglib)

# ##############################################################################
# 外部库依赖配置（dqglib 之后，保证 SQLite3_lib 只创建一次）
# ##############################################################################

find_package(SQLite3 REQUIRED)
find_package(Drogon CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(TBB REQUIRED)
find_package(PostgreSQL REQUIRED)
find_package(GEOS REQUIRED)

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
        controller/api_multiSource_tifGrid.cc
        controller/api_multiSource_tifGrid.h
        text/prefixToCode.cc
        text/prefixToCode.h
)
# ##############################################################################
# 链接库
# ##############################################################################

target_link_libraries(${PROJECT_NAME} PRIVATE
        Drogon::Drogon
        dqglib
        TBB::tbb
        ${PostgreSQL_LIBRARIES}
        geos
        geos_c

)

# ##############################################################################
# 链接库
# ##############################################################################

target_link_libraries(${PROJECT_NAME} PRIVATE
        JsonCpp::JsonCpp
        Drogon::Drogon
        dqglib
        TBB::tbb
        ${PostgreSQL_LIBRARIES}
        geos
        geos_c)

# ##############################################################################
# 验证C++标准
# ##############################################################################

if (CMAKE_CXX_STANDARD LESS 20)
    message(FATAL_ERROR "C++20 or higher is required for Drogon framework")
else ()
    message(STATUS "Using C++20")
endif ()

# ##############################################################################
# 源码目录扫描
# ##############################################################################

file(GLOB MODEL_SRC "models/*.cc")
list(REMOVE_ITEM MODEL_SRC "models/GridData.cc")

# ##############################################################################
# 包含目录和源文件配置
# ##############################################################################

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/models
        ${CMAKE_CURRENT_SOURCE_DIR}/GridEvaluatorLib
        dqglib/include
        ${PostgreSQL_INCLUDE_DIRS}
        ${GEOS_INCLUDE_DIRS})
target_sources(${PROJECT_NAME}
        PRIVATE
        ${SRC_DIR}
        ${MODEL_SRC})

# ##############################################################################
# 测试子目录
# ##############################################################################

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
else()
    message(STATUS "No test/CMakeLists.txt found; skipping add_subdirectory(test)")
endif()

# ##############################################################################
# 安装规则
# ##############################################################################

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.json")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config.json" DESTINATION share/${PROJECT_NAME})
endif()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml" DESTINATION share/${PROJECT_NAME})
endif()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/region.json")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/region.json" DESTINATION share/${PROJECT_NAME})
endif()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data/" DESTINATION share/${PROJECT_NAME}/data)
endif()