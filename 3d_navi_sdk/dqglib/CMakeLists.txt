cmake_minimum_required(VERSION 3.10)

project(dqglib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OSG 库
find_package(OpenSceneGraph REQUIRED COMPONENTS osgDB osg osgGA osgViewer osgUtil osgText)

# 查找 TIFF 库
find_package(TIFF REQUIRED)

# 查找 ZLIB 库
find_package(ZLIB REQUIRED)

# 查找 jsoncpp 库
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(JSONCPP jsoncpp)
endif()

# 如果pkg-config找不到，尝试find_package
if(NOT JSONCPP_FOUND)
    find_package(jsoncpp QUIET)
    if(jsoncpp_FOUND)
        set(JSONCPP_FOUND TRUE)
        set(JSONCPP_LIBRARIES jsoncpp)
        set(JSONCPP_INCLUDE_DIRS "")
    endif()
endif()

# 如果都找不到，尝试手动查找
if(NOT JSONCPP_FOUND)
    find_path(JSONCPP_INCLUDE_DIR json/json.h
        PATHS
        /usr/include/jsoncpp
        /usr/local/include/jsoncpp
        /usr/include
        /usr/local/include
        /usr/include/x86_64-linux-gnu
    )
    find_library(JSONCPP_LIBRARY jsoncpp
        PATHS
        /usr/lib
        /usr/local/lib
        /usr/lib/x86_64-linux-gnu
        /usr/lib/x86_64-linux-gnu/jsoncpp
    )
    if(JSONCPP_INCLUDE_DIR AND JSONCPP_LIBRARY)
        set(JSONCPP_FOUND TRUE)
        set(JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR})
        set(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY})
        message(STATUS "Found jsoncpp manually: ${JSONCPP_LIBRARY}")
    endif()
endif()

if(NOT JSONCPP_FOUND)
    message(FATAL_ERROR "Could not find jsoncpp library. Install libjsoncpp-dev or set JSONCPP_DIR.")
else()
    message(STATUS "Found jsoncpp: ${JSONCPP_LIBRARIES}")
endif()

# 查找 PROJ 库：优先使用 CMake 配置，如果找不到则使用 pkg-config 回退
# 确保SQLite3_lib目标已存在，避免PROJ的依赖查找时创建重复目标
if(NOT TARGET SQLite3_lib)
    find_package(SQLite3 QUIET)
endif()
find_package(PROJ CONFIG QUIET)
if(PROJ_FOUND)
    if(TARGET PROJ::proj)
        set(PROJ_LIBRARIES PROJ::proj)
    else()
        set(PROJ_LIBRARIES "")
        set(PROJ_INCLUDE_DIRS ${PROJ_INCLUDE_DIRS})
    endif()
else()
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PROJ_PKG proj)
        if(PROJ_PKG_FOUND)
            set(PROJ_LIBRARIES ${PROJ_PKG_LIBRARIES})
            set(PROJ_INCLUDE_DIRS ${PROJ_PKG_INCLUDE_DIRS})
        endif()
    endif()
endif()

if(NOT DEFINED PROJ_LIBRARIES OR PROJ_LIBRARIES STREQUAL "")
    message(FATAL_ERROR "Could not find PROJ (via CMake config or pkg-config). Install libproj-dev or set PROJ_DIR.")
endif()

# 查找 PROJ 库的数据目录路径，用于设置 PROJ_LIB 环境变量
if(PROJ_FOUND)
    # 尝试从 PROJ::proj 目标获取数据目录
    if(TARGET PROJ::proj)
        get_target_property(PROJ_LIB_DIR PROJ::proj INTERFACE_INCLUDE_DIRECTORIES)
        if(PROJ_LIB_DIR)
            # 通常数据目录在包含目录的 share/proj 子目录中
            foreach(dir ${PROJ_LIB_DIR})
                if(EXISTS "${dir}/share/proj")
                    set(PROJ_DATA_PATH "${dir}/share/proj")
                    break()
                endif()
                
                # 检查父目录的 share/proj
                get_filename_component(parent_dir "${dir}" DIRECTORY)
                if(EXISTS "${parent_dir}/share/proj")
                    set(PROJ_DATA_PATH "${parent_dir}/share/proj")
                    break()
                endif()
            endforeach()
        endif()
    endif()
endif()

# 如果没有找到，尝试一些常见的安装路径
if(NOT DEFINED PROJ_DATA_PATH)
    # Windows 常见路径
    if(WIN32)
        set(COMMON_PROJ_PATHS
            "D:/vcpkg/installed/x64-windows/share/proj"
            "C:/vcpkg/installed/x64-windows/share/proj"
            "C:/Program Files/PROJ/share/proj"
            "C:/Program Files (x86)/PROJ/share/proj"
            "C:/OSGeo4W64/share/proj"
        )
    else()
        # Linux/Unix 常见路径
        set(COMMON_PROJ_PATHS
            "/usr/share/proj"
            "/usr/local/share/proj"
            "/opt/proj/share/proj"
            "/usr/share/proj9"
            "/usr/share/proj8"
            "/usr/share/proj7"
        )
    endif()
    
    foreach(path ${COMMON_PROJ_PATHS})
        if(EXISTS "${path}")
            set(PROJ_DATA_PATH "${path}")
            break()
        endif()
    endforeach()
endif()

# 如果找到了 PROJ 数据路径，在编译时定义一个宏
if(DEFINED PROJ_DATA_PATH)
    message(STATUS "Found PROJ data path: ${PROJ_DATA_PATH}")
    # 将路径转换为适用于编译器的字符串
    file(TO_CMAKE_PATH "${PROJ_DATA_PATH}" PROJ_DATA_PATH_CMAKE)
else()
    message(WARNING "Could not find PROJ data directory. PROJ coordinate transformations may fail.")
    message(STATUS "You may need to set PROJ_LIB environment variable at runtime.")
endif()

# 添加头文件路径
include_directories(include)

# 自动搜索源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")
# 添加PlanCheckLib目录的源文件
file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")

# 排除主函数文件
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/OSGtoCode.cpp")

# 使用原始源文件列表
set(ALL_SOURCES ${SOURCES})

# 生成库文件
add_library(${PROJECT_NAME} STATIC ${ALL_SOURCES} ${HEADERS})

# 如果找到了 PROJ 数据路径，在编译时定义一个宏
if(DEFINED PROJ_DATA_PATH)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PROJ_DATA_PATH="${PROJ_DATA_PATH_CMAKE}")
endif()

# 链接所有库
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${OPENSCENEGRAPH_LIBRARIES}
        TIFF::TIFF
        ZLIB::ZLIB
        ${PROJ_LIBRARIES}
        ${JSONCPP_LIBRARIES}
)

# 设置包含目录
target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
        ${JSONCPP_INCLUDE_DIRS}
)

# 设置公共头文件目录，使其他项目可以包含这些头文件
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 导出库信息
set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION 1.0
        SOVERSION 1
        PUBLIC_HEADER "${HEADERS}"
)