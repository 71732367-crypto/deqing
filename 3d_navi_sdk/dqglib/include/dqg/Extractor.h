#pragma once
#include <iostream>
#include <filesystem>
#include <unordered_set>
#include <array>
#include <cmath>
#include <algorithm>
#include <limits>

#include <osg/Geometry>
#include <osg/LOD>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Group>
#include <osg/Geode>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/CoordinateSystemNode>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <proj.h>
#include"Data.h"
using namespace std;
namespace fs = std::filesystem;
struct Vertex {
    double lon, lat, alt;
    Vertex(double lon, double lat, double alt) : lon(lon), lat(lat), alt(alt) {}
};

void convertCoordinates(vector<Triangle>& triangles,

    ////以下为德清数据
    const string& sourceCRS = "EPSG:4528",
    const string& targetCRS = "EPSG:4326",
    //double offsetX = 0.0, double offsetY = 0.0);
    double offsetX= 40497564.61263241, double offsetY= 3377677.966467825,double offsetZ = 53.29796380422264);


////以下为香港数据
//const string& sourceCRS = "EPSG:2326",
//const string& targetCRS = "EPSG:4326",
//double offsetX = 800000.0, double offsetY = 800000.0,double offsetZ = 0.0);


std::vector<osg::Vec3d> convertOSG_Coordinates(const std::vector<Vertex>& vertices,
    double offsetX, double offsetY, double offsetZ=0.0 );

//从指定层级的 .osgb 文件中提取多个文件中三角形

inline std::string constructFileName(const std::string& baseName, int level) {
    int zeroCount = std::max(1, level - 13); // 层级越高，0 越多
    std::string zeros(zeroCount, '0');
    return baseName + "_L" + std::to_string(level) + "_" + zeros + ".osgb";


}

//最高效率:根块直提
void extractTriangles(osg::Node* node, vector<Triangle>& triangles);
//从指定层级的文件中提取单个文件中的三角形
void extractTrianglesFromLevel(const std::string& folderPath, int level, std::vector<Triangle>& triangles);
//从指定层级的 .osgb 文件中提取多个文件中三角形
void extractTrianglesFromLevelFiles(const std::string& folderPath, int level, std::vector<Triangle>& triangles);

void extractTriangles_bug(osg::Node* node, vector<Triangle>& triangles);

// 从metadata.xml文件中读取源坐标系信息
std::string readSourceCRSFromXML(const std::string& dataDir);

// 从metadata.xml文件中读取坐标偏移量信息
CoordinateOffset readCoordinateOffsetFromXML(const std::string& dataDir);

// 从数据目录自动读取坐标系和偏移量的版本
void convertCoordinatesFromXML(vector<Triangle>& triangles,
    const std::string& dataDir,
    const string& targetCRS = "EPSG:4326");

//三角面片立体模型填充网格计算结果结构体
struct PolyhedronGridResult {
    std::unordered_set<std::string> gridCodes;  // 网格编码集合
    double multiplierUsed;                        // 实际使用的采样倍率
    size_t actualSamples;                         // 实际采样数量
    bool hitMaxSamples;                          // 是否达到最大采样数限制
    std::string centerCode;                       // 中心点网格编码
    
    // 构造函数，提供合理的默认值
    PolyhedronGridResult() 
        : multiplierUsed(2.0)
        , actualSamples(0)
        , hitMaxSamples(false) {}
};

/**
 * @brief 三角面片立体模型填充网格计算
 *
 * 算法流程：
 * 1. 边界计算：计算多面体的最小包围盒
 * 2. 网格参数计算：确定网格步长和中心点编码
 * 3. 几何计算：定义射线投射、三角形相交等辅助函数
 * 4. 凸包构建：计算顶点水平投影的凸包
 * 5. 自适应采样：从精细到粗糙进行采样，直到满足内存限制
 * 6. 网格单元检测：判断采样点是否在多面体内或与多面体相交
 * 
 * @param faces 三角面片数组，每个面片包含3个顶点坐标（经纬度高度）
 * @param baseTile 基础网格配置，用于网格编码计算
 * @param level 网格层级（0-21），层级越高网格越精细
 * @param sampleMultiplier 采样倍率，控制采样密度，默认2.0
 * @param maxSamples 最大采样数，0表示无限制，用于内存控制
 * @return PolyhedronGridResult 包含所有相交网格编码的集合及采样统计信息
 *
 * @note 输入坐标应为经纬度坐标系（WGS84/EPSG:4326）
 */
PolyhedronGridResult computePolyhedronGridFill(
    const std::vector<std::array<PointLBHd, 3>>& faces,
    const BaseTile& baseTile,
    int level,
    double sampleMultiplier = 2.0,
    size_t maxSamples = 0);
