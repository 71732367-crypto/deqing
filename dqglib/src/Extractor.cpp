/*
Copyright (c) [2025] [AnYiZong]
All rights reserved.
No part of this code may be copied or distributed in any form without permission.
*/
#include <dqg/Extractor.h>
#include <dqg/DQG3DBasic.h>
#include <stdint.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iomanip>

#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/Array>
#include <osgDB/ReadFile>


void extractTriangles_bug(osg::Node* node, vector<Triangle>& triangles) {
    class TriangleExtractor : public osg::NodeVisitor {
    public:
        explicit TriangleExtractor(vector<Triangle>& triangles)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), triangles(triangles) {}

        void apply(osg::Geode& geode) override {
            for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
                osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
                if (geometry) {
                    extractFromGeometry(geometry);
                }
            }
        }
    private:
        vector<Triangle>& triangles;

        void extractFromGeometry(osg::Geometry* geometry) {
            osg::DrawElementsUInt* indices =
                dynamic_cast<osg::DrawElementsUInt*>(geometry->getPrimitiveSet(0));
            osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

            if (!indices || !vertices) return;

            // 提取三角形
            for (size_t i = 0; i < indices->size(); i += 3) {
                unsigned int i1 = (*indices)[i];
                unsigned int i2 = (*indices)[i + 1];
                unsigned int i3 = (*indices)[i + 2];

                if (i1 < vertices->size() && i2 < vertices->size() && i3 < vertices->size()) {
                    osg::Vec3 v1 = (*vertices)[i1];
                    osg::Vec3 v2 = (*vertices)[i2];
                    osg::Vec3 v3 = (*vertices)[i3];

                    triangles.push_back({
                        {v1.x(), v1.y(), v1.z()},
                        {v2.x(), v2.y(), v2.z()},
                        {v3.x(), v3.y(), v3.z()}
                        });
                }
            }
        }
    };

    // 运行 NodeVisitor
    TriangleExtractor extractor(triangles);
    node->accept(extractor);
}

//单一提取根控制块-------- 适用于L8以下
// 从OSG节点中提取所有三角形数据
// @param node OSG场景图节点指针
// @param triangles 用于存储提取出的三角形的向量
void extractTriangles(osg::Node* node, vector<Triangle>& triangles) {
    // 自定义NodeVisitor类，用于遍历场景图并提取三角形
    class TriangleExtractor : public osg::NodeVisitor {
    public:
        // 构造函数，初始化NodeVisitor并引用三角形存储容器
        // @param triangles 用于存储提取出的三角形的向量引用
        explicit TriangleExtractor(vector<Triangle>& triangles)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), triangles(triangles) {}

        // 重写apply方法，处理Geode节点
        // @param geode 当前处理的Geode节点
        void apply(osg::Geode& geode) override {
            // 遍历Geode中的所有Drawable对象
            for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
                osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
                if (geometry) {
                    extractFromGeometry(geometry);
                }
            }
        }
    private:
        vector<Triangle>& triangles;

        // 从几何体中提取三角形数据
        // @param geometry OSG几何体对象指针
        void extractFromGeometry(osg::Geometry* geometry) {
            if (!geometry) return;

            // 获取顶点数组
            osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
            if (!vertices || vertices->empty()) return;

            // 遍历所有图元集
            for (unsigned int p = 0; p < geometry->getNumPrimitiveSets(); ++p) {
                osg::PrimitiveSet* primitiveSet = geometry->getPrimitiveSet(p);
                if (!primitiveSet) continue;

                GLenum mode = primitiveSet->getMode();
                // 检查是否为支持的三角形绘制模式
                if (mode != GL_TRIANGLES && mode != GL_TRIANGLE_STRIP && mode != GL_TRIANGLE_FAN) {
                    std::cerr << "警告：未处理的绘制模式: " << mode << std::endl;
                    continue;
                }

                // 定义处理单个三角形的lambda函数
                auto processTriangle = [&](unsigned int i1, unsigned int i2, unsigned int i3) {
                    if (i1 < vertices->size() && i2 < vertices->size() && i3 < vertices->size()) {
                        osg::Vec3 v1 = (*vertices)[i1];
                        osg::Vec3 v2 = (*vertices)[i2];
                        osg::Vec3 v3 = (*vertices)[i3];
                        triangles.push_back({
                            {v1.x(), v1.y(), v1.z()},
                            {v2.x(), v2.y(), v2.z()},
                            {v3.x(), v3.y(), v3.z()}
                            });
                    }
                    };

                // 根据不同的PrimitiveSet类型提取三角形索引
                if (auto* drawElementsUInt = dynamic_cast<osg::DrawElementsUInt*>(primitiveSet)) {
                    for (size_t i = 0; i + 2 < drawElementsUInt->size(); i += 3)
                        processTriangle((*drawElementsUInt)[i], (*drawElementsUInt)[i + 1], (*drawElementsUInt)[i + 2]);
                }
                else if (auto* drawElementsUShort = dynamic_cast<osg::DrawElementsUShort*>(primitiveSet)) {
                    for (size_t i = 0; i + 2 < drawElementsUShort->size(); i += 3)
                        processTriangle((*drawElementsUShort)[i], (*drawElementsUShort)[i + 1], (*drawElementsUShort)[i + 2]);
                }
                else if (auto* drawElementsUByte = dynamic_cast<osg::DrawElementsUByte*>(primitiveSet)) {
                    for (size_t i = 0; i + 2 < drawElementsUByte->size(); i += 3)
                        processTriangle((*drawElementsUByte)[i], (*drawElementsUByte)[i + 1], (*drawElementsUByte)[i + 2]);
                }
                else if (auto* drawArrays = dynamic_cast<osg::DrawArrays*>(primitiveSet)) {
                    int first = drawArrays->getFirst();
                    int count = drawArrays->getCount();
                    for (int i = first; i + 2 < first + count; i += 3)
                        processTriangle(i, i + 1, i + 2);
                }
                else {
                    std::cerr << "警告：不支持的 PrimitiveSet 类型：" << typeid(*primitiveSet).name() << std::endl;
                }
            }
        }
    //};

    };

    // 创建TriangleExtractor实例并应用到节点
    TriangleExtractor extractor(triangles);
    node->accept(extractor);
}

//单一提取某个高级层块------------如提取Tile_xxxx_yyyyyy_L14.osgb
void extractTrianglesFromLevel(const std::string& folderPath, int level, std::vector<Triangle>& triangles) {
    namespace fs = std::filesystem;

    std::string baseName;
    for (const auto& entry : fs::directory_iterator(folderPath)) {
        std::string name = entry.path().filename().string();
        if (name.ends_with(".osgb") && name.find("_L") == std::string::npos) {
            baseName = name.substr(0, name.size() - 5); // 去掉 ".osgb"
            break;
        }
    }

    if (baseName.empty()) {
        std::cerr << "[错误] 未找到基础 .osgb 文件！" << std::endl;
        return;
    }

    fs::path targetPath = fs::path(folderPath) / constructFileName(baseName, level);
    std::string targetFile = targetPath.string(); 


    if (!fs::exists(targetFile)) {
        std::cerr << "[错误] 未找到指定层级文件: " << targetFile << std::endl;
        return;
    }

    std::cout << "[加载:提取地理信息] " << targetFile << std::endl;
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(targetFile);
    if (!node) {
        std::cerr << "[错误] 加载失败: " << targetFile << std::endl;
        return;
    }

    extractTriangles(node.get(), triangles);
}

//遍历提取多个某高级层块----------提取Tile_xxxx_yyyyyy_L14_xx_.........
// 从指定目录中的特定层级文件提取三角形数据（支持递归搜索子目录）
// @param folderDirPath 文件夹路径
// @param level 目录层级
// @param triangles 用于存储提取出的三角形的向量
void extractTrianglesFromLevelFiles(const std::string& folderDirPath, int level, std::vector<Triangle>& triangles) {
    namespace fs = std::filesystem;

    // 构造层级关键字，用于匹配文件名
    std::string levelKey = "_L" + std::to_string(level) + "_";

    // 递归遍历目录中的所有条目（包括子目录）
    for (const auto& entry : fs::recursive_directory_iterator(folderDirPath)) {
        if (!entry.is_regular_file()) continue;

        const fs::path& filePath = entry.path();
        std::string fileName = filePath.filename().string();

        // 检查文件扩展名是否为.osgb且文件名包含指定层级关键字
        if (filePath.extension() == ".osgb" && fileName.find(levelKey) != std::string::npos) {
            std::cout << "[加载:提取 L" << level << "] " << filePath << std::endl;

            // 读取OSG节点文件
            osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filePath.string());
            if (!node) {
                std::cerr << "  [×] 加载失败: " << filePath << std::endl;
                continue;
            }

            // 提取节点中的三角形数据
            extractTriangles(node.get(), triangles);
        }
    }

    // 输出提取结果统计信息
    std::cout << "[√] 层级 L" << level << "：提取完成，共计三角形数: " << triangles.size() << "\n";
}


// 从metadata.xml文件中读取源坐标系信息
std::string readSourceCRSFromXML(const std::string& dataDir) {
    namespace fs = std::filesystem;
    fs::path metadataPath = fs::path(dataDir) / "metadata.xml";
    
    if (!fs::exists(metadataPath)) {
        std::cerr << "[警告] 未找到metadata.xml文件，使用默认源坐标系" << std::endl;
        return "EPSG:4528";  // 默认值
    }
    
    // 简单的XML解析，查找<SRS>标签内容
    std::ifstream file(metadataPath);
    std::string line;
    std::string sourceCRS;
    
    while (std::getline(file, line)) {
        size_t start = line.find("<SRS>");
        if (start != std::string::npos) {
            size_t end = line.find("</SRS>");
            if (end != std::string::npos && end > start) {
                sourceCRS = line.substr(start + 5, end - start - 5);
                break;
            }
        }
    }
    
    if (sourceCRS.empty()) {
        std::cerr << "[警告] 无法从metadata.xml解析源坐标系，使用默认值" << std::endl;
        return "EPSG:4528";  // 默认值
    }
    
    std::cout << "[信息] 从metadata.xml读取源坐标系: " << sourceCRS << std::endl;
    return sourceCRS;
}

// 从metadata.xml文件中读取坐标偏移量信息
CoordinateOffset readCoordinateOffsetFromXML(const std::string& dataDir) {
    namespace fs = std::filesystem;
    fs::path metadataPath = fs::path(dataDir) / "metadata.xml";

    CoordinateOffset offset;

    if (!fs::exists(metadataPath)) {
        std::cerr << "[警告] 未找到metadata.xml文件，使用默认偏移量(0,0,0)" << std::endl;
        return offset;  // 默认值为(0,0,0)
    }

    // 简单的XML解析，查找<SRSOrigin>标签内容
    std::ifstream file(metadataPath);
    std::string line;
    std::string originStr;

    while (std::getline(file, line)) {
        size_t start = line.find("<SRSOrigin>");
        if (start != std::string::npos) {
            size_t end = line.find("</SRSOrigin>");
            if (end != std::string::npos && end > start) {
                originStr = line.substr(start + 11, end - start - 11);
                break;
            }
        }
    }

    if (originStr.empty()) {
        std::cerr << "[警告] 无法从metadata.xml解析坐标偏移量，使用默认值(0,0,0)" << std::endl;
        return offset;
    }

    // 解析偏移量字符串，格式为：x,y,z
    std::stringstream ss(originStr);
    std::string token;

    try {
        if (std::getline(ss, token, ',')) {
            offset.offsetX = std::stod(token);
        }
        if (std::getline(ss, token, ',')) {
            offset.offsetY = std::stod(token);
        }
        if (std::getline(ss, token, ',')) {
            offset.offsetZ = std::stod(token);
        }

        std::cout << "[信息] 从metadata.xml读取坐标偏移量: ("
                  << offset.offsetX << ", " << offset.offsetY << ", " << offset.offsetZ << ")" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[错误] 解析坐标偏移量失败: " << e.what() << "，使用默认值(0,0,0)" << std::endl;
        return CoordinateOffset();  // 返回默认值
    }

    return offset;
}

// 坐标转换函数：将三角形顶点从 指定系 转换为 目标系
// 输入参数：
// triangles：要转换的三角形数组引用
// sourceCRS：源坐标系字符串（为空时尝试从XML读取）
// targetCRS：目标坐标系字符串（默认WGS84）
// offsetX, offsetY, offsetZ：坐标偏移量（默认0,0,0）
void convertCoordinates(vector<Triangle>& triangles,
    const string& sourceCRS,
    const string& targetCRS,
    double offsetX , double offsetY,double offsetZ)
{
    // 如果未提供源坐标系，尝试从XML读取
    string actualSourceCRS = sourceCRS;
    if (actualSourceCRS.empty()) {
        // 这里需要知道数据目录路径，暂时使用默认值
        // 实际应用中应该从XML读取
        actualSourceCRS = "EPSG:4528";  // 默认值
        std::cout << "[信息] 使用默认源坐标系: " << actualSourceCRS << std::endl;
    }
    
    // 确保目标坐标系为WGS84
    string actualTargetCRS = targetCRS;
    if (actualTargetCRS != "EPSG:4326") {
        std::cout << "[信息] 目标坐标系已设置为WGS84 (EPSG:4326)" << std::endl;
        actualTargetCRS = "EPSG:4326";
    }
    
    // 检查是否使用了默认偏移量(0,0,0)，如果是，提示可能需要从XML读取
    if (offsetX == 0.0 && offsetY == 0.0 && offsetZ == 0.0) {
        std::cout << "[提示] 使用默认偏移量(0,0,0)，如果需要从XML读取偏移量，请使用带数据目录参数的版本" << std::endl;
    }
    
    // // 设置PROJ库路径（由CMake在编译时确定）
    // #ifdef PROJ_DATA_PATH
    //     // 如果编译时确定了 PROJ 数据路径，且当前环境未设置 PROJ_LIB，则设置它
    //     if (!std::getenv("PROJ_LIB")) {
    //         #ifdef _WIN32
    //             _putenv(("PROJ_LIB=" + std::string(PROJ_DATA_PATH)).c_str());
    //         #else
    //             setenv("PROJ_LIB", PROJ_DATA_PATH, 1);
    //         #endif
    //         std::cout << "[信息] 设置PROJ库路径: " << PROJ_DATA_PATH << std::endl;
    //     }
    // #else
    //     // 如果编译时未找到PROJ路径，检查运行时环境变量
    //     if (!std::getenv("PROJ_LIB")) {
    //         std::cerr << "[警告] 未设置PROJ_LIB环境变量，坐标转换可能失败。请确保PROJ正确安装并设置PROJ_LIB环境变量。" << std::endl;
    //     }
    // #endif

    PJ_CONTEXT* context = proj_context_create();
    if (!context) {
        cerr << "创建PROJ上下文失败" << endl;
        return;
    }

    // 创建坐标转换
    PJ* proj = proj_create_crs_to_crs(context,
        actualSourceCRS.c_str(),
        actualTargetCRS.c_str(),
        nullptr);
    if (!proj) {
        cerr << "未能成功坐标转换: "
            << actualSourceCRS << " to " << actualTargetCRS << endl;
        proj_context_destroy(context);
        return;
    }

    // 创建可视化转换（自动处理轴顺序）
    PJ* transform = proj_normalize_for_visualization(context, proj);
    proj_destroy(proj);
    if (!transform) {
        cerr << "创建可视化转换失败" << endl;
        proj_context_destroy(context);
        return;
    }

    // 执行坐标转换
    for (auto& triangle : triangles) {
        auto transformVertex = [&](PointLBHd& vertex) {
            // 应用起算点偏移量
            double adjustedLng = vertex.Lng + offsetX;
            double adjustedLat = vertex.Lat + offsetY;
            double adjustedHgt = vertex.Hgt + offsetZ;

            // 创建包含三维坐标的输入
            PJ_COORD src = proj_coord(adjustedLng, adjustedLat, adjustedHgt, 0);

            // 执行坐标转换
            PJ_COORD dst = proj_trans(transform, PJ_FWD, src);

            // 处理输出坐标
            if (proj_degree_input(transform, PJ_FWD)) {
                vertex.Lng = dst.lpzt.lam;  // 经度
                vertex.Lat = dst.lpzt.phi;   // 纬度
                vertex.Hgt = dst.lpzt.z;     // 高程
            }
            else {
                vertex.Lng = dst.xyzt.x;     // X坐标
                vertex.Lat = dst.xyzt.y;     // Y坐标
                vertex.Hgt = dst.xyzt.z;     // 高程
            }
            };

        transformVertex(triangle.vertex1);
        transformVertex(triangle.vertex2);
        transformVertex(triangle.vertex3);
    }

    // 清理资源
    proj_destroy(transform);
    proj_context_destroy(context);
    
    std::cout << "[信息] 坐标转换完成: " << actualSourceCRS << " -> " << actualTargetCRS << std::endl;
}

// 坐标转换函数：将三角形顶点从 XML 中指定的源坐标系 转换为 目标坐标系
// 输入参数：
// triangles：要转换的三角形数组引用
// dataDir：包含 metadata.xml 的数据目录路径
// targetCRS：目标坐标系，默认WGS84 (EPSG:4326)
void convertCoordinatesFromXML(vector<Triangle>& triangles,
    const std::string& dataDir,
    const string& targetCRS)
{
    // 从XML读取源坐标系
    std::string sourceCRS = readSourceCRSFromXML(dataDir);

    // 从XML读取坐标偏移量
    CoordinateOffset offset = readCoordinateOffsetFromXML(dataDir);

    // 确保目标坐标系为WGS84
    string actualTargetCRS = targetCRS;
    if (actualTargetCRS != "EPSG:4326") {
        std::cout << "[信息] 目标坐标系已设置为WGS84 (EPSG:4326)" << std::endl;
        actualTargetCRS = "EPSG:4326";
    }

    std::cout << "[信息] 开始坐标转换: " << sourceCRS << " -> " << actualTargetCRS
              << "，偏移量: (" << offset.offsetX << ", " << offset.offsetY << ", " << offset.offsetZ << ")" << std::endl;

    // 调用原版本的转换函数
    convertCoordinates(triangles, sourceCRS, actualTargetCRS, offset.offsetX, offset.offsetY, offset.offsetZ);
}

// OSG坐标转换函数：将XML中的顶点坐标转换为OSG使用的本地坐标系统
// 输入参数：
// vertices：要转换的顶点数组引用
// offsetX, offsetY, offsetZ：坐标偏移量，用于调整转换后的坐标
// 返回值：转换后的OSG Vec3d 数组
std::vector<osg::Vec3d> convertOSG_Coordinates(const std::vector<Vertex>& vertices,
    double offsetX, double offsetY, double offsetZ) 
{
    PJ_CONTEXT* C = proj_context_create();
    PJ* P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:2326", nullptr);
    PJ* P_for_GIS = proj_normalize_for_visualization(C, P);
    std::vector<osg::Vec3d> projectedVertices;

    for (const auto& v : vertices) {
        PJ_COORD input = proj_coord(v.lon, v.lat, v.alt, 0);
        PJ_COORD output = proj_trans(P_for_GIS, PJ_FWD, input);

        if (proj_errno(P_for_GIS) != 0) {
            std::cerr << "坐标转换错误: " << proj_errno_string(proj_errno(P_for_GIS)) << std::endl;
        }

        double x = output.xyz.x - offsetX;
        double y = output.xyz.y - offsetY;
        double z = output.xyz.z-  offsetZ ;

        projectedVertices.emplace_back(x, y, z);
    }

    proj_destroy(P_for_GIS);
    proj_destroy(P);
    proj_context_destroy(C);
    return projectedVertices;
}

// 三角面片立体模型填充网格计算
// 输入参数：
// faces：多面体的三角形面片数组，每个元素为3个顶点的数组
// baseTile：基础图块，用于确定网格采样范围
// level：图块级别，用于确定采样步长
// sampleMultiplier：采样倍率，用于调整采样密度
// maxSamples：最大采样数，用于限制总采样数量
// 返回值：填充网格结果结构体，包含网格顶点、索引和其他相关信息
PolyhedronGridResult computePolyhedronGridFill(
    const std::vector<std::array<PointLBHd, 3>>& faces,
    const BaseTile& baseTile,
    int level,
    double sampleMultiplier,
    size_t maxSamples) {

    PolyhedronGridResult result;

    // ==================== 1. 边界计算 ====================

    // 计算多面体的边界框，用于确定采样范围
    double minlon = std::numeric_limits<double>::infinity(), maxlon = -minlon;
    double minlat = std::numeric_limits<double>::infinity(), maxlat = -minlat;
    double minh = std::numeric_limits<double>::infinity(), maxh = -minh;

    // 遍历所有三角形面片，找出经度、纬度和高度的最小最大值
    for (const auto& triangle : faces) {
        for (int k = 0; k < 3; ++k) {
            minlon = std::min(minlon, triangle[k].Lng);
            maxlon = std::max(maxlon, triangle[k].Lng);
            minlat = std::min(minlat, triangle[k].Lat);
            maxlat = std::max(maxlat, triangle[k].Lat);
            minh = std::min(minh, triangle[k].Hgt);
            maxh = std::max(maxh, triangle[k].Hgt);
        }
    }

    // ==================== 2. 网格参数计算 ====================

    // 计算所有顶点的几何中心，用于确定网格步长
    double centerLon = 0, centerLat = 0;
    size_t vertexCount = 0;
    for (const auto &triangle : faces) {
        for (int i = 0; i < 3; ++i) {
            centerLon += triangle[i].Lng;
            centerLat += triangle[i].Lat;
            vertexCount++;
        }
    }
    centerLon /= static_cast<double>(vertexCount);
    centerLat /= static_cast<double>(vertexCount);

    // 计算中心高度
    double centerHeight = 0;
    for (const auto &triangle : faces) {
        for (int i = 0; i < 3; ++i) {
            centerHeight += triangle[i].Hgt;
        }
    }
    centerHeight /= static_cast<double>(vertexCount);

    // 度到米的转换系数（考虑地球曲率）
    const double DEG_TO_M_LAT = 111320.0;  // 纬度方向近似固定，每度约111.32公里
    const double DEG_TO_M_LON = 111320.0 * std::cos(centerLat * M_PI / 180.0);  // 经度方向随纬度变化

    // 获取中心点的网格编码
    std::string centerCode = getLocalCode(static_cast<uint8_t>(level), centerLon, centerLat, centerHeight, baseTile);

    // 计算网格步长和基础尺寸
    double lonStep = 0.001, latStep = 0.001, vertStep = 1.0;
    double baseLon = 0.001, baseLat = 0.001, baseVert = 1.0;

    // 根据中心点网格编码计算实际步长
    if (!centerCode.empty()) {
        LatLonHei tileInfo = getLocalTileLatLon(centerCode, baseTile);
        baseLon = std::max(1e-9, tileInfo.east - tileInfo.west);
        baseLat = std::max(1e-9, tileInfo.north - tileInfo.south);
        baseVert = std::max(1e-6, tileInfo.top - tileInfo.bottom);
        lonStep = baseLon / sampleMultiplier;
        latStep = baseLat / sampleMultiplier;
        vertStep = baseVert / sampleMultiplier;
    }

    // ==================== 3. 几何计算辅助函数定义 ====================

    // 将经纬度坐标转换为米坐标（局部坐标系）
    auto toMeters = [&](const PointLBHd& point) -> V3 {
        return V3{
            point.Lng * DEG_TO_M_LON,
            point.Lat * DEG_TO_M_LAT,
            point.Hgt
        };
    };

    // 使用类型别名定义米坐标系的三角形（三个V3顶点）
    using TriangleMeters = std::array<V3, 3>;

    // 转换所有三角形到米为单位
    std::vector<TriangleMeters> trianglesMeters;
    trianglesMeters.reserve(faces.size());
    for (const auto &triangle : faces) {
        trianglesMeters.push_back({
            toMeters(triangle[0]),
            toMeters(triangle[1]),
            toMeters(triangle[2])
        });
    }

    // 射线与三角形相交检测（Möller-Trumbore算法）
    auto rayIntersectsTriangle = [&](const V3 &origin, const V3 &direction,
                                   const TriangleMeters &triangle, double &outT) -> bool {
        const double EPSILON = 1e-12;

        // 计算三角形的两条边向量
        V3 edge1{ triangle[1].x - triangle[0].x, triangle[1].y - triangle[0].y, triangle[1].z - triangle[0].z };
        V3 edge2{ triangle[2].x - triangle[0].x, triangle[2].y - triangle[0].y, triangle[2].z - triangle[0].z };

        // 计算射线方向与三角形边2的叉积
        V3 pvec{ direction.y * edge2.z - direction.z * edge2.y,
                 direction.z * edge2.x - direction.x * edge2.z,
                 direction.x * edge2.y - direction.y * edge2.x };

        // 计算行列式，判断射线是否与三角形平行
        double det = edge1.x * pvec.x + edge1.y * pvec.y + edge1.z * pvec.z;
        if (std::abs(det) < EPSILON) return false;  // 射线与三角形平行

        double invDet = 1.0 / det;
        V3 tvec{ origin.x - triangle[0].x, origin.y - triangle[0].y, origin.z - triangle[0].z };

        // 计算重心坐标u
        double u = (tvec.x * pvec.x + tvec.y * pvec.y + tvec.z * pvec.z) * invDet;
        if (u < -EPSILON || u > 1.0 + EPSILON) return false;

        // 计算重心坐标v
        V3 qvec{ tvec.y * edge1.z - tvec.z * edge1.y,
                 tvec.z * edge1.x - tvec.x * edge1.z,
                 tvec.x * edge1.y - tvec.y * edge1.x };

        double v = (direction.x * qvec.x + direction.y * qvec.y + direction.z * qvec.z) * invDet;
        if (v < -EPSILON || u + v > 1.0 + EPSILON) return false;

        // 计算交点参数t
        double t = (edge2.x * qvec.x + edge2.y * qvec.y + edge2.z * qvec.z) * invDet;
        if (t <= EPSILON) return false;  // 相交点在射线起点后方

        outT = t;
        return true;
    };

    // 判断点是否在多面体内部（射线投射算法）
    auto pointInsidePolyhedron = [&](const V3 &point) -> bool {
        V3 rayDirection{1.0, 0.0, 0.0};  // X轴正方向射线
        int intersectionCount = 0;
        double dummyT;

        // 计算射线与所有三角形面片的交点数量
        for (const auto &triangle : trianglesMeters) {
            if (rayIntersectsTriangle(point, rayDirection, triangle, dummyT)) {
                ++intersectionCount;
            }
        }
        return (intersectionCount % 2) == 1;  // 奇数交点表示在内部
    };

    // 三角形与包围盒相交检测（分离轴定理）
    auto triangleBoxOverlap = [&](const V3 &boxCenter, const V3 &boxHalfSize,
                                 const V3 &v0, const V3 &v1, const V3 &v2) -> bool {
        // 将三角形顶点转换到包围盒本地坐标系
        double v0x = v0.x - boxCenter.x, v0y = v0.y - boxCenter.y, v0z = v0.z - boxCenter.z;
        double v1x = v1.x - boxCenter.x, v1y = v1.y - boxCenter.y, v1z = v1.z - boxCenter.z;
        double v2x = v2.x - boxCenter.x, v2y = v2.y - boxCenter.y, v2z = v2.z - boxCenter.z;

        // 计算三角形的三条边向量
        double e0x = v1x - v0x, e0y = v1y - v0y, e0z = v1z - v0z;
        double e1x = v2x - v1x, e1y = v2y - v1y, e1z = v2z - v1z;
        double e2x = v0x - v2x, e2y = v0y - v2y, e2z = v0z - v2z;

        const double EPSILON = 1e-12;

        // 轴测试函数：测试给定轴是否为分离轴
        auto testAxis = [&](double ax, double ay, double az) -> bool {
            // 计算三角形顶点在轴上的投影
            double p0 = v0x * ax + v0y * ay + v0z * az;
            double p1 = v1x * ax + v1y * ay + v1z * az;
            double p2 = v2x * ax + v2y * ay + v2z * az;
            // 计算包围盒在轴上的投影半径
            double r = boxHalfSize.x * std::abs(ax) + boxHalfSize.y * std::abs(ay) + boxHalfSize.z * std::abs(az);
            double minp = std::min({p0, p1, p2});
            double maxp = std::max({p0, p1, p2});
            // 如果投影区间不重叠，则该轴为分离轴
            return !(minp > r + EPSILON || maxp < -r - EPSILON);
        };

        // 测试9个分离轴（3个面的法线 + 6个叉积轴）
        if (!testAxis(0, e0z, -e0y)) return false;
        if (!testAxis(-e0z, 0, e0x)) return false;
        if (!testAxis(e0y, -e0x, 0)) return false;
        if (!testAxis(0, e1z, -e1y)) return false;
        if (!testAxis(-e1z, 0, e1x)) return false;
        if (!testAxis(e1y, -e1x, 0)) return false;
        if (!testAxis(0, e2z, -e2y)) return false;
        if (!testAxis(-e2z, 0, e2x)) return false;
        if (!testAxis(e2y, -e2x, 0)) return false;

        // AABB重叠测试（轴对齐包围盒）
        double minx = std::min({v0x, v1x, v2x}), maxx = std::max({v0x, v1x, v2x});
        double miny = std::min({v0y, v1y, v2y}), maxy = std::max({v0y, v1y, v2y});
        double minz = std::min({v0z, v1z, v2z}), maxz = std::max({v0z, v1z, v2z});

        if (minx > boxHalfSize.x + EPSILON || maxx < -boxHalfSize.x - EPSILON) return false;
        if (miny > boxHalfSize.y + EPSILON || maxy < -boxHalfSize.y - EPSILON) return false;
        if (minz > boxHalfSize.z + EPSILON || maxz < -boxHalfSize.z - EPSILON) return false;

        // 三角形平面与包围盒中心测试
        double nx = e0y * e1z - e0z * e1y;  // 平面法向量X分量
        double ny = e0z * e1x - e0x * e1z;  // 平面法向量Y分量
        double nz = e0x * e1y - e0y * e1x;  // 平面法向量Z分量
        double d = -(nx * v0x + ny * v0y + nz * v0z);  // 平面方程常数项
        double r = boxHalfSize.x * std::abs(nx) + boxHalfSize.y * std::abs(ny) + boxHalfSize.z * std::abs(nz);

        return !(std::abs(d) > r + EPSILON);
    };

    // 判断网格单元是否与多面体相交
    auto tileIntersectsPolyhedron = [&](const LatLonHei &tile) -> bool {
        // 将网格单元转换到米坐标系
        double cellMinX = tile.west * DEG_TO_M_LON;
        double cellMaxX = tile.east * DEG_TO_M_LON;
        double cellMinY = tile.south * DEG_TO_M_LAT;
        double cellMaxY = tile.north * DEG_TO_M_LAT;
        double cellMinZ = tile.bottom;
        double cellMaxZ = tile.top;

        // 计算包围盒中心和半尺寸
        V3 boxCenter{ (cellMinX + cellMaxX) * 0.5, (cellMinY + cellMaxY) * 0.5, (cellMinZ + cellMaxZ) * 0.5 };
        V3 halfSize{ (cellMaxX - cellMinX) * 0.5, (cellMaxY - cellMinY) * 0.5, (cellMaxZ - cellMinZ) * 0.5 };

        // 计算多面体的边界框
        double polyMinX = std::numeric_limits<double>::infinity(), polyMaxX = -polyMinX;
        double polyMinY = std::numeric_limits<double>::infinity(), polyMaxY = -polyMinY;
        double polyMinZ = std::numeric_limits<double>::infinity(), polyMaxZ = -polyMinZ;

        for (const auto &triangle : trianglesMeters) {
            polyMinX = std::min({polyMinX, triangle[0].x, triangle[1].x, triangle[2].x});
            polyMaxX = std::max({polyMaxX, triangle[0].x, triangle[1].x, triangle[2].x});
            polyMinY = std::min({polyMinY, triangle[0].y, triangle[1].y, triangle[2].y});
            polyMaxY = std::max({polyMaxY, triangle[0].y, triangle[1].y, triangle[2].y});
            polyMinZ = std::min({polyMinZ, triangle[0].z, triangle[1].z, triangle[2].z});
            polyMaxZ = std::max({polyMaxZ, triangle[0].z, triangle[1].z, triangle[2].z});
        }

        // 快速边界框测试：如果两个边界框不相交，则直接返回false
        if (polyMaxX < cellMinX - 1e-9 || polyMinX > cellMaxX + 1e-9) return false;
        if (polyMaxY < cellMinY - 1e-9 || polyMinY > cellMaxY + 1e-9) return false;
        if (polyMaxZ < cellMinZ - 1e-9 || polyMinZ > cellMaxZ + 1e-9) return false;

        // 精确相交测试：检查每个三角形是否与包围盒相交
        for (const auto &triangle : trianglesMeters) {
            if (triangleBoxOverlap(boxCenter, halfSize, triangle[0], triangle[1], triangle[2])) {
                return true;
            }
        }

        // 顶点包含测试：检查多面体顶点是否在网格单元内
        for (const auto &triangle : trianglesMeters) {
            const V3* vertices = triangle.data();
            for (int vi = 0; vi < 3; ++vi) {
                const V3 &v = vertices[vi];
                if (v.x >= cellMinX - 1e-9 && v.x <= cellMaxX + 1e-9 &&
                    v.y >= cellMinY - 1e-9 && v.y <= cellMaxY + 1e-9 &&
                    v.z >= cellMinZ - 1e-9 && v.z <= cellMaxZ + 1e-9) {
                    return true;
                }
            }
        }

        // 包围盒中心或角点包含测试：检查包围盒的关键点是否在多面体内部
        if (pointInsidePolyhedron(boxCenter)) return true;

        // 检查包围盒的8个角点
        for (int xi = 0; xi < 2; ++xi) {
            for (int yi = 0; yi < 2; ++yi) {
                for (int zi = 0; zi < 2; ++zi) {
                    V3 corner{ xi ? cellMaxX : cellMinX, yi ? cellMaxY : cellMinY, zi ? cellMaxZ : cellMinZ };
                    if (pointInsidePolyhedron(corner)) return true;
                }
            }
        }

        return false;
    };

    // ==================== 4. 凸包构建和水平裁剪 ====================

    // 收集所有顶点的水平投影点（用于凸包计算）
    std::vector<std::pair<double, double>> points2D;
    for (const auto &triangle : faces) {
        for (int i = 0; i < 3; ++i) {
            points2D.emplace_back(triangle[i].Lng, triangle[i].Lat);
        }
    }

    // 排序和去重
    std::sort(points2D.begin(), points2D.end(),
              [](const auto &a, const auto &b) {
                  if (a.first != b.first) return a.first < b.first;
                  return a.second < b.second;
              });
    points2D.erase(std::unique(points2D.begin(), points2D.end()), points2D.end());

    // 计算凸包（Andrew算法）
    std::vector<std::pair<double, double>> lower, upper, hull;
    auto crossProduct = [](const std::pair<double, double> &o,
                          const std::pair<double, double> &a,
                          const std::pair<double, double> &b) {
        return (a.first - o.first) * (b.second - o.second) -
               (a.second - o.second) * (b.first - o.first);
    };

    // 构建下凸包
    for (const auto &point : points2D) {
        while (lower.size() >= 2 && crossProduct(lower[lower.size() - 2], lower.back(), point) <= 0) {
            lower.pop_back();
        }
        lower.push_back(point);
    }

    // 构建上凸包
    for (auto it = points2D.rbegin(); it != points2D.rend(); ++it) {
        while (upper.size() >= 2 && crossProduct(upper[upper.size() - 2], upper.back(), *it) <= 0) {
            upper.pop_back();
        }
        upper.push_back(*it);
    }

    // 合并凸包（移除重复的端点）
    for (size_t i = 0; i < lower.size(); ++i) {
        hull.push_back(lower[i]);
    }
    for (size_t i = 1; i + 1 < upper.size(); ++i) {
        hull.push_back(upper[i]);
    }

    // 判断点是否在凸多边形内部
    auto pointInsideConvexPolygon = [&](double x, double y) -> bool {
        if (hull.empty()) return false;

        const double tolerance = 1e-12;
        double sign = 0;
        size_t n = hull.size();

        // 通过叉积符号判断点是否在多边形内部
        for (size_t i = 0; i < n; ++i) {
            const auto &p1 = hull[i];
            const auto &p2 = hull[(i + 1) % n];
            double cross = (p2.first - p1.first) * (y - p1.second) -
                           (p2.second - p1.second) * (x - p1.first);

            if (std::abs(cross) < tolerance) continue;  // 点在边上
            if (sign == 0) sign = cross;
            else if (sign * cross < -tolerance) return false;  // 点在多边形外部
        }
        return true;
    };

    // ==================== 5. 自适应采样和网格单元检测 ====================

    std::unordered_set<std::string> gridCodes;
    const double MAX_MULTIPLIER = 64.0;
    double currentMultiplier = sampleMultiplier;
    bool performedSampling = false;
    double usedMultiplier = sampleMultiplier;
    size_t totalSamples = 0;

    // 自适应采样：从精细到粗糙，直到满足内存限制
    for (; currentMultiplier <= MAX_MULTIPLIER; currentMultiplier *= 2.0) {
        // 根据当前倍数调整网格步长
        lonStep = baseLon / currentMultiplier;
        latStep = baseLat / currentMultiplier;
        vertStep = baseVert / currentMultiplier;

        // 计算采样网格尺寸
        size_t nLon = std::max<size_t>(1, static_cast<size_t>(std::ceil((maxlon - minlon) / lonStep)));
        size_t nLat = std::max<size_t>(1, static_cast<size_t>(std::ceil((maxlat - minlat) / latStep)));
        size_t nVert = std::max<size_t>(1, static_cast<size_t>(std::ceil((maxh - minh) / vertStep)));

        // 估算采样点数量，如果超过最大限制则停止
        double estimatedSamples = static_cast<double>(nLon) * static_cast<double>(nLat) * static_cast<double>(nVert);
        if (estimatedSamples > static_cast<double>(maxSamples) && maxSamples > 0) break;

        performedSampling = true;
        usedMultiplier = currentMultiplier;

        // 水平采样循环
        for (size_t iy = 0; iy < nLat; ++iy) {
            double sampleLat = minlat + (static_cast<double>(iy) + 0.5) * latStep;
            if (sampleLat < minlat - 1e-12 || sampleLat > maxlat + 1e-12) continue;

            for (size_t ix = 0; ix < nLon; ++ix) {
                double sampleLon = minlon + (static_cast<double>(ix) + 0.5) * lonStep;
                if (sampleLon < minlon - 1e-12 || sampleLon > maxlon + 1e-12) continue;

                // 凸包快速剔除：如果采样点不在凸包内，则跳过
                if (!pointInsideConvexPolygon(sampleLon, sampleLat)) continue;

                V3 baseXY{ sampleLon * DEG_TO_M_LON, sampleLat * DEG_TO_M_LAT, 0.0 };
                double currentZ = minh;
                size_t verticalIterations = 0;
                const size_t MAX_VERTICAL_ITERATIONS = maxSamples > 0 ? std::max<size_t>(10000, maxSamples) : 10000000;

                // 垂直方向采样
                while (currentZ <= maxh + 1e-9 && verticalIterations < MAX_VERTICAL_ITERATIONS) {
                    double queryHeight = std::max(currentZ, minh);
                    std::string gridCode = getLocalCode(static_cast<uint8_t>(level),
                                                        sampleLon, sampleLat, queryHeight, baseTile);
                    if (gridCode.empty()) break;

                    LatLonHei tile = getLocalTileLatLon(gridCode, baseTile);
                    double sampleHeight = (tile.bottom + tile.top) * 0.5;

                    // 跳过在高度范围之外的瓦片
                    if (tile.top < minh - 1e-9) {
                        currentZ = tile.top + 1e-9;
                        ++verticalIterations;
                        continue;
                    }
                    if (tile.bottom > maxh + 1e-9) break;

                    V3 testPoint{ baseXY.x, baseXY.y, sampleHeight };
                    ++totalSamples;

                    // 包含测试：判断采样点是否在多面体内部或与多面体相交
                    if (pointInsidePolyhedron(testPoint)) {
                        gridCodes.insert(gridCode);
                    } else if (tileIntersectsPolyhedron(tile)) {
                        gridCodes.insert(gridCode);
                    }

                    currentZ = tile.top + 1e-9;
                    ++verticalIterations;

                    if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
                }

                if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
            }

            if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
        }

        if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
    }

    // 如果自适应采样没有执行（例如估算超出限制），使用最粗糙的采样
    if (!performedSampling) {
        lonStep = baseLon;
        latStep = baseLat;
        vertStep = baseVert;
        usedMultiplier = 1.0;

        for (double lat = minlat; lat <= maxlat + 1e-12; lat += latStep) {
            for (double lon = minlon; lon <= maxlon + 1e-12; lon += lonStep) {
                double sampleLon = lon + lonStep * 0.5;
                double sampleLat = lat + latStep * 0.5;
                if (!pointInsideConvexPolygon(sampleLon, sampleLat)) continue;

                double zStart = std::floor(minh / vertStep) * vertStep;
                double zEnd = std::ceil(maxh / vertStep) * vertStep;
                double currentZ2 = zStart;
                size_t verticalIterations2 = 0;
                const size_t MAX_VERTICAL_ITERATIONS2 = maxSamples > 0 ? std::max<size_t>(10000, maxSamples) : 10000000;

                while (currentZ2 <= zEnd + 1e-9 && verticalIterations2 < MAX_VERTICAL_ITERATIONS2) {
                    double queryHeight = std::max(currentZ2, minh);
                    std::string gridCode = getLocalCode(static_cast<uint8_t>(level),
                                                        sampleLon, sampleLat, queryHeight, baseTile);
                    if (gridCode.empty()) break;

                    LatLonHei tile = getLocalTileLatLon(gridCode, baseTile);
                    double sampleHeight = (tile.bottom + tile.top) * 0.5;

                    if (tile.top < minh - 1e-9) {
                        currentZ2 = tile.top + 1e-9;
                        ++verticalIterations2;
                        continue;
                    }
                    if (tile.bottom > maxh + 1e-9) break;

                    V3 testPoint{ sampleLon * DEG_TO_M_LON, sampleLat * DEG_TO_M_LAT, sampleHeight };
                    ++totalSamples;

                    if (pointInsidePolyhedron(testPoint)) {
                        gridCodes.insert(gridCode);
                    } else if (tileIntersectsPolyhedron(tile)) {
                        gridCodes.insert(gridCode);
                    }

                    currentZ2 = tile.top + 1e-9;
                    ++verticalIterations2;

                    if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
                }

                if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
            }

            if (maxSamples > 0 && gridCodes.size() >= maxSamples) break;
        }
    }

    // ==================== 6. 结果构建 ====================

    // 如果没有找到任何网格单元，至少返回中心点的编码
    if (gridCodes.empty() && !centerCode.empty()) {
        gridCodes.insert(centerCode);
    }

    // 填充结果结构
    result.gridCodes = std::move(gridCodes);
    result.multiplierUsed = usedMultiplier;
    result.actualSamples = totalSamples;
    result.hitMaxSamples = maxSamples > 0 ? (gridCodes.size() >= maxSamples) : false;
    result.centerCode = centerCode;

    return result;
}
