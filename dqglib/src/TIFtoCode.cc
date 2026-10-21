#include <dqg/TIFtoCode.h>
#include <iostream>
#include <cmath>
#include <unordered_set>
#include <drogon/drogon.h>
#include <dqg/DQG3DTil.h>
#include <fstream>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <memory>
#include <json/json.h>

// 补齐 GDAL 强依赖的 C 风格宏定义
#ifndef MIN
#define MIN(a,b)      (((a)<(b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b)      (((a)>(b)) ? (a) : (b))
#endif

// 使用 GDAL 读取包含真实地理坐标的 GeoTIFF
#include <gdal/gdal_priv.h>
#include <gdal/cpl_conv.h>
#include <gdal/cpl_port.h>

namespace TIFGridProcessor {

    /// 每次网格化任务重新读取研究区域，任务内保持同一份边界。
    static BaseTile getRegionBounds() {
        std::ifstream regionFile("./region.json");
        if (!regionFile.is_open()) {
            regionFile.open("../region.json");
        }
        if (!regionFile.is_open()) {
            throw std::runtime_error("无法读取region.json");
        }
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(regionFile, root)) {
            throw std::runtime_error("region.json 解析失败");
        }
        if (!root.isObject() || !root["region"].isObject() ||
            !root["region"]["bounds"].isObject()) {
            throw std::runtime_error("region.json 中的 region.bounds 无效");
        }
        const auto& bounds = root["region"]["bounds"];

        BaseTile region{};
        region.west = region.south = std::numeric_limits<double>::infinity();
        region.east = region.north = -std::numeric_limits<double>::infinity();
        for (const char* corner : {"southwest", "northwest", "northeast", "southeast"}) {
            const auto& point = bounds[corner];
            if (!point.isObject() || !point["longitude"].isNumeric() ||
                !point["latitude"].isNumeric()) {
                throw std::runtime_error(std::string("region.json 缺少有效角点: ") + corner);
            }
            const double longitude = point["longitude"].asDouble();
            const double latitude = point["latitude"].asDouble();
            if (!std::isfinite(longitude) || !std::isfinite(latitude) ||
                longitude < -180.0 || longitude > 180.0 ||
                latitude < -90.0 || latitude > 90.0) {
                throw std::runtime_error(std::string("region.json 角点经纬度越界: ") + corner);
            }
            region.west = std::min(region.west, longitude);
            region.east = std::max(region.east, longitude);
            region.south = std::min(region.south, latitude);
            region.north = std::max(region.north, latitude);
        }
        if (region.west >= region.east || region.south >= region.north) {
            throw std::runtime_error("region.json 研究区域宽度和高度必须大于零");
        }
        return region;
    }

    bool readDEMBlockToPoints(GDALRasterBand* poBand, double* adfGeoTransform,
                              int xOff, int yOff, int xSize, int ySize,
                              std::vector<PointLBHd>& outPoints) {

        outPoints.resize(xSize * ySize);
        std::vector<float> scanline(xSize);

        for (int row = 0; row < ySize; ++row) {
            CPLErr err = poBand->RasterIO(GF_Read, xOff, yOff + row, xSize, 1, scanline.data(), xSize, 1, GDT_Float32, 0, 0);
            if (err != CE_None) return false;

            for (int col = 0; col < xSize; ++col) {
                int globalCol = xOff + col;
                int globalRow = yOff + row;

                double lon = adfGeoTransform[0] + (globalCol + 0.5) * adfGeoTransform[1] + (globalRow + 0.5) * adfGeoTransform[2];
                double lat = adfGeoTransform[3] + (globalCol + 0.5) * adfGeoTransform[4] + (globalRow + 0.5) * adfGeoTransform[5];
                double hgt = static_cast<double>(scanline[col]);

                outPoints[row * xSize + col] = {lon, lat, hgt};
            }
        }
        return true;
    }

    static std::vector<Triangle> buildTINInRegion(const std::vector<PointLBHd>& points, int width, int height,
                                                double noDataValue, const BaseTile& region) {
        std::vector<Triangle> faces;
        faces.reserve((width - 1) * (height - 1) * 2);

        const auto outsideRegion = [&region](const PointLBHd& p) {
            return p.Lng < region.west || p.Lng > region.east ||
                   p.Lat < region.south || p.Lat > region.north;
        };

        for (int y = 0; y < height - 1; ++y) {
            for (int x = 0; x < width - 1; ++x) {
                const PointLBHd& p1 = points[y * width + x];
                const PointLBHd& p2 = points[y * width + (x + 1)];
                const PointLBHd& p3 = points[(y + 1) * width + x];
                const PointLBHd& p4 = points[(y + 1) * width + (x + 1)];

                if (std::abs(p1.Hgt - noDataValue) < 1e-5 || std::abs(p2.Hgt - noDataValue) < 1e-5 ||
                    std::abs(p3.Hgt - noDataValue) < 1e-5 || std::abs(p4.Hgt - noDataValue) < 1e-5) {
                    continue;
                }

                // 小块任一顶点越界就跳过，不生成三角面及后续网格编码。
                if (outsideRegion(p1) || outsideRegion(p2) ||
                    outsideRegion(p3) || outsideRegion(p4)) {
                    continue;
                }
                faces.push_back({p1, p4, p3});
                faces.push_back({p1, p2, p4});
            }
        }
        return faces;
    }

    std::vector<Triangle> buildTIN(const std::vector<PointLBHd>& points, int width, int height, double noDataValue) {
        return buildTINInRegion(points, width, height, noDataValue, getRegionBounds());
    }

    size_t convertTIFtoGridCodes(const std::string &tifFilePath, const TIFGridProcessor::TIFGridConfig &config, std::function<void(const std::vector<std::string>&)> batchCallback) {
        const BaseTile region = getRegionBounds();
        const BaseTile baseTile = ::getProjectBaseTile();
        if (region.west < baseTile.west || region.east > baseTile.east ||
            region.south < baseTile.south || region.north > baseTile.north) {
            throw std::runtime_error("region.json 超出当前基础瓦片，请重启服务更新基础瓦片后再网格化");
        }
        GDALAllRegister();

        // ✨ 限制 GDAL 缓存上限，防止超大 TIF 撑爆内存
        CPLSetConfigOption("GDAL_CACHEMAX", "256");

        // 后续计算或写入回调抛出异常时也必须关闭 TIFF。
        std::unique_ptr<GDALDataset, decltype(&GDALClose)> poDataset(
            static_cast<GDALDataset*>(GDALOpen(tifFilePath.c_str(), GA_ReadOnly)), GDALClose);

        if (poDataset == nullptr) {
            throw std::runtime_error("无法打开 TIF 文件");
        }

        int totalWidth = poDataset->GetRasterXSize();
        int totalHeight = poDataset->GetRasterYSize();
        GDALRasterBand* poBand = poDataset->GetRasterBand(1);
        if (!poBand) {
            throw std::runtime_error("TIF 文件缺少高程波段");
        }

        double adfGeoTransform[6];
        if (poDataset->GetGeoTransform(adfGeoTransform) != CE_None) {
            throw std::runtime_error("TIF 文件不包含地理坐标变换信息");
        }

        std::unordered_set<std::string> batchGridCodes;
        size_t totalGenerated = 0;

        // 回调批次严格不超过 100000，使单条入库 SQL 最多处理十万个网格。
        const size_t BATCH_LIMIT = 100000;

        const int BLOCK_SIZE = 512;
        const int STEP = BLOCK_SIZE - 1;

        int xBlocks = std::ceil((double)(totalWidth - 1) / STEP);
        int yBlocks = std::ceil((double)(totalHeight - 1) / STEP);
        int totalBlocks = xBlocks * yBlocks;
        int currentBlock = 0;

        LOG_INFO << "开始分块处理 TIF，总块数: " << totalBlocks << " (总尺寸: " << totalWidth << "x" << totalHeight << ")";

        for (int yOff = 0; yOff < totalHeight - 1; yOff += STEP) {
            for (int xOff = 0; xOff < totalWidth - 1; xOff += STEP) {
                currentBlock++;

                int curWidth = std::min(BLOCK_SIZE, totalWidth - xOff);
                int curHeight = std::min(BLOCK_SIZE, totalHeight - yOff);

                if (curWidth < 2 || curHeight < 2) continue;

                if (currentBlock % (totalBlocks/10 + 1) == 0 || currentBlock == 1) {
                     LOG_INFO << "[进度 " << currentBlock << "/" << totalBlocks << "] 处理分块，坐标(" << xOff << "," << yOff << ")";
                }

                std::vector<PointLBHd> points;
                if (!readDEMBlockToPoints(poBand, adfGeoTransform, xOff, yOff, curWidth, curHeight, points)) {
                    throw std::runtime_error("读取 TIF 高程分块失败");
                }

                auto faces = buildTINInRegion(points, curWidth, curHeight, config.noDataValue, region);

                // ✨ 立即强制释放散点内存
                std::vector<PointLBHd>().swap(points);

                if (faces.empty()) continue;

                std::vector<std::string> localCodes = triangular_multiple(faces, config.level, baseTile);

                // ✨ 立即强制释放三角网内存
                std::vector<Triangle>().swap(faces);

                for (const auto& code : localCodes) {
                    batchGridCodes.insert(code);
                    if (batchGridCodes.size() >= BATCH_LIMIT) {
                        if (batchCallback) {
                            batchCallback(std::vector<std::string>(batchGridCodes.begin(), batchGridCodes.end()));
                        }
                        totalGenerated += batchGridCodes.size();
                        batchGridCodes.clear();
                    }
                }
            }
        }

        if (!batchGridCodes.empty()) {
            if (batchCallback) {
                batchCallback(std::vector<std::string>(batchGridCodes.begin(), batchGridCodes.end()));
            }
            totalGenerated += batchGridCodes.size();
            std::unordered_set<std::string>().swap(batchGridCodes);
        }

        LOG_INFO << "TIF 算法提取完毕！累计生成网格: " << totalGenerated << " 个";

        return totalGenerated;
    }
}
