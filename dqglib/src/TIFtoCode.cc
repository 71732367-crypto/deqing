#include <dqg/TIFtoCode.h>
#include <iostream>
#include <cmath>
#include <unordered_set>
#include <drogon/drogon.h>
#include <dqg/DQG3DTil.h>

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

    std::vector<Triangle> buildTIN(const std::vector<PointLBHd>& points, int width, int height, double noDataValue) {
        std::vector<Triangle> faces;
        faces.reserve((width - 1) * (height - 1) * 2);

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

                faces.push_back({p1, p4, p3});
                faces.push_back({p1, p2, p4});
            }
        }
        return faces;
    }

    size_t convertTIFtoGridCodes(const std::string &tifFilePath, const TIFGridProcessor::TIFGridConfig &config, std::function<void(const std::vector<std::string>&)> batchCallback) {
        GDALAllRegister();

        // ✨ 限制 GDAL 缓存上限，防止超大 TIF 撑爆内存
        CPLSetConfigOption("GDAL_CACHEMAX", "256");

        GDALDataset* poDataset = (GDALDataset*)GDALOpen(tifFilePath.c_str(), GA_ReadOnly);

        if (poDataset == nullptr) {
            LOG_ERROR << "错误：无法打开 TIF 文件 " << tifFilePath;
            return 0;
        }

        int totalWidth = poDataset->GetRasterXSize();
        int totalHeight = poDataset->GetRasterYSize();
        GDALRasterBand* poBand = poDataset->GetRasterBand(1);

        double adfGeoTransform[6];
        if (poDataset->GetGeoTransform(adfGeoTransform) != CE_None) {
            LOG_ERROR << "错误：该 TIF 文件不包含地理坐标变换信息";
            GDALClose(poDataset);
            return 0;
        }

        BaseTile baseTile = getProjectBaseTile();

        std::unordered_set<std::string> batchGridCodes;
        size_t totalGenerated = 0;

        // 批次上限，存够 10000 个就触发回调推入 Redis
        const size_t BATCH_LIMIT = 10000;

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
                if (!readDEMBlockToPoints(poBand, adfGeoTransform, xOff, yOff, curWidth, curHeight, points)) continue;

                auto faces = buildTIN(points, curWidth, curHeight, config.noDataValue);

                // ✨ 立即强制释放散点内存
                std::vector<PointLBHd>().swap(points);

                if (faces.empty()) continue;

                std::vector<std::string> localCodes = triangular_multiple(faces, config.level, baseTile);

                // ✨ 立即强制释放三角网内存
                std::vector<Triangle>().swap(faces);

                for (const auto& code : localCodes) {
                    batchGridCodes.insert(code);
                }

                if (batchGridCodes.size() >= BATCH_LIMIT) {
                    if (batchCallback) {
                        batchCallback(std::vector<std::string>(batchGridCodes.begin(), batchGridCodes.end()));
                    }
                    totalGenerated += batchGridCodes.size();
                    // ✨ 立即强制销毁哈希表，归还内存给操作系统
                    std::unordered_set<std::string>().swap(batchGridCodes);
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

        GDALClose(poDataset);
        LOG_INFO << "TIF 算法提取完毕！累计生成网格: " << totalGenerated << " 个";

        return totalGenerated;
    }
}