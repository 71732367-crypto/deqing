#pragma once

#include <string>
#include <vector>
#include <array>
#include <unordered_set>
#include <functional>

#include <dqg/Data.h>
#include <dqg/Extractor.h>
#include <dqg/GlobalBaseTile.h>

// ==========================================
// 【只需新增这一行】前置声明，告诉编译器这是一个类
class GDALRasterBand;
// ==========================================

namespace TIFGridProcessor {

    struct TIFGridConfig {
        int level;
        double sample;
        size_t MaxSamples;
        double noDataValue;
        TIFGridConfig(): level(11),sample(2.0),MaxSamples(0),noDataValue(-9999.0) {};
    };

    size_t convertTIFtoGridCodes(
        const std::string& tifFilePath,
        const TIFGridProcessor::TIFGridConfig& config,
        std::function<void(const std::vector<std::string>&)> batchCallback = nullptr
    );

    bool readDEMBlockToPoints(GDALRasterBand* poBand, double* adfGeoTransform, int xOff, int yOff, int xSize, int ySize, std::vector<PointLBHd>& outPoints);
    std::vector<Triangle> buildTIN(const std::vector<PointLBHd>& points, int width, int height, double noDataValue);

}