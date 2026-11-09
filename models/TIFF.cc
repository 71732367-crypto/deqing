
#include "TIFF.h"
#include <drogon/drogon.h>
#include <cmath>
#include <gdal/ogr_spatialref.h>
//单实例，避免重复读取庞大的 TIFF 文件浪费内存。
TiffReader& TiffReader::getInstance() {
    static TiffReader instance;
    return instance;
}

bool TiffReader::init(const std::string& filePath) {
    std::lock_guard<std::mutex> lock(mtx);
    if (dataset != nullptr) return true; // 避免重复初始化

    // 注册 GDAL 所有驱动
    GDALAllRegister();

    // 以只读模式打开 TIFF
    dataset = (GDALDataset*)GDALOpen(filePath.c_str(), GA_ReadOnly);
    if (!dataset) {
        LOG_ERROR << "GDAL 无法打开目标 TIFF 文件: " << filePath;
        return false;
    }

    // 获取地理仿射变换参数
    if (dataset->GetGeoTransform(geoTransform) != CE_None) {
        LOG_ERROR << "无法从 TIFF 中提取 GeoTransform 变换参数";
        GDALClose((GDALDatasetH)dataset);
        dataset = nullptr;
        return false;
    }

    // 计算逆向仿射变换矩阵（将地理坐标转换为行列号）
    if (!GDALInvGeoTransform(geoTransform, invGeoTransform)) {
        LOG_ERROR << "GeoTransform 矩阵求逆失败，无法进行地理坐标到像素的转换";
        GDALClose((GDALDatasetH)dataset);
        dataset = nullptr;
        return false;
    }

    // 仅转换水平位置：CGCS2000 经纬度 -> TIFF 源 CRS，不转换高程。
    const OGRSpatialReference* sourceSrs = dataset->GetSpatialRef();
    OGRSpatialReference inputSrs;
    if (sourceSrs && inputSrs.importFromEPSG(4490) == OGRERR_NONE) {
        OGRSpatialReference rasterSrs(*sourceSrs);
        if (rasterSrs.StripVertical() == OGRERR_NONE) {
            inputSrs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
            rasterSrs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
            coordinateTransform = OGRCreateCoordinateTransformation(&inputSrs, &rasterSrs);
        }
    }
    if (!coordinateTransform) {
        LOG_ERROR << "无法创建 CGCS2000 到 TIFF 源坐标系的转换";
        GDALClose((GDALDatasetH)dataset);
        dataset = nullptr;
        return false;
    }

    // 默认读取第 1 个波段（高程波段）
    rasterBand = dataset->GetRasterBand(1);
    rasterXSize = dataset->GetRasterXSize();
    rasterYSize = dataset->GetRasterYSize();

    LOG_INFO << "成功加载高程 TIFF 文件。分辨率: " << rasterXSize << "x" << rasterYSize;
    return true;
}

float TiffReader::getElevation(double lon, double lat) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!rasterBand) return INVALID_ELEVATION; // [修改] 未初始化时返回无效值
    if (!coordinateTransform->Transform(1, &lon, &lat) ||
        !std::isfinite(lon) || !std::isfinite(lat)) return INVALID_ELEVATION;

    double pixel_d = invGeoTransform[0] + invGeoTransform[1] * lon + invGeoTransform[2] * lat;
    double line_d = invGeoTransform[3] + invGeoTransform[4] * lon + invGeoTransform[5] * lat;

    int pixel = static_cast<int>(std::floor(pixel_d));
    int line = static_cast<int>(std::floor(line_d));

    // 边界条件安全检查
    if (pixel < 0 || pixel >= rasterXSize || line < 0 || line >= rasterYSize) {
        return INVALID_ELEVATION; // [修改] 超出边界时返回无效高程
    }

    float pixelValue = 0.0f;
    CPLErr err = rasterBand->RasterIO(GF_Read, pixel, line, 1, 1, &pixelValue, 1, 1, GDT_Float32, 0, 0);
    if (err != CE_None) {
        return INVALID_ELEVATION; // [修改] 读取失败返回无效高程
    }

    // 处理无效值 (NoData) 校验
    int hasNoData = 0;
    double noDataVal = rasterBand->GetNoDataValue(&hasNoData);
    if (hasNoData && std::abs(pixelValue - static_cast<float>(noDataVal)) < 1e-4) {
        return INVALID_ELEVATION; // [修改] 是 NoData 区域时返回无效高程
    }

    return pixelValue;
}

TiffReader::~TiffReader() {
    if (coordinateTransform) {
        OGRCoordinateTransformation::DestroyCT(coordinateTransform);
    }
    if (dataset) {
        GDALClose((GDALDatasetH)dataset);
    }
}
