
#ifndef TIFF_H
#define TIFF_H

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#include <gdal/gdal_priv.h>
#include <string>
#include <mutex>

class TiffReader {
public:
    // [新增] 定义一个统一的无效高程标识值
    static constexpr float INVALID_ELEVATION = -9999.0f;

    static TiffReader& getInstance();
    bool init(const std::string& filePath);
    float getElevation(double lon, double lat);
    ~TiffReader();

    TiffReader(const TiffReader&) = delete;
    TiffReader& operator=(const TiffReader&) = delete;

private:
    TiffReader() = default;

    GDALDataset* dataset = nullptr;
    GDALRasterBand* rasterBand = nullptr;

    double geoTransform[6] = {0};
    double invGeoTransform[6] = {0};

    int rasterXSize = 0;
    int rasterYSize = 0;
    std::mutex mtx;
};

#endif // TIFF_READER_H
