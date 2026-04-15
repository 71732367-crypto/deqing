#include <dqg/GlobalBaseTile.h>
#include <dqg/DQG3DBasic.h>
#include <stdexcept>
#include <fstream>
#include <json/json.h>
#include<sstream>
#include<iomanip>
#include<cstdint>

/// @brief 全局基础瓦片变量定义
BaseTile projectBaseTile = {};

/// @brief 标记是否已初始化
static bool isInitialized = false;


///@brief 当前研究区域的互操作标识（用于局部编码跨系统交换）
static std::string g_regionId;
///@brief FNV-1a 64位哈希，生成稳定且轻量的区域指纹
static uint64_t fnv1a64(const std::string& input)
{
    uint64_t hash=14695981039346656037ull;
    for (unsigned char ch: input)
    {
        hash ^=static_cast<uint64_t>(ch);
        hash *=1099511628211ull;
    }
    return hash;
}

static const char kBase62Chars[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

static std::string toBase62_6(uint32_t value)
{
    char buf[7];
    buf[6] = '\0';
    for (int i = 5; i >= 0; --i)
    {
        buf[i] = kBase62Chars[value % 62];
        value /= 62;
    }
    return std::string(buf);
}
///@brief 根据BaseTile生成区域互操作标识
///@detials 将区域边界按固定精度序列化后哈希，保证同一区域稳定一致
static std::string buildRegionId(const BaseTile& tile)
{
    std::ostringstream normalized;
    normalized << std::fixed << std::setprecision(8)
               << tile.west   << "|" << tile.south << "|"
               << tile.east   << "|" << tile.north << "|"
               << tile.bottom << "|" << tile.top;

    uint64_t hash   = fnv1a64(normalized.str());
    uint32_t hash32 = static_cast<uint32_t>(hash ^ (hash >> 32));
    return toBase62_6(hash32);
}
/// @brief 初始化全局基础瓦片数据
void initializeProjectBaseTile(const PointLBd& A, const PointLBd& B, const PointLBd& C, const PointLBd& D) {
    try {
        // 调用 getBaseTile 函数计算基础网格数据
        projectBaseTile = getBaseTile(A, B, C, D);
        isInitialized = true;
        //初始化后立即刷新区域标识，供互操作编码使用
        g_regionId=buildRegionId(projectBaseTile);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("[initializeProjectBaseTile] Failed to initialize base tile: ") + e.what());
    }
}

/// @brief 从JSON配置文件初始化全局基础瓦片数据
/// @param configFilePath JSON配置文件路径
/// @return 是否成功初始化
bool initializeProjectBaseTileFromConfig(const std::string& configFilePath) {
    try {
        // 读取JSON配置文件
        std::ifstream configFile(configFilePath);
        if (!configFile.is_open()) {
            throw std::runtime_error("[initializeProjectBaseTileFromConfig] Cannot open config file: " + configFilePath);
        }

        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(configFile, root)) {
            throw std::runtime_error("[initializeProjectBaseTileFromConfig] Failed to parse JSON config file: " + reader.getFormattedErrorMessages());
        }

        // 检查region字段是否存在
        if (!root.isMember("region") || !root["region"].isMember("bounds")) {
            throw std::runtime_error("[initializeProjectBaseTileFromConfig] Invalid config file format: missing 'region.bounds' field");
        }

        const Json::Value& bounds = root["region"]["bounds"];

        // 检查所有必需的角点是否存在
        if (!bounds.isMember("southwest") || !bounds.isMember("northwest") ||
            !bounds.isMember("northeast") || !bounds.isMember("southeast")) {
            throw std::runtime_error("[initializeProjectBaseTileFromConfig] Invalid config file format: missing corner points in bounds");
        }

        // 提取四个角点坐标
        PointLBd A = { // 西南角
            bounds["southwest"]["longitude"].asDouble(),
            bounds["southwest"]["latitude"].asDouble()
        };
        PointLBd B = { // 西北角
            bounds["northwest"]["longitude"].asDouble(),
            bounds["northwest"]["latitude"].asDouble()
        };
        PointLBd C = { // 东北角
            bounds["northeast"]["longitude"].asDouble(),
            bounds["northeast"]["latitude"].asDouble()
        };
        PointLBd D = { // 东南角
            bounds["southeast"]["longitude"].asDouble(),
            bounds["southeast"]["latitude"].asDouble()
        };

        // 调用原有的初始化函数
        initializeProjectBaseTile(A, B, C, D);
        if (root["region"].isMember("height"))
        {
            const Json::Value& height = root["region"]["height"];
            if (height.isMember("bottom"))
            {
                projectBaseTile.bottom = height["bottom"].asDouble();
            }
            if (height.isMember("top"))
            {
                projectBaseTile.top = height["top"].asDouble();
            }
        }
        //配置覆盖高度后，重新计算区域标识，确保跨系统标识一致
        g_regionId=buildRegionId(projectBaseTile);
        return true;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

/// @brief 获取全局基础瓦片数据
const BaseTile& getProjectBaseTile() {
    if (!isInitialized) {
        throw std::runtime_error("[getProjectBaseTile] projectBaseTile has not been initialized. Call initializeProjectBaseTile first.");
    }
    return projectBaseTile;
}

///@brief 获取当前研究取的互操作标识
std::string getProjectRegionId()
{
    if (!isInitialized)
    {
        throw std::runtime_error("[getProjectRegionId] projectBaseTile has not been initialized. Call initializeProjectBaseTile first.");
    }
    return g_regionId;
}