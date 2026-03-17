#pragma once
#include <drogon/orm/Mapper.h>
#include <drogon/orm/CoroMapper.h>
#include <drogon/orm/Result.h>

namespace models {

class GridData {
public:
    using PrimaryKeyType = int64_t;
    
    // 表字段定义
    int64_t code;                    // numeric(40,0) PRIMARY KEY
    std::string centerGeometry;       // GEOMETRY(POINTZ, 4326) - 使用字符串存储 WKT 格式
    double maxlon;                   // DOUBLE PRECISION
    double minlon;                   // DOUBLE PRECISION
    double maxlat;                   // DOUBLE PRECISION
    double minlat;                   // DOUBLE PRECISION
    double top;                      // DOUBLE PRECISION
    double bottom;                   // DOUBLE PRECISION
    int64_t x;                      // int8
    int64_t y;                      // int8
    int32_t z;                      // int4
    std::string type;                // VARCHAR(50)

    // 元数据 - 支持动态表名
    static std::string getTableName(int level = -1) {
        if (level >= 0 && level <= 21) {
            return "osgbgrid_" + std::to_string(level);
        }
        return "osgbgrid"; // 默认表名，向后兼容
    }
    
    static const std::string &tableName() {
        static const std::string table_name = "osgbgrid";
        return table_name;
    }
    
    static const std::vector<std::string> &primaryKeyName() {
        static const std::vector<std::string> primary_keys{"code"};
        return primary_keys;
    }
    
    static const std::string &primaryKeyName(size_t index) {
        static const std::vector<std::string> primary_keys{"code"};
        return primary_keys[index];
    }
    
    // 字段映射
    static const std::vector<std::string> &columnName() {
        static const std::vector<std::string> column_names{
            "code", "center", "maxlon", "minlon", "maxlat", "minlat", 
            "top", "bottom", "x", "y", "z", "type"
        };
        return column_names;
    }
};

} // namespace models

// 为 GridData 注册映射
namespace drogon
{
namespace orm
{
template <>
struct Mapper<models::GridData> {
    static std::string getTableName() { return models::GridData::tableName(); }
    static std::string getPrimaryKeyName(size_t index = 0) { 
        return models::GridData::primaryKeyName(index); 
    }
    static std::vector<std::string> getPrimaryKeyNames() { 
        return models::GridData::primaryKeyName(); 
    }
    static std::vector<std::string> getColumnNames() { 
        return models::GridData::columnName(); 
    }
};
} // namespace orm
} // namespace drogon