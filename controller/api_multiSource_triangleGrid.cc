#include "api_multiSource_triangleGrid.h"
#include <drogon/HttpTypes.h>
#include <drogon/drogon.h>
#include <drogon/orm/DbClient.h>
#include <json/json.h>
#include <dqg/DQG3DBasic.h>
#include <dqg/DQG3DPolygon.h>
#include <dqg/DQG3DTil.h>
#include <dqg/Data.h>
#include <dqg/Extractor.h>
#include <dqg/GlobalBaseTile.h>
#include "models/GridData.h"
#include <vector>
#include <unordered_set>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <limits>
#include <array>
#include <future>
#include <thread>
#include <mutex>
#include <atomic>
#include <regex>



using namespace api::multiSource;

/**
 * @brief 插入更新日志到数据库
 *
 * 该函数用于记录OSGB网格化处理过程中的关键事件，便于其他服务监控处理状态。
 *
 * @param dbClient 数据库客户端连接
 * @param moduleCode 模块编码（6表示实景三维数据网格化）
 * @param moduleName 模块名称
 * @param updateContent 更新内容描述
 * @return 是否成功插入日志
 */
bool insertUpdateLog(const std::shared_ptr<drogon::orm::DbClient>& dbClient, 
                     const std::string& moduleCode, 
                     const std::string& moduleName, 
                     const std::string& updateContent) {
    try {
        // 检查并创建 update_log 表（如果不存在）
        try {
            std::string checkTableSql = "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'update_log')";
            auto result = dbClient->execSqlSync(checkTableSql);
            bool tableExists = result[0]["exists"].as<bool>();
            
            if (!tableExists) {
                // 创建 update_log 表
                try {
                    // 先创建序列
                    std::string createSeqSql = "CREATE SEQUENCE IF NOT EXISTS update_log_id_seq "
                                                "START WITH 1 INCREMENT BY 1 NO MINVALUE NO MAXVALUE CACHE 1";
                    dbClient->execSqlSync(createSeqSql);
                    
                    // 创建表
                    std::string createTableSql = 
                        "CREATE TABLE \"public\".\"update_log\" ("
                        "\"id\" int8 NOT NULL DEFAULT nextval('update_log_id_seq'::regclass),"
                        "\"module_code\" varchar(100) COLLATE \"pg_catalog\".\"default\" NOT NULL,"
                        "\"module_name\" varchar(200) COLLATE \"pg_catalog\".\"default\" NOT NULL,"
                        "\"update_content\" text COLLATE \"pg_catalog\".\"default\","
                        "\"create_time\" timestamp(6) NOT NULL DEFAULT CURRENT_TIMESTAMP,"
                        "\"update_time\" timestamp(6) NOT NULL DEFAULT CURRENT_TIMESTAMP"
                        ")";
                    dbClient->execSqlSync(createTableSql);
                    
                    // 创建索引
                    std::string createIndex1Sql = 
                        "CREATE INDEX \"idx_update_log_module_time\" ON \"public\".\"update_log\" USING btree ("
                        "\"module_code\" COLLATE \"pg_catalog\".\"default\" \"pg_catalog\".\"text_ops\" ASC NULLS LAST,"
                        "\"create_time\" \"pg_catalog\".\"timestamp_ops\" DESC NULLS FIRST"
                        ")";
                    dbClient->execSqlSync(createIndex1Sql);
                    
                    std::string createIndex2Sql = 
                        "CREATE INDEX \"idx_update_log_update_time\" ON \"public\".\"update_log\" USING btree ("
                        "\"module_code\" COLLATE \"pg_catalog\".\"default\" \"pg_catalog\".\"text_ops\" ASC NULLS LAST,"
                        "\"update_time\" \"pg_catalog\".\"timestamp_ops\" DESC NULLS FIRST"
                        ")";
                    dbClient->execSqlSync(createIndex2Sql);
                    
                    // 设置主键
                    std::string createPkSql = "ALTER TABLE \"public\".\"update_log\" ADD CONSTRAINT \"update_log_pkey\" PRIMARY KEY (\"id\")";
                    dbClient->execSqlSync(createPkSql);
                    
                    LOG_INFO << "成功创建 update_log 表";
                } catch (const drogon::orm::DrogonDbException &e) {
                    LOG_ERROR << "创建 update_log 表过程中出错: " << e.base().what();
                    // 抛出异常让外层捕获
                    throw;
                }
            }
        } catch (const drogon::orm::DrogonDbException &e) {
            LOG_ERROR << "检查或创建 update_log 表失败: " << e.base().what();
            // 继续尝试插入，可能表已经存在但检查失败
        }
        
        // 插入日志记录
        std::string sql = "INSERT INTO update_log (module_code, module_name, update_content) "
                         "VALUES ($1, $2, $3)";
        dbClient->execSqlSync(sql, moduleCode, moduleName, updateContent);
        LOG_INFO << "成功插入更新日志: " << moduleName << " - " << updateContent;
        return true;
    } catch (const drogon::orm::DrogonDbException &e) {
        LOG_ERROR << "插入更新日志失败: " << e.base().what();
        return false;
    }
}

/**
 * @brief OSGB文件转网格JSON接口
 *
 * 该接口将OSGB格式的三维模型数据转换为网格编码数据，并存储到数据库中。
 * 主要流程：
 * 1. 参数验证和安全检查
 * 2. 从OSGB文件提取三角形数据
 * 3. 坐标系转换（可选）
 * 4. 三角形网格化和编码生成
 * 5. 数据库存储
 * 6. 返回网格信息
 *
 * @note osgbLevel 为可选参数，默认使用20和21级数据进行提取
 * @param req HTTP请求对象，包含JSON参数：osgbFolder（OSGB文件路径）、level（网格层级）、osgbLevel（可选，OSGB文件层级）
 * @param callback HTTP响应回调函数
 */
void triangleGrid::osgbToGridJson(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // ==================== 参数验证阶段 ====================
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查必需参数
        if (!body->isMember("osgbFolder") || !body->isMember("level")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "缺少必需参数: osgbFolder 或 level";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证level参数类型
        if (!(*body)["level"].isInt()) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 参数必须为整数";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 提取并解析参数
        std::string folder = (*body)["osgbFolder"].asString();
        int level = (*body)["level"].asInt();

        // ==================== 安全和范围验证阶段 ====================

        // 路径安全检查：防止路径遍历攻击
        // 检查常见的路径遍历字符，确保访问安全
        if (folder.find("~") != std::string::npos) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "路径包含非法字符";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 路径有效性验证：确保目录存在且可访问
        std::filesystem::path folderPath(folder);
        if (!std::filesystem::exists(folderPath) || !std::filesystem::is_directory(folderPath)) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "指定的目录不存在或不是有效目录";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 参数范围验证：确保层级参数在合理范围内
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // ==================== 数据库初始化与表创建 ====================
        // 提前初始化数据库连接，确保后续分块处理时可用
        auto dbClient = drogon::app().getDbClient("default");
        if (!dbClient) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "无法连接到数据库";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // 记录处理开始日志（在数据库连接建立后）
        std::string startContent = "开始处理OSGB网格化 - 目录: " + folder + ", 网格层级: " + std::to_string(level);
        insertUpdateLog(dbClient, "6", "实景三维数据网格化", startContent);

        // 根据网格层级生成动态表名
        std::string tableName = "osgbgrid_" + std::to_string(level);

        // 创建数据库表（如果不存在）
        try {
            std::string createTableSql = "CREATE TABLE IF NOT EXISTS " + tableName + " ("
                     "code NUMERIC(40,0) PRIMARY KEY, "           // 网格编码
                     "center GEOMETRY(POINTZ, 4326), "             // 中心点3D坐标
                     "maxlon DOUBLE PRECISION, "
                     "minlon DOUBLE PRECISION, "
                     "maxlat DOUBLE PRECISION, "
                     "minlat DOUBLE PRECISION, "
                     "top DOUBLE PRECISION, "
                     "bottom DOUBLE PRECISION, "
                     "x BIGINT, "
                     "y BIGINT, "
                     "z INTEGER, "
                     "type VARCHAR(50)"
                     ")";
            dbClient->execSqlSync(createTableSql);
            LOG_INFO << "成功创建或确认表存在: " << tableName;
        } catch (const drogon::orm::DrogonDbException &e) {
            LOG_ERROR << "创建数据库表失败: " << e.base().what();
            // 继续执行
        }

        // ==================== 分块策略准备 ====================
        // 识别输入目录下的子文件夹，将其作为独立分块进行处理
        // 这样可以避免一次性加载所有数据导致内存溢出
        std::vector<std::string> chunks;
        std::filesystem::path rootPath(folder);

        // 定义目录名称匹配函数，识别常见的OSGB数据块目录命名模式
        // 支持的模式: Block_+000_+000, Tile_+000_+000, ATile_+000_+000
        const auto isValidChunkDir = [](const std::string& dirName) -> bool {
            // 匹配模式: 前缀(字母或字母组合) + 下划线 + +/-数字 + 下划线 + +/-数字
            // 例如: Block_+000_+000, Tile_+001_-002, ATile_+011_+004
            std::regex pattern(R"(^[A-Za-z]+_[+-]\d+_[+-]\d+$)");
            return std::regex_match(dirName, pattern);
        };

        try {
            bool hasSubDirs = false;
            if (std::filesystem::exists(rootPath) && std::filesystem::is_directory(rootPath)) {
                for (const auto& entry : std::filesystem::directory_iterator(rootPath)) {
                    if (entry.is_directory()) {
                        std::string dirName = entry.path().filename().string();
                        // 检查是否是有效的OSGB数据块目录
                        if (isValidChunkDir(dirName)) {
                            chunks.push_back(entry.path().string());
                            hasSubDirs = true;
                            LOG_INFO << "识别到有效分块目录: " << dirName;
                        }
                    }
                }
            }

            // 如果没有子目录，则将根目录作为一个分块处理
            if (!hasSubDirs) {
                chunks.push_back(folder);
                LOG_INFO << "未检测到子目录，将处理根目录";
            }
        } catch (const std::exception& e) {
            LOG_WARN << "扫描子目录时发生错误: " << e.what() << "，将处理根目录";
            chunks.push_back(folder);
        }

        LOG_INFO << "检测到 " << chunks.size() << " 个分块数据待处理";
        cout << "检测到 " << chunks.size() << " 个分块数据待处理" << endl;

        // ==================== 全局统计变量 ====================
        std::atomic<uint64_t> totalTrianglesProcessed{0};
        std::atomic<uint64_t> totalGridCount{0};

        // 获取全局基础瓦片配置
        const BaseTile& baseTile = ::getProjectBaseTile();

        // 定义元数据查找函数
        const auto findMetadataDir = [](std::filesystem::path current) -> std::filesystem::path {
            current = std::filesystem::absolute(current);
            if (!std::filesystem::exists(current)) return {};
            if (std::filesystem::is_regular_file(current)) current = current.parent_path();
            while (!current.empty()) {
                if (std::filesystem::exists(current / "metadata.xml")) return current;
                if (!current.has_parent_path() || current.parent_path() == current) break;
                current = current.parent_path();
            }
            return {};
        };

        // ==================== 顺序处理每个分块 ====================
        // 为了防止内存溢出，现在顺序处理每个分块，确保一个分块完全处理完后再处理下一个
        for (const auto& chunkFolder : chunks) {
            LOG_INFO << "正在顺序处理分块: " << chunkFolder;

            // 记录分块处理开始日志
            std::string chunkStartContent = "开始处理分块: " + chunkFolder + ", 网格层级: " + std::to_string(level);
            insertUpdateLog(dbClient, "6", "实景三维数据网格化", chunkStartContent);

            // 处理当前分块 - 使用独立的变量确保内存隔离
            std::vector<Triangle> triangles;
            {
                // 并行提取两个层级的三角形数据
                auto future20 = std::async(std::launch::async, [&chunkFolder]() {
                    std::vector<Triangle> t;
                    extractTrianglesFromLevelFiles(chunkFolder, 20, t);
                    return t;
                });
                auto future21 = std::async(std::launch::async, [&chunkFolder]() {
                    std::vector<Triangle> t;
                    extractTrianglesFromLevelFiles(chunkFolder, 21, t);
                    return t;
                });
                std::vector<Triangle> t20 = future20.get();
                std::vector<Triangle> t21 = future21.get();
                triangles.reserve(t20.size() + t21.size());
                triangles.insert(triangles.end(), std::make_move_iterator(t20.begin()), std::make_move_iterator(t20.end()));
                triangles.insert(triangles.end(), std::make_move_iterator(t21.begin()), std::make_move_iterator(t21.end()));
            }

            if (triangles.empty()) {
                LOG_INFO << "分块未提取到三角形数据，跳过: " << chunkFolder;
                // 记录分块处理完成日志（跳过的情况）
                std::string chunkEndContent = "分块处理完成（跳过，无数据）: " + chunkFolder;
                insertUpdateLog(dbClient, "6", "实景三维数据网格化", chunkEndContent);
                continue;
            }

            totalTrianglesProcessed.fetch_add(static_cast<uint64_t>(triangles.size()));

            try {
                const std::filesystem::path metadataDir = findMetadataDir(chunkFolder);
                if (!metadataDir.empty()) {
                    convertCoordinatesFromXML(triangles, metadataDir.string(), "EPSG:4326");
                }
            } catch (const std::exception& e) {
                LOG_WARN << "坐标转换失败: " << e.what();
            }

            // 处理三角形到网格的转换
            std::unordered_set<std::string> chunkCodes;
            const uint8_t gridLevelUint = static_cast<uint8_t>(level);
            const unsigned int numThreads = std::max(1u, std::thread::hardware_concurrency() / 2); // 限制线程数以控制内存使用
            const size_t trianglesPerThread = (triangles.size() + numThreads - 1) / numThreads;

            // 并行处理三角形到网格的转换
            std::vector<std::future<std::unordered_set<std::string>>> futures;
            for (unsigned int i = 0; i < numThreads; ++i) {
                size_t startIdx = i * trianglesPerThread;
                size_t endIdx = std::min(startIdx + trianglesPerThread, triangles.size());
                if (startIdx >= triangles.size()) break;

                futures.push_back(std::async(std::launch::async, [&, startIdx, endIdx]() {
                    std::unordered_set<std::string> localCodes;
                    for (size_t idx = startIdx; idx < endIdx; ++idx) {
                        const auto &t = triangles[idx];
                        if (std::isnan(t.vertex1.Lng) || std::isnan(t.vertex1.Lat) || std::isnan(t.vertex1.Hgt) ||
                            std::isnan(t.vertex2.Lng) || std::isnan(t.vertex2.Lat) || std::isnan(t.vertex2.Hgt) ||
                            std::isnan(t.vertex3.Lng) || std::isnan(t.vertex3.Lat) || std::isnan(t.vertex3.Hgt)) continue;
                        try {
                            IJH p1 = localRowColHeiNumber(gridLevelUint, t.vertex1.Lng, t.vertex1.Lat, t.vertex1.Hgt, baseTile);
                            IJH p2 = localRowColHeiNumber(gridLevelUint, t.vertex2.Lng, t.vertex2.Lat, t.vertex2.Hgt, baseTile);
                            IJH p3 = localRowColHeiNumber(gridLevelUint, t.vertex3.Lng, t.vertex3.Lat, t.vertex3.Hgt, baseTile);
                            std::vector<IJH> triangleGrids = triangularGrid(p1, p2, p3, gridLevelUint);
                            for (const auto& grid : triangleGrids) {
                                std::string code = IJH2DQG_str(grid.row, grid.column, grid.layer, gridLevelUint);
                                if (!code.empty()) localCodes.insert(code);
                            }
                        } catch (...) {}
                    }
                    return localCodes;
                }));
            }

            // 收集并合并所有线程的结果
            for (auto& f : futures) {
                auto localCodes = f.get();
                chunkCodes.insert(localCodes.begin(), localCodes.end());
            }

            // 清理三角形数据以释放内存
            std::vector<Triangle>().swap(triangles);

            // 将网格代码转换为数据库记录
            std::vector<models::GridData> gridDataList;
            gridDataList.reserve(chunkCodes.size());
            for (const auto &code : chunkCodes) {
                try {
                    LatLonHei gridInfo = getLocalTileLatLon(code, baseTile);
                    IJH localRCH = getLocalTileRHC(code);
                    if (std::isnan(gridInfo.longitude) || std::isnan(gridInfo.latitude)) continue;
                    models::GridData gridData;
                    if (code.length() > 3 && code.substr(0, 3) == "dqg") {
                        gridData.code = std::stoll(code.substr(3));
                    } else {
                        gridData.code = std::stoll(code);
                    }
                    gridData.centerGeometry = "SRID=4326;POINT(" +
                        std::to_string(gridInfo.longitude) + " " +
                        std::to_string(gridInfo.latitude) + " " +
                        std::to_string(gridInfo.height) + ")";
                    gridData.maxlon = gridInfo.east;
                    gridData.minlon = gridInfo.west;
                    gridData.maxlat = gridInfo.north;
                    gridData.minlat = gridInfo.south;
                    gridData.top = gridInfo.top;
                    gridData.bottom = gridInfo.bottom;
                    gridData.x = localRCH.column;
                    gridData.y = localRCH.row;
                    gridData.z = localRCH.layer;
                    gridData.type = "osgb";
                    gridDataList.push_back(std::move(gridData));
                } catch (...) { continue; }
            }

            totalGridCount.fetch_add(static_cast<uint64_t>(chunkCodes.size()));

            // 记录分块处理完成日志（使用gridDataList.size()而不是triangles.size()，因为triangles已被清理）
            std::string chunkEndContent = "分块处理完成: " + chunkFolder +
                                         ", 网格数量: " + std::to_string(chunkCodes.size());
            insertUpdateLog(dbClient, "6", "实景三维数据网格化", chunkEndContent);

            // 分批入库，避免单次处理过多数据
            int batchSize = 1000;
            for (size_t i = 0; i < gridDataList.size(); i += batchSize) {
                size_t end = std::min(i + batchSize, gridDataList.size());
                std::string sql = "INSERT INTO " + tableName + " (code, center, maxlon, minlon, maxlat, minlat, "
                                 "top, bottom, x, y, z, type) VALUES ";
                for (size_t j = i; j < end; ++j) {
                    const auto &grid = gridDataList[j];
                    sql += "(" + std::to_string(grid.code) + ", " +
                           "ST_GeomFromEWKT('" + grid.centerGeometry + "'), " +
                           std::to_string(grid.maxlon) + ", " + std::to_string(grid.minlon) + ", " +
                           std::to_string(grid.maxlat) + ", " + std::to_string(grid.minlat) + ", " +
                           std::to_string(grid.top) + ", " + std::to_string(grid.bottom) + ", " +
                           std::to_string(grid.x) + ", " + std::to_string(grid.y) + ", " +
                           std::to_string(grid.z) + ", '" + grid.type + "')";
                    if (j < end - 1) sql += ", ";
                }
                sql += " ON CONFLICT (code) DO NOTHING";
                try {
                    dbClient->execSqlSync(sql);
                } catch (const std::exception &e) {
                    LOG_ERROR << "入库失败: " << e.what();
                }
            }

            // 清理当前分块的内存
            std::vector<models::GridData>().swap(gridDataList);
            std::unordered_set<std::string>().swap(chunkCodes);

            LOG_INFO << "分块 " << chunkFolder << " 处理完成并已清理内存";
        }

        // ==================== 响应构建 ====================
        LOG_INFO << "所有分块处理完成，共处理三角形: " << totalTrianglesProcessed
                 << "，处理网格总数: " << totalGridCount;

        // 记录处理完成日志
        std::string completeContent = "OSGB网格化处理完成 - 目录: " + folder +
                                    ", 网格层级: " + std::to_string(level) +
                                    ", 总三角形数: " + std::to_string(totalTrianglesProcessed.load()) +
                                    ", 总网格数: " + std::to_string(totalGridCount.load()) +
                                    ", 目标表: " + tableName;
        insertUpdateLog(dbClient, "6", "实景三维数据网格化", completeContent);

        Json::Value res;
        res["status"] = "success";
        res["data"]["total_triangles_processed"] = static_cast<Json::UInt64>(totalTrianglesProcessed.load());
        res["data"]["total_grid_count"] = static_cast<Json::UInt64>(totalGridCount.load());
        res["data"]["message"] = "处理完成，数据已存入数据库";
        // 移除 cells 字段，避免内存溢出

        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception &e) {
        // ==================== 异常处理阶段 ====================
        // 统一的异常处理，确保API返回标准化的错误响应

        // 尝试记录错误日志
        try {
            auto dbClient = drogon::app().getDbClient("default");
            if (dbClient) {
                std::string errorContent = "OSGB网格化处理异常 - 错误信息: " + std::string(e.what());
                insertUpdateLog(dbClient, "6", "实景三维数据网格化", errorContent);
            }
        } catch (...) {
            LOG_ERROR << "记录错误日志失败";
        }

        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

/**
 * @brief 数据库连接和层级表检查接口
 *
 * 该接口用于测试数据库连接状态，并检查所有OSGB网格层级表的存在情况和数据量。
 * 主要功能：
 * 1. 测试数据库连接是否正常
 * 2. 检查osgbgrid_0到osgbgrid_21所有层级表是否存在
 * 3. 统计每个表的记录数量
 * 4. 返回详细的数据库状态信息
 *
 * @param req HTTP请求对象（无需参数）
 * @param callback HTTP响应回调函数
 */
void triangleGrid::testDatabase(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    Json::Value response;

    try {
        // ==================== 数据库连接测试阶段 ====================

        // 获取数据库客户端连接
        auto dbClient = drogon::app().getDbClient("default");
        if (!dbClient) {
            response["status"] = "error";
            response["message"] = "无法连接到数据库";
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // 执行简单的连接测试查询
        try {
            auto result = dbClient->execSqlSync("SELECT 1 as test_connection");
            response["status"] = "success";
            response["message"] = "数据库连接成功";
        } catch (const drogon::orm::DrogonDbException &e) {
            response["status"] = "error";
            response["message"] = "数据库连接测试失败: " + std::string(e.base().what());
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // ==================== 层级表检查阶段 ====================
        // 检查所有OSGB网格层级表（0-21）的存在情况和数据统计

        try {
            Json::Value tablesInfo(Json::arrayValue); // 存储每个表的详细信息
            int totalRecords = 0;                      // 所有表的总记录数
            bool anyTableExists = false;               // 是否存在任何有效的层级表

            // 遍历所有可能的层级（0-21）
            for (int level = 0; level <= 21; level++) {
                std::string tableName = "osgbgrid_" + std::to_string(level);

                // 查询information_schema检查表是否存在
                auto result = dbClient->execSqlSync("SELECT COUNT(*) as count FROM information_schema.tables WHERE table_name = '" + tableName + "'");

                // 构建当前层级表的信息对象
                Json::Value tableInfo;
                tableInfo["level"] = level;
                tableInfo["table_name"] = tableName;

                if (result.size() > 0 && result[0]["count"].as<int64_t>() > 0) {
                    // 表存在的情况
                    anyTableExists = true;
                    tableInfo["exists"] = true;

                    // 尝试获取该表的记录数量
                    try {
                        auto countResult = dbClient->execSqlSync("SELECT COUNT(*) as record_count FROM " + tableName);
                        int64_t recordCount = countResult[0]["record_count"].as<int64_t>();
                        tableInfo["record_count"] = recordCount;
                        totalRecords += recordCount; // 累加到总数
                    } catch (const drogon::orm::DrogonDbException &e) {
                        // 查询记录数失败时设置-1表示异常
                        tableInfo["record_count"] = -1;
                    }
                } else {
                    // 表不存在的情况
                    tableInfo["exists"] = false;
                    tableInfo["record_count"] = 0;
                }

                // 将表信息添加到结果数组
                tablesInfo.append(tableInfo);
            }

            // 构建最终响应信息
            response["status"] = "success";
            if (anyTableExists) {
                response["message"] = "数据库层级表检查完成，存在 osgbgrid 相关表";
            } else {
                response["message"] = "数据库中不存在任何 osgbgrid 层级表";
            }

            // 设置响应数据
            response["tables_info"] = tablesInfo;          // 每个表的详细信息
            response["total_records"] = totalRecords;      // 所有表的总记录数
            response["any_table_exists"] = anyTableExists; // 是否存在任何有效表

        } catch (const drogon::orm::DrogonDbException &e) {
            response["status"] = "error";
            response["message"] = "检查层级表是否存在失败: " + std::string(e.base().what());
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k500InternalServerError);
            callback(resp);
            return;
        }

        // ==================== 成功响应阶段 ====================

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception &e) {
        // ==================== 统一异常处理阶段 ====================

        Json::Value response;
        response["status"] = "error";
        response["message"] = "测试过程中发生错误: " + std::string(e.what());
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

// 原computePolyhedronGridFill函数已迁移至dqglib的Extractor模块中

/**
 * @brief 三角面片空间网格填充接口（三角面片立体对象网格化）
 *
 * 该接口接受一组三角面片，通过空间采样算法找到与这些三角面片相交的网格单元，
 * 并返回这些网格单元的编码信息。主要用于三维模型的空间网格化处理。
 *
 * 主要流程：
 * 1. 参数验证和输入解析
 * 2. 调用核心计算函数进行网格填充
 * 3. 结果构建和响应返回
 *
 * @param req HTTP请求对象，包含JSON参数：
 *   - faces: 三角面片数组，支持[lon,lat,h]或{longitude,latitude,height}格式
 *   - level: 网格层级（0-21）
 *   - sampleMultiplier: 采样倍率（可选，默认2.0）
 *   - maxSamples: 最大采样数（可选，默认500000）
 * @param callback HTTP响应回调函数
 */
void triangleGrid::fillPolyhedronWithCubes(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // ==================== 1. 参数验证和输入解析 ====================
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 检查必需参数：faces（三角面数组）和level（网格层级）
        if (!body->isMember("faces") || !(*body)["faces"].isArray() || !body->isMember("level")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须包含 'faces' 三角面数组 和整数 'level'";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证level参数类型和范围
        if (!(*body)["level"].isInt()) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 参数必须为整数";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        int level = (*body)["level"].asInt();
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // ==================== 2. 三角面片解析和边界计算 ====================

        // 存储所有三角面片和边界信息
        std::vector<std::array<PointLBHd, 3>> faces;
        const Json::Value &facesJson = (*body)["faces"];

        // 解析输入的面片数据
        for (Json::ArrayIndex fi = 0; fi < facesJson.size(); ++fi) {
            const Json::Value &poly = facesJson[fi];

            // 跳过无效的多边形（至少需要3个顶点）
            if (!poly.isArray() || poly.size() < 3) continue;

            // 收集多边形顶点，支持两种格式：
            // 1. 数组格式：[lon, lat, h]
            // 2. 对象格式：{longitude: lon, latitude: lat, height: h}
            std::vector<PointLBHd> vertices;
            vertices.reserve(poly.size());
            bool validPolygon = true;

            for (Json::ArrayIndex vi = 0; vi < poly.size(); ++vi) {
                const Json::Value &vertex = poly[vi];
                PointLBHd point;

                // 解析数组格式的顶点
                if (vertex.isArray() && vertex.size() >= 3 &&
                    vertex[0].isNumeric() && vertex[1].isNumeric() && vertex[2].isNumeric()) {
                    point.Lng = vertex[(Json::ArrayIndex)0].asDouble();
                    point.Lat = vertex[(Json::ArrayIndex)1].asDouble();
                    point.Hgt = vertex[(Json::ArrayIndex)2].asDouble();
                }
                // 解析对象格式的顶点
                else if (vertex.isObject() && vertex.isMember("longitude") &&
                        vertex.isMember("latitude") && vertex.isMember("height")) {
                    point.Lng = vertex["longitude"].asDouble();
                    point.Lat = vertex["latitude"].asDouble();
                    point.Hgt = vertex["height"].asDouble();
                } else {
                    validPolygon = false;
                    break;
                }
                vertices.push_back(point);
            }

            if (!validPolygon || vertices.size() < 3) continue;

            // 检查多边形是否闭合（首尾顶点相同），如果是则移除重复点
            if (vertices.size() >= 2) {
                const PointLBHd &first = vertices.front();
                const PointLBHd &last = vertices.back();
                if (first.Lng == last.Lng && first.Lat == last.Lat &&
                    first.Hgt == last.Hgt && vertices.size() > 1) {
                    vertices.pop_back();
                }
            }

            if (vertices.size() < 3) continue;

            // 使用扇形三角化算法将多边形分解为三角形
            // 以第一个顶点为基准，依次与相邻的两个顶点形成三角形
            const PointLBHd &v0 = vertices[0];
            for (size_t vi = 1; vi + 1 < vertices.size(); ++vi) {
                std::array<PointLBHd, 3> triangle{{v0, vertices[vi], vertices[vi + 1]}};
                faces.push_back(triangle);
            }
        }

        // 验证是否成功解析到三角形数据
        if (faces.empty()) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "faces 数组无效或为空";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // ==================== 3. 解析可选参数 ====================

        // 解析可选参数：采样倍率和最大采样数
        double sampleMultiplier = 2.0;
        if (body->isMember("sampleMultiplier") && (*body)["sampleMultiplier"].isNumeric()) {
            double value = (*body)["sampleMultiplier"].asDouble();
            if (value > 0) sampleMultiplier = value;
        }

        // 支持无限制网格计算，允许调用者请求更多网格单元
        size_t maxSamples = 0; // 默认无限制
        if (body->isMember("maxSamples") && (*body)["maxSamples"].isNumeric()) {
            double maxVal = (*body)["maxSamples"].asDouble();
            if (maxVal > 0) maxSamples = static_cast<size_t>(maxVal);
        }

        // ==================== 4. 调用核心计算函数 ====================

        // 获取基础网格配置
        const BaseTile &baseTile = ::getProjectBaseTile();

        PolyhedronGridResult result = ::computePolyhedronGridFill(
            faces, baseTile, level, sampleMultiplier, maxSamples);

        // ==================== 5. 结果构建和响应返回 ====================

        // 构建成功响应
        Json::Value response;
        response["status"] = "success";
        response["data"]["count"] = static_cast<Json::UInt>(result.gridCodes.size());
        response["data"]["multiplierUsed"] = result.multiplierUsed;
        response["data"]["actualSamples"] = static_cast<Json::UInt>(result.actualSamples);
        response["data"]["hitMaxSamples"] = result.hitMaxSamples;

        // 构建网格单元详细信息
        Json::Value cells(Json::arrayValue);
        for (const auto &code : result.gridCodes) {
            LatLonHei gridInfo = getLocalTileLatLon(code, baseTile);
            Json::Value cell;

            cell["code"] = code;
            cell["bottom"] = gridInfo.bottom;
            cell["top"] = gridInfo.top;

            Json::Value center(Json::arrayValue);
            center.append(gridInfo.longitude);
            center.append(gridInfo.latitude);
            center.append(gridInfo.height);
            cell["center"] = center;

            cell["minlon"] = gridInfo.west;
            cell["maxlon"] = gridInfo.east;
            cell["minlat"] = gridInfo.south;
            cell["maxlat"] = gridInfo.north;
            cell["level"] = level;

            cells.append(cell);
        }

        response["data"]["cells"] = cells;

        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception &e) {
        // 异常处理
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}



//——————————————————————————以下接口为实验接口，不建议在生产环境中使用————————————————————————————————————————————————————————
/**
 * @brief 四面体网格填充接口（仅限填充三棱锥/四个三角面）
 *
 * @note 该接口将四个顶点构成的四面体转换为四个三角形面，但最终返回重定向提示
 *
 * 处理流程：
 * 1. 参数验证：检查JSON格式和四个顶点数据完整性
 * 2. 四面体几何验证：确保所有顶点包含有效的三维坐标
 * 3. 三角形面构造：将四面体转换为四个三角形面
 * 4. 接口重定向：引导用户使用更通用的多面体接口
 *
 * 四面体到三角形的转换规则：
 * - 面1：(p0, p1, p2) - 底面三角形
 * - 面2：(p0, p1, p3) - 侧面三角形1
 * - 面3：(p0, p2, p3) - 侧面三角形2
 * - 面4：(p1, p2, p3) - 侧面三角形3
 *
 * @param req HTTP请求对象，包含JSON参数：
 *   - points: 4个点的数组，每个点包含 longitude, latitude, height
 *   - level: 网格层级 (0-21)
 *   - sampleMultiplier: 可选，采样倍数
 *   - maxSamples: 可选，最大采样数
 * @param callback HTTP响应回调函数
 */
void triangleGrid::fillTetraWithCubes(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // ==================== 第一阶段：基础参数验证 ====================

        // 验证JSON请求体格式
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value errorResponse;
            errorResponse["status"] = "error";
            errorResponse["message"] = "请求体必须为 JSON";
            auto response = HttpResponse::newHttpJsonResponse(errorResponse);
            response->setStatusCode(k400BadRequest);
            callback(response);
            return;
        }

        // 验证必需参数：四个顶点数组和网格层级
        if (!body->isMember("points") || !(*body)["points"].isArray() ||
            (*body)["points"].size() != 4 || !body->isMember("level")) {
            Json::Value errorResponse;
            errorResponse["status"] = "error";
            errorResponse["message"] = "请求体必须包含四个点的数组 'points' 和整数 'level'";
            auto response = HttpResponse::newHttpJsonResponse(errorResponse);
            response->setStatusCode(k400BadRequest);
            callback(response);
            return;
        }

        // ==================== 第二阶段：顶点数据验证 ====================

        // 验证每个顶点的数据结构完整性
        for (int i = 0; i < 4; ++i) {
            const Json::Value &point = (*body)["points"][(Json::ArrayIndex)i];

            // 检查点是否为对象且包含必需的坐标字段
            if (!point.isObject() || !point.isMember("longitude") ||
                !point.isMember("latitude") || !point.isMember("height")) {
                Json::Value errorResponse;
                errorResponse["status"] = "error";
                errorResponse["message"] = "每个点必须包含 longitude, latitude, height 字段";
                auto response = HttpResponse::newHttpJsonResponse(errorResponse);
                response->setStatusCode(k400BadRequest);
                callback(response);
                return;
            }
        }

        // ==================== 第三阶段：四面体到三角形转换 ====================

        // 提取四个顶点的引用，便于后续操作
        const Json::Value &point0 = (*body)["points"][(Json::ArrayIndex)0];
        const Json::Value &point1 = (*body)["points"][(Json::ArrayIndex)1];
        const Json::Value &point2 = (*body)["points"][(Json::ArrayIndex)2];
        const Json::Value &point3 = (*body)["points"][(Json::ArrayIndex)3];

        // 初始化三角形面数组
        Json::Value facesArray(Json::arrayValue);

        // ==================== 构造面1：(p0, p1, p2) - 底面三角形 ====================
        Json::Value face1(Json::arrayValue);

        // 顶点 p0
        Json::Value vertex0_1(Json::arrayValue);
        vertex0_1.append(point0["longitude"].asDouble());
        vertex0_1.append(point0["latitude"].asDouble());
        vertex0_1.append(point0["height"].asDouble());
        face1.append(vertex0_1);

        // 顶点 p1
        Json::Value vertex1_1(Json::arrayValue);
        vertex1_1.append(point1["longitude"].asDouble());
        vertex1_1.append(point1["latitude"].asDouble());
        vertex1_1.append(point1["height"].asDouble());
        face1.append(vertex1_1);

        // 顶点 p2
        Json::Value vertex2_1(Json::arrayValue);
        vertex2_1.append(point2["longitude"].asDouble());
        vertex2_1.append(point2["latitude"].asDouble());
        vertex2_1.append(point2["height"].asDouble());
        face1.append(vertex2_1);

        facesArray.append(face1);

        // ==================== 构造面2：(p0, p1, p3) - 侧面三角形1 ====================
        Json::Value face2(Json::arrayValue);

        // 顶点 p0 (复用)
        Json::Value vertex0_2(Json::arrayValue);
        vertex0_2.append(point0["longitude"].asDouble());
        vertex0_2.append(point0["latitude"].asDouble());
        vertex0_2.append(point0["height"].asDouble());
        face2.append(vertex0_2);

        // 顶点 p1 (复用)
        Json::Value vertex1_2(Json::arrayValue);
        vertex1_2.append(point1["longitude"].asDouble());
        vertex1_2.append(point1["latitude"].asDouble());
        vertex1_2.append(point1["height"].asDouble());
        face2.append(vertex1_2);

        // 顶点 p3
        Json::Value vertex3_2(Json::arrayValue);
        vertex3_2.append(point3["longitude"].asDouble());
        vertex3_2.append(point3["latitude"].asDouble());
        vertex3_2.append(point3["height"].asDouble());
        face2.append(vertex3_2);

        facesArray.append(face2);

        // ==================== 构造面3：(p0, p2, p3) - 侧面三角形2 ====================
        Json::Value face3(Json::arrayValue);

        // 顶点 p0 (复用)
        Json::Value vertex0_3(Json::arrayValue);
        vertex0_3.append(point0["longitude"].asDouble());
        vertex0_3.append(point0["latitude"].asDouble());
        vertex0_3.append(point0["height"].asDouble());
        face3.append(vertex0_3);

        // 顶点 p2 (复用)
        Json::Value vertex2_3(Json::arrayValue);
        vertex2_3.append(point2["longitude"].asDouble());
        vertex2_3.append(point2["latitude"].asDouble());
        vertex2_3.append(point2["height"].asDouble());
        face3.append(vertex2_3);

        // 顶点 p3 (复用)
        Json::Value vertex3_3(Json::arrayValue);
        vertex3_3.append(point3["longitude"].asDouble());
        vertex3_3.append(point3["latitude"].asDouble());
        vertex3_3.append(point3["height"].asDouble());
        face3.append(vertex3_3);

        facesArray.append(face3);

        // ==================== 构造面4：(p1, p2, p3) - 侧面三角形3 ====================
        Json::Value face4(Json::arrayValue);

        // 顶点 p1 (复用)
        Json::Value vertex1_4(Json::arrayValue);
        vertex1_4.append(point1["longitude"].asDouble());
        vertex1_4.append(point1["latitude"].asDouble());
        vertex1_4.append(point1["height"].asDouble());
        face4.append(vertex1_4);

        // 顶点 p2 (复用)
        Json::Value vertex2_4(Json::arrayValue);
        vertex2_4.append(point2["longitude"].asDouble());
        vertex2_4.append(point2["latitude"].asDouble());
        vertex2_4.append(point2["height"].asDouble());
        face4.append(vertex2_4);

        // 顶点 p3 (复用)
        Json::Value vertex3_4(Json::arrayValue);
        vertex3_4.append(point3["longitude"].asDouble());
        vertex3_4.append(point3["latitude"].asDouble());
        vertex3_4.append(point3["height"].asDouble());
        face4.append(vertex3_4);

        facesArray.append(face4);

        // ==================== 第四阶段：构建多面体参数对象 ====================

        // 创建临时JSON对象，模拟 fillPolyhedronWithCubes 接口的参数格式
        Json::Value polyhedronRequest = Json::Value(Json::objectValue);
        polyhedronRequest["faces"] = facesArray;
        polyhedronRequest["level"] = (*body)["level"];

        // 转发可选参数（如果存在）
        if (body->isMember("sampleMultiplier")) {
            polyhedronRequest["sampleMultiplier"] = (*body)["sampleMultiplier"];
        }
        if (body->isMember("maxSamples")) {
            polyhedronRequest["maxSamples"] = (*body)["maxSamples"];
        }

        // ==================== 第五阶段：接口重定向处理 ====================

        //
        // 注意：由于构建完整的 HttpRequestPtr 对象复杂度较高，
        // 且为了避免代码重复和维护复杂性，
        // 此接口选择直接返回重定向提示，引导用户使用更通用的多面体接口
        //
        // 设计决策：
        // 1. 保持接口向后兼容性
        // 2. 避免复杂的对象构造和代码重复
        // 3. 引导用户使用标准化的多面体接口
        // 4. 简化错误处理和维护负担
        //

        Json::Value redirectResponse;
        redirectResponse["status"] = "error";
        redirectResponse["message"] = "请使用 /api/multiSource/triangleGrid/fillPolyhedronWithCubes 来提交 faces 数据";

        auto httpResponse = HttpResponse::newHttpJsonResponse(redirectResponse);
        httpResponse->setStatusCode(k400BadRequest);
        callback(httpResponse);

    } catch (const std::exception &e) {
        // ==================== 异常处理 ====================
        Json::Value errorResponse;
        errorResponse["status"] = "error";
        errorResponse["message"] = std::string("服务器内部错误: ") + e.what();
        auto response = HttpResponse::newHttpJsonResponse(errorResponse);
        response->setStatusCode(k500InternalServerError);
        callback(response);
    }
}

/**
 * @brief 三角形网格化接口（仅网格化三角面，不进行体填充）
 *
 *
 * 处理流程：
 * 1. 参数验证和边界检查
 * 2. 三角形顶点解析和边界框计算
 * 3. 网格分辨率自适应计算
 * 4. 三角形内部网格单元采样
 * 5. 网格编码生成和结果封装
 *
 * @param req HTTP请求对象，包含JSON参数：
 *   - points: 3个点的数组，每个点包含 longitude, latitude, height
 *   - level: 网格层级 (0-21)
 * @param callback HTTP响应回调函数
 */
void triangleGrid::fillTriangleWithCubes(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // ==================== 第一阶段：参数验证 ====================

        // 验证JSON格式
        auto body = req->getJsonObject();
        if (!body) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须为 JSON";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证必需参数存在性
        if (!body->isMember("points") || !(*body)["points"].isArray() ||
            (*body)["points"].size() != 3 || !body->isMember("level")) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "请求体必须包含三个点的数组 'points' 和整数 'level'";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证level参数类型
        if (!(*body)["level"].isInt()) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 参数必须为整数";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // 验证level参数范围
        int level = (*body)["level"].asInt();
        if (level < 0 || level > 21) {
            Json::Value err;
            err["status"] = "error";
            err["message"] = "level 必须在 0-21 之间";
            auto resp = HttpResponse::newHttpJsonResponse(err);
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }

        // ==================== 第二阶段：核心处理逻辑 ====================

        // 使用异常处理标签简化错误处理流程
        PLAIN_TRY: {
            // 使用 dqglib 标准结构体 PointLBHd 存储三角形顶点
            PointLBHd trianglePoints[3];

            // ==================== 顶点数据解析 ====================
            // 解析并验证三个顶点的坐标数据
            for (int i = 0; i < 3; ++i) {
                const Json::Value &pointJson = (*body)["points"][(Json::ArrayIndex)i];

                // 验证点对象结构完整性
                if (!pointJson.isObject() || !pointJson.isMember("longitude") ||
                    !pointJson.isMember("latitude") || !pointJson.isMember("height")) {
                    Json::Value err;
                    err["status"] = "error";
                    err["message"] = "每个点必须包含 longitude, latitude, height 字段";
                    auto resp = HttpResponse::newHttpJsonResponse(err);
                    resp->setStatusCode(k400BadRequest);
                    callback(resp);
                    return;
                }

                // 提取坐标数据（JSON字段名映射到 PointLBHd 结构体字段）
                trianglePoints[i].Lng = pointJson["longitude"].asDouble();
                trianglePoints[i].Lat = pointJson["latitude"].asDouble();
                trianglePoints[i].Hgt = pointJson["height"].asDouble();
            }

            // ==================== 边界框计算 ====================
            // 计算三角形的地理边界框，用于后续网格采样范围确定
            double minLongitude = std::min({trianglePoints[0].Lng, trianglePoints[1].Lng, trianglePoints[2].Lng});
            double maxLongitude = std::max({trianglePoints[0].Lng, trianglePoints[1].Lng, trianglePoints[2].Lng});
            double minLatitude = std::min({trianglePoints[0].Lat, trianglePoints[1].Lat, trianglePoints[2].Lat});
            double maxLatitude = std::max({trianglePoints[0].Lat, trianglePoints[1].Lat, trianglePoints[2].Lat});

            // ==================== 网格参数初始化 ====================
            // 获取项目基础瓦片配置
            const BaseTile &baseTile = ::getProjectBaseTile();

            // 计算三角形几何中心点（用于自适应网格分辨率计算）
            double centerLongitude = (trianglePoints[0].Lng + trianglePoints[1].Lng + trianglePoints[2].Lng) / 3.0;
            double centerLatitude = (trianglePoints[0].Lat + trianglePoints[1].Lat + trianglePoints[2].Lat) / 3.0;
            double centerHeight = (trianglePoints[0].Hgt + trianglePoints[1].Hgt + trianglePoints[2].Hgt) / 3.0;

            // 获取中心点的网格编码（用于自适应分辨率计算）
            std::string centerCode = getLocalCode(static_cast<uint8_t>(level), centerLongitude, centerLatitude, centerHeight, baseTile);

            // 自适应网格步长计算（基于实际瓦片尺寸）
            double longitudeStep = 0.001, latitudeStep = 0.001; // 默认步长
            if (!centerCode.empty()) {
                LatLonHei tileInfo = getLocalTileLatLon(centerCode, baseTile);
                // 使用实际瓦片尺寸作为步长，添加最小值防止除零错误
                longitudeStep = std::max(1e-9, tileInfo.east - tileInfo.west);
                latitudeStep = std::max(1e-9, tileInfo.north - tileInfo.south);
            }

            // ==================== 几何计算辅助函数 ====================
            // 定义点在三角形内部的判断函数（使用重心坐标法）
            auto isPointInTriangle = [](double testX, double testY, const PointLBHd& a, const PointLBHd& b, const PointLBHd& c) -> bool {
                // 构建向量：v0 = AC, v1 = AB, v2 = AP
                double v0x = c.Lng - a.Lng, v0y = c.Lat - a.Lat;
                double v1x = b.Lng - a.Lng, v1y = b.Lat - a.Lat;
                double v2x = testX - a.Lng, v2y = testY - a.Lat;

                // 计算行列式（判断三角形是否退化）
                double denominator = v0x * v1y - v1x * v0y;

                // 处理退化三角形（三点共线）
                if (std::abs(denominator) < 1e-15) {
                    // 使用边界框判断（近似处理）
                    double minX = std::min({a.Lng, b.Lng, c.Lng});
                    double maxX = std::max({a.Lng, b.Lng, c.Lng});
                    double minY = std::min({a.Lat, b.Lat, c.Lat});
                    double maxY = std::max({a.Lat, b.Lat, c.Lat});
                    return testX >= minX && testX <= maxX && testY >= minY && testY <= maxY;
                }

                // 计算重心坐标 u 和 v
                double inverseDenominator = 1.0 / denominator;
                double u = (v2x * v1y - v1x * v2y) * inverseDenominator;
                double v = (v0x * v2y - v2x * v0y) * inverseDenominator;

                // 判断点是否在三角形内部（包含边界，添加数值误差容限）
                return (u >= -1e-12) && (v >= -1e-12) && (u + v <= 1.0 + 1e-12);
            };

            // ==================== 网格采样和编码生成 ====================
            std::unordered_set<std::string> gridCodes; // 使用集合避免重复编码

            // 遍历边界框内的所有网格单元
            for (double latitude = minLatitude; latitude <= maxLatitude + 1e-12; latitude += latitudeStep) {
                for (double longitude = minLongitude; longitude <= maxLongitude + 1e-12; longitude += longitudeStep) {
                    // 使用网格单元中心点进行采样（提高精度）
                    double sampleLongitude = longitude + longitudeStep * 0.5;
                    double sampleLatitude = latitude + latitudeStep * 0.5;

                    // 检查采样点是否在三角形内部
                    if (!isPointInTriangle(sampleLongitude, sampleLatitude, trianglePoints[0], trianglePoints[1], trianglePoints[2])) {
                        continue;
                    }

                    // 使用三角形平均高度作为采样高度（简化处理）
                    double sampleHeight = (trianglePoints[0].Hgt + trianglePoints[1].Hgt + trianglePoints[2].Hgt) / 3.0;

                    // 生成网格编码
                    std::string gridCode = getLocalCode(static_cast<uint8_t>(level), sampleLongitude, sampleLatitude, sampleHeight, baseTile);
                    if (!gridCode.empty()) {
                        gridCodes.insert(gridCode);
                    }
                }
            }

            // ==================== 结果保障机制 ====================
            // 如果没有找到任何网格编码但中心点有效，则至少返回中心点编码
            if (gridCodes.empty() && !centerCode.empty()) {
                gridCodes.insert(centerCode);
            }

            // ==================== 响应数据构建 ====================
            Json::Value response;
            response["status"] = "success";
            response["data"]["count"] = static_cast<Json::UInt>(gridCodes.size());

            // 构建网格单元详细信息数组
            Json::Value cellsArray(Json::arrayValue);
            for (const auto& code : gridCodes) {
                // 获取网格单元的详细信息
                LatLonHei gridInfo = getLocalTileLatLon(code, baseTile);

                // 构建单个网格单元信息
                Json::Value cellInfo;
                cellInfo["code"] = code;
                cellInfo["bottom"] = gridInfo.bottom;
                cellInfo["top"] = gridInfo.top;

                // 构建中心点坐标数组
                Json::Value centerArray(Json::arrayValue);
                centerArray.append(gridInfo.longitude);
                centerArray.append(gridInfo.latitude);
                centerArray.append(gridInfo.height);
                cellInfo["center"] = centerArray;

                // 添加边界信息
                cellInfo["minlon"] = gridInfo.west;
                cellInfo["maxlon"] = gridInfo.east;
                cellInfo["minlat"] = gridInfo.south;
                cellInfo["maxlat"] = gridInfo.north;
                cellInfo["level"] = static_cast<int>(level);

                cellsArray.append(cellInfo);
            }
            response["data"]["cells"] = cellsArray;

            // 发送成功响应
            auto resp = HttpResponse::newHttpJsonResponse(response);
            resp->setStatusCode(k200OK);
            callback(resp);
        }

    } catch (const std::exception &e) {
        // ==================== 异常处理 ====================
        Json::Value err;
        err["status"] = "error";
        err["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(err);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}