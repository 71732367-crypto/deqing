#define _HAS_STD_BYTE 0
#include <dqg/DQG3DTil.h>
#include <dqg/Extractor.h>
#include <stdint.h>

// ==================== Drogon / DB 相关头文件 ====================
#include <drogon/drogon.h>
#include <drogon/HttpController.h>
#include <drogon/orm/DbClient.h>
#include "controller/api_multiSource_triangleGrid.h"
// ==================== 标准库头文件 ====================
#include <atomic>
#include <filesystem>
#include <future>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>


// ==================== 本地模型 ====================
#include "dqg/GlobalBaseTile.h"
#include "models/GridData.h"   // models::GridData

using namespace api::multiSource;

// ============================================================
//  辅助函数：自动检测指定目录中 OSGB 文件的两个最高层级
// ============================================================

/**
 * @brief 自动检测指定目录中OSGB文件的两个最高层级
 *
 * 递归扫描目录中的所有.osgb文件，通过文件名中的"_L{level}_"模式提取层级编号，
 * 返回最高的两个层级（降序排列）。这样可以兼容不同精度的OSGB数据源。
 *
 * @param folderPath 要扫描的OSGB数据目录路径
 * @param topN 需要返回的最高层级数量，默认为2
 * @return 包含检测到的最高层级编号的vector（降序），可能包含0、1或2个元素
 *
 * @note 文件名匹配模式示例：
 *       - Tile_xxx_yyy_L20_0.osgb  → 提取层级 20
 *       - Tile_xxx_yyy_L21_00.osgb → 提取层级 21
 *       - Data.osgb（无层级标记）  → 跳过
 */
static std::vector<int> findTopOSGBLevels(const std::string& folderPath, int topN = 2)
{
    std::set<int, std::greater<int>> levels;
    //正则表达式，用于匹配特定层级标记用于匹配（_L（数字）_）
    const std::regex levelPattern(R"(_L(\d+)_)");
//遍历指定文件夹，寻找所有的 .osgb 模型文件，从中提取出它们所属的层级（Level），最后按降序（从大到小）挑选出前 $N$ 个层级返回。
    try {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(folderPath)) {
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ".osgb") continue;
            std::string fileName = entry.path().filename().string();

            std::smatch match;
            if (std::regex_search(fileName, match, levelPattern)) {
                int level = std::stoi(match[1].str());
                levels.insert(level);
            }
        }
    } catch (const std::exception& e) {
        LOG_WARN << "扫描OSGB层级时发生错误: " << e.what();
    }
   // 取出前 topN 个最高层级存入 result 向量中并返回
    std::vector<int> result;
    for (auto it = levels.begin(); it != levels.end() && static_cast<int>(result.size()) < topN; ++it) {
        result.push_back(*it);
    }
    return result;
}


// ============================================================
//  Drogon 接口：OSGB 文件转网格 JSON
// ============================================================

/**
 * @brief OSGB文件转网格JSON接口
 *
 * 该接口将OSGB格式的三维模型数据转换为网格编码数据，并存储到数据库中。
 * 主要流程：
 * 1. 参数验证和安全检查
 * 2. 自动检测OSGB文件中每个分块的最高两个层级并提取三角形数据
 * 3. 坐标系转换（可选）
 * 4. 三角形网格化和编码生成
 * 5. 数据库存储
 * 6. 返回网格信息
 *
 * @note 自动从每个分块目录中检测最高两个OSGB层级（如L20+L21、L8+L9等），兼容不同精度的OSGB数据
 * @param req HTTP请求对象，包含JSON参数：osgbFolder（OSGB文件路径）、level（网格层级）
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
        if (folder.find("..") != std::string::npos || folder.find("~") != std::string::npos) {
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

        // 记录处理开始日志
        std::string startContent = "开始处理OSGB网格化 - 目录: " + folder + ", 网格层级: " + std::to_string(level);
        insertUpdateLog(dbClient, "6", "实景三维数据网格化", startContent);

        // 根据网格层级生成动态表名
        std::string tableName = "osgbgrid_" + std::to_string(level);

        // 创建数据库表（如果不存在）
        try {
            std::string createTableSql = "CREATE TABLE IF NOT EXISTS " + tableName + " ("
                     "code NUMERIC(40,0) PRIMARY KEY, "
                     "center GEOMETRY(POINTZ, 4326), "
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
        std::vector<std::string> chunks;
        std::filesystem::path rootPath(folder);

        // 识别常见的OSGB数据块目录命名模式
        // 支持: Block_+000_+000, Tile_+000_+000, ATile_+000_+000
        const auto isValidChunkDir = [](const std::string& dirName) -> bool {
            std::regex pattern(R"(^[A-Za-z]+_[+-]\d+_[+-]\d+$)");
            return std::regex_match(dirName, pattern);
        };
//遍历指定的根目录，寻找并记录所有符合特定规则的“有效分块（Chunk）子目录
        try {
            bool hasSubDirs = false;
            if (std::filesystem::exists(rootPath) && std::filesystem::is_directory(rootPath)) {
                for (const auto& entry : std::filesystem::directory_iterator(rootPath)) {
                    if (entry.is_directory()) {
                        std::string dirName = entry.path().filename().string();
                        if (isValidChunkDir(dirName)) {
                            chunks.push_back(entry.path().string());
                            hasSubDirs = true;
                            LOG_INFO << "识别到有效分块目录: " << dirName;
                        }
                    }
                }
            }

            // 没有子目录则将根目录作为一个分块
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

        const BaseTile& baseTile = ::getProjectBaseTile();

        // 从给定的路径开始，向上（向父目录）逐级查找，直到找到包含 metadata.xml 文件的文件夹为止
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
        for (const auto& chunkFolder : chunks) {
            LOG_INFO << "正在顺序处理分块: " << chunkFolder;

            std::string chunkStartContent = "开始处理分块: " + chunkFolder + ", 网格层级: " + std::to_string(level);
            insertUpdateLog(dbClient, "6", "实景三维数据网格化", chunkStartContent);

            // ---- 自动检测OSGB层级并提取三角形 ----
            std::vector<int> topLevels = findTopOSGBLevels(chunkFolder, 2);

            if (topLevels.empty()) {
                LOG_WARN << "分块中未找到层级化OSGB文件，跳过: " << chunkFolder;
                insertUpdateLog(dbClient, "6", "实景三维数据网格化",
                    "分块处理完成（跳过，未检测到层级文件）: " + chunkFolder);
                continue;
            }

            std::string levelsInfo;
            for (size_t li = 0; li < topLevels.size(); ++li) {
                if (li > 0) levelsInfo += "+";
                levelsInfo += "L" + std::to_string(topLevels[li]);
            }
            LOG_INFO << "分块检测到最高OSGB层级: " << levelsInfo << "，路径: " << chunkFolder;

            // ---- 并行提取各层级三角形 ----
            //利用电脑的多核 CPU 性能，同时从不同的 OSGB 层级（Level）中提取三角形网格数据，最后汇总在一起。
            std::vector<Triangle> triangles;
            {
                std::vector<std::vector<Triangle>> levelResults(topLevels.size());
                std::vector<std::future<void>> levelFutures;

                for (size_t li = 0; li < topLevels.size(); ++li) {
                    int lvl = topLevels[li];
                    levelFutures.push_back(std::async(std::launch::async,
                        [&chunkFolder, lvl, &levelResults, li]() {
                            extractTrianglesFromLevelFiles(chunkFolder, lvl, levelResults[li]);
                        }));
                }
                for (auto& f : levelFutures) f.get();
         //将多线程进行汇总
                for (auto& lr : levelResults) {
                    triangles.insert(triangles.end(),
                        std::make_move_iterator(lr.begin()),
                        std::make_move_iterator(lr.end()));
                }
            }

            if (triangles.empty()) {
                LOG_INFO << "分块未提取到三角形数据，跳过: " << chunkFolder;
                insertUpdateLog(dbClient, "6", "实景三维数据网格化",
                    "分块处理完成（跳过，无数据）: " + chunkFolder);
                continue;
            }

            totalTrianglesProcessed.fetch_add(static_cast<uint64_t>(triangles.size()));

            // ---- 坐标转换 ----
            try {
                const std::filesystem::path metadataDir = findMetadataDir(chunkFolder);
                if (!metadataDir.empty()) {
                    convertCoordinatesFromXML(triangles, metadataDir.string(), "EPSG:4326");
                }
            } catch (const std::exception& e) {
                LOG_WARN << "坐标转换失败: " << e.what();
            }

            // ---- 并行三角形 → 网格编码 ----
            //把一堆 3D 三角形（Triangles）映射到某种地理网格系统中（ DQG），并获取这些三角形覆盖的所有网格编码
            std::unordered_set<std::string> chunkCodes;
            const uint8_t gridLevelUint = static_cast<uint8_t>(level);

            // 【新增】提前计算当前层级的网格步长，避免在线程内重复计算
            const double LDV = (baseTile.north - baseTile.south) / std::pow(2.0, level);
            const double LOV = (baseTile.east - baseTile.west) / std::pow(2.0, level);
            const double HDV = 78125.0 / std::pow(2.0, level);

            const unsigned int numThreads = std::max(1u, std::thread::hardware_concurrency() / 2);//获取cpu最大线程的1/2
            const size_t trianglesPerThread = (triangles.size() + numThreads - 1) / numThreads;

            std::vector<std::future<std::unordered_set<std::string>>> futures;
            for (unsigned int i = 0; i < numThreads; ++i) {
                size_t startIdx = i * trianglesPerThread;
                size_t endIdx = std::min(startIdx + trianglesPerThread, triangles.size());
                if (startIdx >= triangles.size()) break;

                // 注意这里把 LDV, LOV, HDV 传进了 lambda 表达式
                futures.push_back(std::async(std::launch::async, [&, startIdx, endIdx, LDV, LOV, HDV]() {
                    std::unordered_set<std::string> localCodes;
                    for (size_t idx = startIdx; idx < endIdx; ++idx) {
                        const auto &t = triangles[idx];
                        if (std::isnan(t.vertex1.Lng) || std::isnan(t.vertex1.Lat) || std::isnan(t.vertex1.Hgt) ||
                            std::isnan(t.vertex2.Lng) || std::isnan(t.vertex2.Lat) || std::isnan(t.vertex2.Hgt) ||
                            std::isnan(t.vertex3.Lng) || std::isnan(t.vertex3.Lat) || std::isnan(t.vertex3.Hgt))
                            continue;

                        try {
                            // ==========================================================
                            // 第一步：获取三个顶点的精确浮点网格坐标 (避免过早 floor 取整)
                            // ==========================================================
                            auto getExactCol = [&](double lon) {
                                double exact_col = (lon - baseTile.west) / LOV;
                                if (baseTile.west < -180.0 && lon > 0.0) exact_col = (lon - baseTile.west - 360.0) / LOV;
                                return exact_col;
                            };
                            auto getExactRow = [&](double lat) { return (baseTile.north - lat) / LDV; };
                            auto getExactHgt = [&](double hgt) { return std::max(0.0, (hgt - baseTile.bottom) / HDV); };

                            double c1 = getExactCol(t.vertex1.Lng), r1 = getExactRow(t.vertex1.Lat), h1 = getExactHgt(t.vertex1.Hgt);
                            double c2 = getExactCol(t.vertex2.Lng), r2 = getExactRow(t.vertex2.Lat), h2 = getExactHgt(t.vertex2.Hgt);
                            double c3 = getExactCol(t.vertex3.Lng), r3 = getExactRow(t.vertex3.Lat), h3 = getExactHgt(t.vertex3.Hgt);

                            // ==========================================================
                            // 第二步：使用原有的 triangularGrid 填充三角形内部
                            // ==========================================================
                            IJH p1{static_cast<uint32_t>(std::floor(r1)), static_cast<uint32_t>(std::floor(c1)), static_cast<uint32_t>(std::floor(h1))};
                            IJH p2{static_cast<uint32_t>(std::floor(r2)), static_cast<uint32_t>(std::floor(c2)), static_cast<uint32_t>(std::floor(h2))};
                            IJH p3{static_cast<uint32_t>(std::floor(r3)), static_cast<uint32_t>(std::floor(c3)), static_cast<uint32_t>(std::floor(h3))};

                            std::vector<IJH> triangleGrids = triangularGrid(p1, p2, p3, gridLevelUint);
                            for (const auto& grid : triangleGrids) {
                                std::string code = IJH2DQG_str(grid.row, grid.column, grid.layer, gridLevelUint);
                                if (!code.empty()) localCodes.insert(code);
                            }

                            // ==========================================================
                            // 第三步：3D保守光栅化边界补全（替代原来的整数 Bresenham）
                            // ==========================================================
                            auto conservativeEdge3D = [&](double er1, double ec1, double eh1, double er2, double ec2, double eh2) {
                                // 取最大跨距的 2 倍作为步数，保证采样率足够高，不漏任何一个体素
                                double steps = std::max({std::abs(er2 - er1), std::abs(ec2 - ec1), std::abs(eh2 - eh1)}) * 2.0;
                                if (steps < 1.0) steps = 1.0;

                                for (int s = 0; s <= steps; ++s) {
                                    double t = static_cast<double>(s) / steps;
                                    // 此时再做 floor，就能准确命中边缘碰到的每一个网格
                                    uint32_t r = static_cast<uint32_t>(std::floor(er1 + t * (er2 - er1)));
                                    uint32_t c = static_cast<uint32_t>(std::floor(ec1 + t * (ec2 - ec1)));
                                    uint32_t h = static_cast<uint32_t>(std::floor(eh1 + t * (eh2 - eh1)));

                                    try {
                                        std::string code = IJH2DQG_str(r, c, h, gridLevelUint);
                                        if (!code.empty()) localCodes.insert(code);
                                    } catch (...) {}
                                }
                            };

                            // 补全三条边，彻底解决尖角漏网问题
                            conservativeEdge3D(r1, c1, h1, r2, c2, h2);
                            conservativeEdge3D(r2, c2, h2, r3, c3, h3);
                            conservativeEdge3D(r1, c1, h1, r3, c3, h3);

                        } catch (...) {}
                    }
                    return localCodes;
                }));
            }
            for (auto& f : futures) {
                auto localCodes = f.get();
                chunkCodes.insert(localCodes.begin(), localCodes.end());
            }

            // 释放三角形内存
            std::vector<Triangle>().swap(triangles);

            // ---- 网格编码 → 数据库记录 ----
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

            insertUpdateLog(dbClient, "6", "实景三维数据网格化",
                "分块处理完成: " + chunkFolder + ", 网格数量: " + std::to_string(chunkCodes.size()));

            // ---- 分批入库 ----
            int batchSize = 1000;
            for (size_t i = 0; i < gridDataList.size(); i += batchSize) {
                size_t end = std::min(i + batchSize, gridDataList.size());
                std::string sql = "INSERT INTO " + tableName +
                    " (code, center, maxlon, minlon, maxlat, minlat, top, bottom, x, y, z, type) VALUES ";
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

            // 清理当前分块内存
            std::vector<models::GridData>().swap(gridDataList);
            std::unordered_set<std::string>().swap(chunkCodes);

            LOG_INFO << "分块 " << chunkFolder << " 处理完成并已清理内存";
        }

        // ==================== 响应构建 ====================
        LOG_INFO << "所有分块处理完成，共处理三角形: " << totalTrianglesProcessed
                 << "，处理网格总数: " << totalGridCount;

        insertUpdateLog(dbClient, "6", "实景三维数据网格化",
            "OSGB网格化处理完成 - 目录: " + folder +
            ", 网格层级: " + std::to_string(level) +
            ", 总三角形数: " + std::to_string(totalTrianglesProcessed.load()) +
            ", 总网格数: " + std::to_string(totalGridCount.load()) +
            ", 目标表: " + tableName);

        Json::Value res;
        res["status"] = "success";
        res["data"]["total_triangles_processed"] = static_cast<Json::UInt64>(totalTrianglesProcessed.load());
        res["data"]["total_grid_count"]           = static_cast<Json::UInt64>(totalGridCount.load());
        res["data"]["message"]                    = "处理完成，数据已存入数据库";

        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k200OK);
        callback(resp);

    } catch (const std::exception &e) {
        // ==================== 异常处理阶段 ====================
        try {
            auto dbClient = drogon::app().getDbClient("default");
            if (dbClient) {
                insertUpdateLog(dbClient, "6", "实景三维数据网格化",
                    "OSGB网格化处理异常 - 错误信息: " + std::string(e.what()));
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


// ============================================================
//  原始独立测试入口（保留供离线调试使用）
// ============================================================

// int main()
// {
//     system("chcp 65001");
//     BaseTile baseTile = getBaseTile(
//         { 114.1518, 22.3419 },
//         { 114.1961, 22.3441 },
//         { 114.1526, 22.2933 },
//         { 114.1969, 22.2924 });
//
//     vector<string> Allcodes;
//     string dataFolderPath = "D:\\Data";
//
//     for (const auto& entry : filesystem::directory_iterator(dataFolderPath)) {
//         if (entry.is_directory()) {
//             string subFolderPath = entry.path().string();
//             string osgbFilePath = subFolderPath + "\\" + entry.path().filename().string() + ".osgb";
//
//             osg::ref_ptr<osg::Node> lodNode = osgDB::readNodeFile(osgbFilePath);
//             if (!lodNode.valid()) {
//                 cerr << "Failed to read LOD file: " << osgbFilePath << endl;
//                 continue;
//             }
//
//             vector<Triangle> triangles;
//             extractTriangles(lodNode.get(), triangles);
//             convertCoordinatesFromXML(triangles, subFolderPath, "EPSG:4326");
//             if (triangles.empty()) continue;
//
//             vector<string> codes = triangular_multiple(triangles, LEVEL, baseTile);
//             cout << "Generated codes for " << osgbFilePath << ":\n";
//             for (const auto& code : codes) cout << code << endl;
//
//             cout << "Processed file: " << osgbFilePath << endl;
//             if (codes.empty()) {
//                 cerr << "Warning: No codes generated for file: " << osgbFilePath << endl;
//                 continue;
//             }
//             Allcodes.insert(Allcodes.end(), codes.begin(), codes.end());
//             cout << "========================================\n" << endl;
//         }
//     }
//
//     cout << "\nAll files processed successfully!" << endl;
//     cout << "Total codes generated: " << Allcodes.size() << endl;
//     return 0;
// }