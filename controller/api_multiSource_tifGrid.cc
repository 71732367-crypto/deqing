#include "api_multiSource_tifGrid.h"
#include "api_multiSource_triangleGrid.h"
#include <dqg/TIFtoCode.h>
#include <dqg/DQG3DBasic.h>
#include <thread>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <span>
#include <cstdint>
#include <charconv>
#include <limits>
#include <json/json.h>

using namespace api::multiSource;
using namespace drogon;

namespace {

constexpr const char* demModuleCode = "dem";
constexpr const char* demModuleName = "DEM地形数据网格化";

/// 使用可往返的精度绑定浮点数，避免坐标被 std::to_string 截断到六位小数。
std::string pgNumber(double value) {
    char buffer[64];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
        std::chars_format::general, std::numeric_limits<double>::max_digits10);
    if (result.ec != std::errc{}) {
        throw std::runtime_error("网格坐标转换失败");
    }
    return std::string(buffer, result.ptr);
}

/// 沿用 OSGB 的网格字段，DEM 单独按层级存储，避免混淆数据来源。
void ensureDemGridTable(const drogon::orm::DbClientPtr& dbClient, const std::string& tableName) {
    dbClient->execSqlSync("CREATE TABLE IF NOT EXISTS public." + tableName + " ("
        "code NUMERIC(40,0) PRIMARY KEY, center GEOMETRY(POINTZ, 4326), "
        "maxlon DOUBLE PRECISION, minlon DOUBLE PRECISION, "
        "maxlat DOUBLE PRECISION, minlat DOUBLE PRECISION, "
        "top DOUBLE PRECISION, bottom DOUBLE PRECISION, "
        "x BIGINT, y BIGINT, z INTEGER, type VARCHAR(50))");
    dbClient->execSqlSync("CREATE INDEX IF NOT EXISTS idx_" + tableName +
        "_center ON public." + tableName + " USING GIST (center)");
}

/// 每 100000 条独立入库；13 个等长数值数组通过 UNNEST 展开，网格和日志原子提交。
size_t saveDemGridBatch(const drogon::orm::DbClientPtr& dbClient, const std::string& tableName,
                       std::span<const std::string> batch, const BaseTile& baseTile,
                       const std::string& sourceName, size_t processedCount, size_t previousInserted) {
    const auto startedAt = std::chrono::steady_clock::now();
    // 数组使用 PostgreSQL 文本格式绑定，参数数量固定为 13 个数组加 4 个日志字段。
    // 所有列在同一次循环追加，避免 UNNEST 对长度不等的数组补 NULL。
    std::vector<std::string> parameters(13, "{");
    for (auto& parameter : parameters) {
        parameter.reserve(batch.size() * 24 + 2);
    }
    const auto appendValue = [&parameters](size_t column, const std::string& value) {
        auto& array = parameters[column];
        if (array.size() > 1) array += ',';
        array += value;
    };
    for (const auto& code : batch) {
        const auto grid = getLocalTileLatLon(code, baseTile);
        const auto ijh = getLocalTileRHC(code);
        for (double value : {grid.longitude, grid.latitude, grid.height, grid.east,
                             grid.west, grid.north, grid.south, grid.top, grid.bottom}) {
            if (!std::isfinite(value)) {
                throw std::runtime_error("网格解码得到无效坐标或边界");
            }
        }
        // 保留编码字符串，21 位编码可能超出十进制 int64 的范围。
        appendValue(0, code);
        size_t column = 1;
        for (double value : {grid.longitude, grid.latitude, grid.height, grid.east,
                             grid.west, grid.north, grid.south, grid.top, grid.bottom}) {
            appendValue(column++, pgNumber(value));
        }
        appendValue(10, std::to_string(ijh.column));
        appendValue(11, std::to_string(ijh.row));
        appendValue(12, std::to_string(ijh.layer));
    }
    for (auto& parameter : parameters) parameter += '}';
    const std::string content = "TIFF PostgreSQL批次完成 - 文件: " + sourceName +
        ", 目标表: " + tableName + ", 本批网格: " + std::to_string(batch.size()) +
        ", 累计处理网格: " + std::to_string(processedCount + batch.size());
    const std::string sql = "WITH inserted AS (INSERT INTO public." + tableName +
        " (code, center, maxlon, minlon, maxlat, minlat, top, bottom, x, y, z, type) "
        "SELECT code, ST_SetSRID(ST_MakePoint(longitude, latitude, height), 4326), "
        "maxlon, minlon, maxlat, minlat, top, bottom, x, y, z, 'dem' "
        "FROM UNNEST($1::numeric[], $2::double precision[], $3::double precision[], "
        "$4::double precision[], $5::double precision[], $6::double precision[], "
        "$7::double precision[], $8::double precision[], $9::double precision[], "
        "$10::double precision[], $11::bigint[], $12::bigint[], $13::integer[]) "
        "AS grids(code, longitude, latitude, height, maxlon, minlon, maxlat, minlat, "
        "top, bottom, x, y, z) WHERE true "
        "ON CONFLICT (code) DO NOTHING RETURNING code), "
        "stats AS (SELECT count(*) AS inserted_count FROM inserted), "
        "logged AS (INSERT INTO public.update_log "
        "(module_code, module_name, update_content, create_time, update_time) "
        "SELECT $14, $15, $16 || ', 本批新增: ' || inserted_count::text || "
        "', 累计新增: ' || ($17::bigint + inserted_count)::text, "
        "CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Shanghai', "
        "CURRENT_TIMESTAMP AT TIME ZONE 'Asia/Shanghai' FROM stats RETURNING id) "
        "SELECT inserted_count, logged.id AS log_id FROM stats CROSS JOIN logged";
    parameters.emplace_back(demModuleCode);
    parameters.emplace_back(demModuleName);
    parameters.push_back(content);
    parameters.push_back(std::to_string(previousInserted));
    const auto sqlStartedAt = std::chrono::steady_clock::now();
    try {
        // 数据修改 CTE 保证任一步失败时整条 SQL 回滚，不再手动发起 ROLLBACK。
        // 日志表已在任务开始时初始化，批次内无需重复查询 information_schema。
        drogon::orm::Result result(nullptr);
        auto binder = *dbClient << sql;
        for (const auto& parameter : parameters) {
            binder << parameter;
        }
        binder << drogon::orm::Mode::Blocking;
        binder >> [&result](const drogon::orm::Result& value) { result = value; };
        binder.exec();
        if (result.size() != 1) {
            throw std::runtime_error("PostgreSQL 未返回完整的网格与日志写入结果");
        }
        // pipeline 的行结果可能早于隐式提交；下一条查询等待上一批同步后核验日志。
        // 网格和日志由同一条 SQL 写入，能读到该日志才确认该批已经提交。
        const auto committed = dbClient->execSqlSync(
            "SELECT id FROM public.update_log WHERE id = $1",
            result[0]["log_id"].as<int64_t>());
        if (committed.size() != 1) {
            throw std::runtime_error("PostgreSQL 未确认本批次网格与日志已提交，累计数量未增加");
        }
        const size_t inserted = result[0]["inserted_count"].as<size_t>();
        const auto finishedAt = std::chrono::steady_clock::now();
        LOG_INFO << "TIFF PG批次已提交: 网格=" << batch.size() << ", 新增=" << inserted
                 << ", 累计处理=" << processedCount + batch.size()
                 << ", 累计新增=" << previousInserted + inserted
                 << ", 准备耗时ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(sqlStartedAt - startedAt).count()
                 << ", SQL耗时ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(finishedAt - sqlStartedAt).count();
        return inserted;
    } catch (const drogon::orm::DrogonDbException& e) {
        throw std::runtime_error("TIFF PG批次SQL失败，已提交网格=" + std::to_string(processedCount) +
            ", 当前批次=" + std::to_string(batch.size()) + ", 错误: " + e.base().what());
    }
}

}

void tifGrid::processTif(const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) const {
    auto jsonBody = req->getJsonObject();
    if (!jsonBody || !jsonBody->isObject()) {
        Json::Value error;
        error["status"] = "error";
        error["message"] = "请求体必须为 JSON 对象";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    if (!jsonBody->isMember("tifPath") || !jsonBody->isMember("level")) {
        Json::Value error;
        error["status"] = "error";
        error["message"] = "缺少必需参数: tifPath 或 level";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    if (!(*jsonBody)["tifPath"].isString() || (*jsonBody)["tifPath"].asString().empty() ||
        !(*jsonBody)["level"].isInt() ||
        (jsonBody->isMember("noDataValue") &&
         (!(*jsonBody)["noDataValue"].isNumeric() ||
          !std::isfinite((*jsonBody)["noDataValue"].asDouble())))) {
        Json::Value error;
        error["status"] = "error";
        error["message"] = "tifPath 必须为非空字符串，level 必须为整数，noDataValue 必须为有限数值";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    std::string tifPath = (*jsonBody)["tifPath"].asString();
    const std::string sourceName = std::filesystem::path(tifPath).filename().string();
    TIFGridProcessor::TIFGridConfig config;
    config.level = (*jsonBody)["level"].asInt();

    // 当前局部 Morton 编码每层占 3 位，使用 64 位存储，非空编码支持 1..21 层。
    if (config.level < 1 || config.level > 21) {
        Json::Value error;
        error["status"] = "error";
        error["message"] = "TIFF 局部网格 level 必须在 1-21 之间";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    const std::filesystem::path inputPath(tifPath);
    std::error_code pathError;
    if (tifPath.find('~') != std::string::npos ||
        std::any_of(inputPath.begin(), inputPath.end(), [](const auto& part) { return part == ".."; }) ||
        !std::filesystem::is_regular_file(inputPath, pathError)) {
        Json::Value error;
        error["status"] = "error";
        error["message"] = "tifPath 必须指向可访问的普通文件，且不能包含 ~ 或 .. 路径段";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    if (jsonBody->isMember("noDataValue")) {
        config.noDataValue = (*jsonBody)["noDataValue"].asDouble();
    }

    // 保留响应回调，计算、PostgreSQL 写入及日志在工作线程完成后再响应。
    auto responseCallback = std::move(callback);
    try {
        std::thread([tifPath, sourceName, config, responseCallback]() {
            Json::Value ret;
            auto statusCode = k200OK;
            drogon::orm::DbClientPtr dbClient;
            size_t pgInsertedCount = 0;
            size_t processedCount = 0;
            const std::string tableName = "demgrid_" + std::to_string(config.level);
            try {
                const auto defaultClient = drogon::app().getDbClient("default");
                if (!defaultClient) {
                    throw std::runtime_error("PostgreSQL 客户端未初始化");
                }
                // TIFF 使用单连接客户端，避免长时间入库占用 A* 的共享连接池。
                dbClient = drogon::orm::DbClient::newPgClient(defaultClient->connectionInfo(), 1);
                dbClient->setTimeout(150.0);
                // 先由 PostgreSQL 取消慢 SQL/锁等待，客户端保留接收取消结果的时间。
                dbClient->execSqlSync("SELECT set_config('statement_timeout', '120s', false), "
                                     "set_config('lock_timeout', '5s', false)");
                if (!insertUpdateLog(dbClient, demModuleCode, demModuleName,
                        "开始处理TIFF网格化 - 文件: " + tifPath +
                        ", 网格层级: " + std::to_string(config.level) + ", 目标表: " + tableName)) {
                    throw std::runtime_error("写入 TIFF 开始日志失败");
                }
                ensureDemGridTable(dbClient, tableName);
                const BaseTile baseTile = ::getProjectBaseTile();
                // 计算一批就提交一批，避免将全部网格留在内存中。
                auto batchSaver = [sourceName, dbClient, tableName, baseTile,
                                   &pgInsertedCount, &processedCount](const std::vector<std::string>& batch) {
                    if (batch.empty()) return;

                    constexpr size_t batchSize = 100000;
                    // 同层编码排序后写入，减少主键和空间索引的随机更新。
                    auto sortedCodes = batch;
                    std::sort(sortedCodes.begin(), sortedCodes.end());
                    const std::span<const std::string> codes(sortedCodes);
                    for (size_t start = 0; start < codes.size(); start += batchSize) {
                        const auto part = codes.subspan(start, std::min(batchSize, codes.size() - start));
                        pgInsertedCount += saveDemGridBatch(dbClient, tableName, part, baseTile,
                            sourceName, processedCount, pgInsertedCount);
                        processedCount += part.size();
                    }
                };

                // 开始计算并触发回调
                size_t totalCount = TIFGridProcessor::convertTIFtoGridCodes(tifPath, config, batchSaver);
                if (!insertUpdateLog(dbClient, demModuleCode, demModuleName,
                        "TIFF网格化处理完成 - 文件: " + tifPath + ", 目标表: " + tableName +
                        ", 累计生成网格: " + std::to_string(totalCount) +
                        ", PostgreSQL新增: " + std::to_string(pgInsertedCount))) {
                    throw std::runtime_error("写入 TIFF 完成日志失败");
                }
                LOG_INFO << "TIF 网格化和 PostgreSQL 写入全部完成，累计生成: " << totalCount
                         << "，PostgreSQL新增: " << pgInsertedCount;
                ret["status"] = "success";
                ret["message"] = "网格化完成，已写入 PostgreSQL 和更新日志";
                ret["totalCount"] = Json::UInt64(totalCount);
                ret["pgInsertedCount"] = Json::UInt64(pgInsertedCount);
                ret["pgTableName"] = tableName;
                ret["level"] = config.level;
            } catch (const std::exception& e) {
                LOG_ERROR << "TIF 网格化处理异常: " << e.what();
                if (dbClient) {
                    try {
                        insertUpdateLog(dbClient, demModuleCode, demModuleName,
                            "TIFF网格化处理异常 - 文件: " + tifPath + ", 目标表: " + tableName +
                            ", PostgreSQL已新增: " + std::to_string(pgInsertedCount) + ", 错误: " + e.what());
                    } catch (const std::exception& logError) {
                        LOG_ERROR << "记录 TIFF 异常日志失败: " << logError.what();
                    }
                }
                statusCode = k500InternalServerError;
                ret["status"] = "error";
                ret["message"] = e.what();
            }
            auto resp = HttpResponse::newHttpJsonResponse(ret);
            resp->setStatusCode(statusCode);
            resp->addHeader("Access-Control-Allow-Origin", "*");
            responseCallback(resp);
        }).detach();
    } catch (const std::exception& e) {
        LOG_ERROR << "无法启动 TIF 网格化工作线程: " << e.what();
        Json::Value error;
        error["status"] = "error";
        error["message"] = "无法启动网格化任务";
        auto resp = HttpResponse::newHttpJsonResponse(error);
        resp->setStatusCode(k500InternalServerError);
        responseCallback(resp);
    }
}
