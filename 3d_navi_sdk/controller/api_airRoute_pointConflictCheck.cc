// 引入航路冲突检测控制器头文件
#include "api_airRoute_pointConflictCheck.h"
// 引入 Drogon Web 框架核心头文件
#include <drogon/drogon.h>
// 引入 Drogon Redis 客户端头文件
#include <drogon/nosql/RedisClient.h>
// 引入 JSON 解析库头文件
#include <json/json.h>
// 引入三维基础数据结构头文件
#include <dqg/DQG3DBasic.h>
// 引入全球基础瓦片头文件
#include <dqg/GlobalBaseTile.h>
// 引入三维缓冲区头文件
#include <dqg/DQG3DBuffer.h>
// 引入数学函数库
#include <cmath>
// 引入可选值类型
#include <optional>
// 引入字符串流库
#include <sstream>
// 引入无序映射容器
#include <unordered_map>
// 引入无序集合容器
#include <unordered_set>
// 引入动态数组容器
#include <vector>

/*
 * 航路冲突检测控制器
 * - 路由：
 * 1) POST /api/airRoute/conflictCheck/lineGrid
 * 将三维折线按层级进行网格化，并对每个网格执行静态/环境/占用等约束检查（可选依赖 Redis）。
 * 2) POST /api/airRoute/conflictCheck/pointBuffer
 * 对指定点位（经纬高）生成球形缓冲区的网格编码，与传入编码集求交集，判断是否冲突。
 */

// 使用 Drogon 框架的 HTTP 请求指针类型
using drogon::HttpRequestPtr;
// 使用 Drogon 框架的 HTTP 响应类型
using drogon::HttpResponse;
// 使用 Drogon 框架的 HTTP 响应指针类型
using drogon::HttpResponsePtr;
// 使用 HTTP 200 OK 状态码常量
using drogon::k200OK;
// 使用 HTTP 400 Bad Request 状态码常量
using drogon::k400BadRequest;
// 使用 HTTP 500 Internal Server Error 状态码常量
using drogon::k500InternalServerError;
// 使用 Redis 客户端智能指针类型
using drogon::nosql::RedisClient;
// 使用 Redis 结果类型
using drogon::nosql::RedisResult;
// 使用 Drogon 应用实例访问器
using drogon::app;

namespace
{
// --------------------------------------------------------------------------------
// 辅助结构与函数 (保持原有逻辑不变)
// --------------------------------------------------------------------------------

// 约束规则结构体，定义各种约束检查的规则参数
struct ConstraintRule
{
    std::string prefix;  // 约束前缀，用于标识约束类型（如 hl, gd, ad 等）
    bool checkValueMustExist{false};  // 是否检查值必须存在
    bool checkValueNotEmpty{false};  // 是否检查值非空
    std::string valueType{"string"};  // 值类型，支持 string/json/set 等
    std::string jsonPath;  // JSON 路径，用于提取嵌套 JSON 对象中的值
    std::string op;  // 操作符，支持 <= >= < > == != 等
    std::string description;  // 约束描述，用于说明约束的含义
    std::function<Json::Value(const Json::Value &, const std::string &)> extractor;  // 值提取器函数，用于从选项中提取期望值
};

// 约束检查结果结构体，保存单次约束检查的结果
struct ConstraintResult
{
    bool pass{true};  // 是否通过约束检查
    double weight{1.0};  // 权重值
    std::string reason{};  // 未通过时的原因说明
};

// 冲突检查上下文结构体，保存整个检查过程中的共享数据
struct ConflictContext
{
    Json::Value options;  // 检查选项配置
    std::unordered_map<std::string, std::string> prefixLevel;  // 约束前缀与层级的映射关系
    std::shared_ptr<RedisClient> redis;  // Redis 客户端连接
    bool checkDp{false};  // 是否检查无人机占用（dp）
    int level{14};  // 网格层级，默认为 14
    const BaseTile *baseTile{nullptr};  // 基础瓦片指针，用于网格编码计算
};

// 字符串分割函数，将文本按指定分隔符拆分成多个子串
std::vector<std::string> split(const std::string &text, char sep)
{
    std::vector<std::string> parts;  // 存储分割后的子串
    std::stringstream ss(text);  // 创建字符串流
    std::string item;
    while (std::getline(ss, item, sep))  // 逐行按分隔符读取
    {
        parts.push_back(item);  // 将子串添加到结果数组
    }
    return parts;  // 返回分割结果
}

// 从 Redis 获取字符串值，使用 GET 命令
std::optional<std::string> redisGetString(const std::shared_ptr<RedisClient> &redis, const std::string &key)
{
    try
    {
        // 同步执行 Redis GET 命令
        RedisResult res = redis->execCommandSync(
            [](const RedisResult &r) { return r; },  // 返回原始结果的 lambda
            "GET %s",
            key.c_str());  // Redis 键名
        if (res.isNil())  // 如果结果为空
        {
            return std::nullopt;  // 返回空值
        }
        return res.asString();  // 返回字符串结果
    }
    catch (const std::exception &e)  // 捕获异常
    {
        LOG_ERROR << "Redis GET error for key " << key << ": " << e.what();  // 记录错误日志
        return std::nullopt;  // 返回空值
    }
}

// 从 Redis 获取集合，使用 SMEMBERS 命令
std::vector<std::string> redisGetSet(const std::shared_ptr<RedisClient> &redis, const std::string &key)
{
    try
    {
        // 同步执行 Redis SMEMBERS 命令
        RedisResult res = redis->execCommandSync(
            [](const RedisResult &r) { return r; },  // 返回原始结果的 lambda
            "SMEMBERS %s",
            key.c_str());  // Redis 键名
        if (res.isNil())  // 如果结果为空
        {
            return {};  // 返回空数组
        }
        std::vector<std::string> values;  // 存储集合元素
        for (const auto &item : res.asArray())  // 遍历结果数组
        {
            values.emplace_back(item.asString());  // 将元素转为字符串并添加到结果
        }
        return values;  // 返回集合内容
    }
    catch (const std::exception &e)  // 捕获异常
    {
        LOG_ERROR << "Redis SMEMBERS error for key " << key << ": " << e.what();  // 记录错误日志
        return {};  // 返回空数组
    }
}

// 从嵌套 JSON 对象中按路径提取数值（如 "data.windSpeed"）
std::optional<double> nestedNumber(const Json::Value &obj, const std::string &path)
{
    if (!obj.isObject() || path.empty())  // 检查输入有效性
    {
        return std::nullopt;  // 返回空值
    }
    const auto parts = split(path, '.');  // 按点号分割路径
    const Json::Value *cursor = &obj;  // 指向当前 JSON 节点的指针
    for (const auto &p : parts)  // 遍历路径的每一部分
    {
        if (!cursor->isObject() || !cursor->isMember(p))  // 检查节点是否存在
        {
            return std::nullopt;  // 路径不存在，返回空值
        }
        cursor = &((*cursor)[p]);  // 移动指针到下一级节点
    }
    if (cursor->isNumeric())  // 如果最终值是数值类型
    {
        return cursor->asDouble();  // 返回双精度浮点数值
    }
    if (cursor->isString())  // 如果最终值是字符串类型
    {
        try
        {
            return std::stod(cursor->asString());  // 尝试将字符串转为双精度浮点数
        }
        catch (...)
        {
            return std::nullopt;  // 转换失败，返回空值
        }
    }
    return std::nullopt;  // 不支持的类型，返回空值
}

// 解析期望的集合值，支持字符串或数组格式
std::vector<std::string> parseExpectedSet(const Json::Value &expected)
{
    std::vector<std::string> out;  // 存储解析结果
    if (expected.isString())  // 如果输入是字符串类型（逗号分隔）
    {
        const auto parts = split(expected.asString(), ',');  // 按逗号分割
        for (const auto &p : parts)  // 遍历每个部分
        {
            if (p.empty())
                continue;  // 跳过空字符串
            const auto seg = split(p, ':');  // 按冒号分割（格式可能是 value:weight）
            out.push_back(seg.empty() ? p : seg.front());  // 取冒号前的值
        }
    }
    else if (expected.isArray())  // 如果输入是数组类型
    {
        for (const auto &v : expected)  // 遍历数组元素
        {
            if (v.isString())
            {
                const auto seg = split(v.asString(), ':');  // 按冒号分割
                out.push_back(seg.empty() ? v.asString() : seg.front());  // 取冒号前的值
            }
        }
    }
    return out;  // 返回解析结果
}

// 将 JSON 值转换为双精度浮点数
double toDouble(const Json::Value &v)
{
    if (v.isNumeric())  // 如果是数值类型
    {
        return v.asDouble();  // 直接返回双精度浮点数
    }
    if (v.isString())  // 如果是字符串类型
    {
        try
        {
            return std::stod(v.asString());  // 尝试将字符串转为双精度浮点数
        }
        catch (...)
        {
            return std::numeric_limits<double>::quiet_NaN();  // 转换失败，返回 NaN
        }
    }
    return std::numeric_limits<double>::quiet_NaN();  // 不支持的类型，返回 NaN
}

// 获取约束规则映射表，定义所有支持的约束类型及其规则
const std::unordered_map<std::string, std::vector<ConstraintRule>> &ruleMap()
{
    // 静态常量映射表，确保全局唯一
    static const std::unordered_map<std::string, std::vector<ConstraintRule>> map = {
        // hl - 航路校验，必须存在于航路规划中
        {"hl", {{.prefix = "hl", .checkValueMustExist = true, .description = "航路校验：必须存在于航路规划中"}}},
        // gd - 三维实景，存在即冲突
        {"gd", {{.prefix = "gd", .checkValueNotEmpty = true, .description = "三维实景（存在即冲突）"}}},
        // dt - 无人机实时占用校验，存在即冲突
        {"dt", {{.prefix = "dt", .checkValueNotEmpty = true, .description = "无人机实时占用校验（存在即冲突）"}}},
        // dz - 电子围栏，存在即冲突
        {"dz", {{.prefix = "dz", .checkValueNotEmpty = true, .description = "电子围栏（存在即冲突）"}}},
        // za - 障碍物，存在即冲突
        {"za", {{.prefix = "za", .checkValueNotEmpty = true, .description = "障碍物（存在即冲突）"}}},
        // ad - 空域类型占用冲突校验，使用集合包含关系判断
        {"ad",
         {{.prefix = "ad",
           .valueType = "set",  // 值类型为集合
           .op = "containsAny",  // 操作为包含任意一个
           .description = "空域类型占用冲突校验",
           .extractor = [](const Json::Value &opts, const std::string &key) {
               // 从选项中提取期望的空域类型集合
               if (!opts.isMember(key))
               {
                   return Json::Value(Json::arrayValue);  // 不存在则返回空数组
               }
               Json::Value out(Json::arrayValue);
               for (const auto &s : parseExpectedSet(opts[key]))  // 解析期望值集合
               {
                   out.append(s);  // 添加到输出数组
               }
               return out;
           }}}},
        // wd - 气象限制，包括风速和温度
        {"wd",
         {{.prefix = "wd",
           .valueType = "json",  // 值类型为 JSON 对象
           .jsonPath = "windSpeed",  // JSON 路径指向风速字段
           .op = "<=",  // 操作为小于等于
           .description = "风速限制（实际 <= 允许最大值）",
           .extractor = [](const Json::Value &opts, const std::string &key) {
               // 从选项中提取风速限制值
               if (!opts.isMember(key))
                   return Json::Value();  // 不存在则返回空值
               const auto &obj = opts[key];
               return obj.isObject() && obj.isMember("windSpeed") ? obj["windSpeed"] : Json::Value();
           }},
          {.prefix = "wd",
           .valueType = "json",  // 值类型为 JSON 对象
           .jsonPath = "temperature",  // JSON 路径指向温度字段
           .op = "<=",  // 操作为小于等于
           .description = "温度限制（实际 <= 允许最大值）",
           .extractor = [](const Json::Value &opts, const std::string &key) {
               // 从选项中提取温度限制值
               if (!opts.isMember(key))
                   return Json::Value();  // 不存在则返回空值
               const auto &obj = opts[key];
               return obj.isObject() && obj.isMember("temperature") ? obj["temperature"] : Json::Value();
           }}}}
    };
    return map;  // 返回规则映射表
}

// 评估单个约束规则是否通过
bool evaluateConstraint(const ConstraintRule &rule, const Json::Value &expectedRaw, const Json::Value &actualRaw)
{
    // 处理集合类型的约束（如空域类型占用检查）
    if (rule.valueType == "set" && rule.op == "containsAny")
    {
        // 提取实际值集合
        const auto actualVec = [&]() {
            std::vector<std::string> v;  // 存储实际值
            if (actualRaw.isArray())  // 如果实际值是数组
            {
                for (const auto &item : actualRaw)  // 遍历数组元素
                {
                    const auto seg = split(item.asString(), ':');  // 按冒号分割（格式可能是 value:weight）
                    v.push_back(seg.empty() ? item.asString() : seg.front());  // 取冒号前的值
                }
            }
            return v;  // 返回实际值集合
        }();
        const auto expectedVec = parseExpectedSet(expectedRaw);  // 解析期望值集合
        for (const auto &e : expectedVec)  // 遍历期望值
        {
            // 如果期望值存在于实际值集合中，则冲突
            if (std::find(actualVec.begin(), actualVec.end(), e) != actualVec.end())
            {
                return false;  // 发现冲突，返回不通过
            }
        }
        return true;  // 无冲突，返回通过
    }

    double actual = 0.0;  // 实际值
    if (rule.valueType == "json")  // 如果值类型为 JSON 对象
    {
        auto val = nestedNumber(actualRaw, rule.jsonPath);  // 从 JSON 路径提取数值
        if (!val)  // 提取失败
        {
            return false;  // 返回不通过
        }
        actual = *val;  // 赋值实际值
    }
    else  // 其他类型
    {
        actual = toDouble(actualRaw);  // 转换为双精度浮点数
        if (std::isnan(actual))  // 转换失败
        {
            return false;  // 返回不通过
        }
    }

    double expected = toDouble(expectedRaw);  // 转换期望值为双精度浮点数
    if (std::isnan(expected))  // 期望值为 NaN（表示无限制）
    {
        return true;  // 无限制条件，直接通过
    }

    // 根据操作符进行比较
    if (rule.op == "<=")
        return actual <= expected;  // 实际值小于等于期望值
    if (rule.op == ">=")
        return actual >= expected;  // 实际值大于等于期望值
    if (rule.op == "<")
        return actual < expected;  // 实际值小于期望值
    if (rule.op == ">")
        return actual > expected;  // 实际值大于期望值
    if (rule.op == "==")
        return std::fabs(actual - expected) < 1e-9;  // 实际值约等于期望值（浮点数容差比较）
    if (rule.op == "!=")
        return std::fabs(actual - expected) > 1e-9;  // 实际值不等于期望值
    return true;  // 未知操作符，默认通过
}

// 评估网格编码的所有约束条件
ConstraintResult evaluateGridConstraints(const std::string &code, const ConflictContext &ctx)
{
    if (!ctx.redis)  // 如果没有 Redis 连接
    {
        return {.pass = true, .weight = 1.0, .reason = {}};  // 无约束条件，直接通过
    }
    // 定义约束检查的优先级顺序
    static const std::vector<std::string> prefixOrder = {"hl", "gd", "ad", "wd", "dz", "za", "dt"};

    for (const auto &prefix : prefixOrder)  // 按优先级顺序遍历约束前缀
    {
        const auto itLevel = ctx.prefixLevel.find(prefix);  // 查找前缀对应的层级
        if (itLevel == ctx.prefixLevel.end())  // 如果未找到该前缀
        {
            continue;  // 跳过该前缀
        }

        // 获取层级并计算编码切片长度
        const int lv = std::stoi(itLevel->second);  // 字符串转整数层级
        const auto sliceLen = std::min(static_cast<size_t>(lv + 1), code.size());  // 切片长度
        const std::string slice = code.substr(0, sliceLen);  // 从编码开头截取指定长度的子串
        const std::string redisKey = prefix + "_" + slice;  // 构建 Redis 键名
        const std::string optionKey = prefix + "_" + itLevel->second;  // 构建选项键名

        const auto mapIt = ruleMap().find(prefix);  // 查找前缀对应的规则
        if (mapIt == ruleMap().end())  // 如果未找到规则
        {
            continue;  // 跳过该前缀
        }

        for (const auto &rule : mapIt->second)  // 遍历该前缀的所有规则
        {
            // 检查值必须存在
            if (rule.checkValueMustExist)
            {
                auto value = redisGetString(ctx.redis, redisKey);  // 从 Redis 获取值
                if (!value || value->empty())  // 值不存在或为空
                {
                    // 返回不通过，权重设为无穷大
                    return {.pass = false, .weight = std::numeric_limits<double>::infinity(), .reason = rule.description};
                }
                continue;  // 检查通过，继续下一个规则
            }

            // 检查值非空（存在即冲突）
            if (rule.checkValueNotEmpty)
            {
                auto value = redisGetString(ctx.redis, redisKey);  // 从 Redis 获取值
                if (value && !value->empty())  // 值存在且非空
                {
                    // 返回不通过，权重设为无穷大
                    return {.pass = false, .weight = std::numeric_limits<double>::infinity(), .reason = rule.description};
                }
                continue;  // 检查通过，继续下一个规则
            }

            Json::Value actual;  // 实际值
            if (rule.valueType == "json")  // 如果值类型为 JSON 对象
            {
                auto value = redisGetString(ctx.redis, redisKey);  // 从 Redis 获取 JSON 字符串
                if (!value || value->empty())  // 值不存在或为空
                {
                    // 返回不通过，说明无环境数据
                    return {.pass = false, .weight = std::numeric_limits<double>::infinity(), .reason = "Redis中无环境数据"};
                }
                Json::CharReaderBuilder builder;  // JSON 解析器构建器
                std::string errs;  // 错误信息字符串
                std::istringstream iss(*value);  // 字符串输入流
                if (!Json::parseFromStream(builder, iss, &actual, &errs))  // 解析 JSON
                {
                    // 解析失败，返回不通过
                    return {.pass = false, .weight = std::numeric_limits<double>::infinity(), .reason = "环境数据解析失败"};
                }
            }
            else if (rule.valueType == "set")  // 如果值类型为集合
            {
                actual = Json::Value(Json::arrayValue);  // 初始化为 JSON 数组
                for (const auto &item : redisGetSet(ctx.redis, redisKey))  // 从 Redis 获取集合
                {
                    actual.append(item);  // 将集合元素添加到 JSON 数组
                }
            }
            else  // 其他类型（字符串）
            {
                auto value = redisGetString(ctx.redis, redisKey);  // 从 Redis 获取值
                if (value)  // 如果值存在
                {
                    actual = *value;  // 赋值实际值
                }
            }

            Json::Value expected;  // 期望值
            if (rule.extractor)  // 如果有自定义提取器
            {
                expected = rule.extractor(ctx.options, optionKey);  // 使用提取器获取期望值
            }
            else if (ctx.options.isMember(optionKey))  // 如果选项中存在该键
            {
                expected = ctx.options[optionKey];  // 直接从选项中获取期望值
            }

            // 如果期望值为空或无效，跳过该规则检查
            if (expected.isNull() || (expected.isString() && expected.asString().empty()) || (expected.isArray() && expected.empty()))
            {
                continue;  // 跳过该规则
            }

            // 评估约束条件
            if (!evaluateConstraint(rule, expected, actual))  // 如果不通过
            {
                // 返回不通过，权重设为无穷大
                return {.pass = false, .weight = std::numeric_limits<double>::infinity(), .reason = rule.description};
            }
        }
    }

    // 所有约束检查通过
    return {.pass = true, .weight = 1.0, .reason = {}};
}

// 检查指定网格编码在给定时间段内是否存在无人机占用冲突
bool timeConflictInterval(const std::string &code, double startTimeMs, double endTimeMs, const ConflictContext &ctx)
{
    if (!ctx.redis)  // 如果没有 Redis 连接
    {
        return false;  // 无法检查，返回无冲突
    }
    const std::string redisKey = "dp_" + code;  // 构建 Redis 键名（dp 表示无人机占用）
    const auto ranges = redisGetSet(ctx.redis, redisKey);  // 从 Redis 获取所有占用时间段
    for (const auto &r : ranges)  // 遍历每个时间段
    {
        const auto parts = split(r, ':');  // 按冒号分割时间段（格式: start:end）
        if (parts.size() < 2)  // 如果格式不正确
        {
            continue;  // 跳过该时间段
        }
        try
        {
            const double start = std::stod(parts[0]);  // 解析开始时间（毫秒）
            const double end = std::stod(parts[1]);  // 解析结束时间（毫秒）
            // 检查时间区间是否重叠（交集条件）
            if (start <= endTimeMs && end >= startTimeMs)
            {
                return true;  // 发现时间重叠，存在冲突
            }
        }
        catch (...)
        {
            continue;  // 解析失败，跳过该时间段
        }
    }
    return false;  // 无冲突
}

// 根据选项构建前缀与层级的映射关系
std::unordered_map<std::string, std::string> buildPrefixLevelMap(const Json::Value &opts)
{
    std::unordered_map<std::string, std::string> map;  // 存储前缀到层级的映射
    for (const auto &key : opts.getMemberNames())  // 遍历选项的所有键
    {
        const auto pos = key.find('_');  // 查找下划线位置
        if (pos == std::string::npos)  // 如果没有下划线
            continue;  // 跳过该键
        const std::string prefix = key.substr(0, pos);  // 提取前缀（下划线前部分）
        const std::string level = key.substr(pos + 1);  // 提取层级（下划线后部分）
        // 验证前缀和层级非空，且前缀未被记录过
        if (!prefix.empty() && !level.empty() && map.find(prefix) == map.end())
        {
            map.emplace(prefix, level);  // 添加到映射表
        }
    }
    return map;  // 返回映射表
}

// 检查选项中是否包含无人机占用（dp）相关配置
bool hasDpOption(const Json::Value &opts)
{
    for (const auto &key : opts.getMemberNames())  // 遍历选项的所有键
    {
        if (key.rfind("dp_", 0) == 0)  // 如果键以 "dp_" 开头
        {
            return true;  // 发现无人机占用配置
        }
    }
    return false;  // 无无人机占用配置
}

// 根据网格层级获取对应的网格大小（单位：米）
double gridSizeForLevel(int level)
{
    switch (level)
    {
    case 14:
        return 4.5;   // 14 层对应 4.5 米
    case 13:
        return 10.0;  // 13 层对应 10 米
    case 12:
        return 20.0;  // 12 层对应 20 米
    default:
        return 10.0;  // 默认 10 米
    }
}

} // namespace

// --------------------------------------------------------------------------------
// 接口函数实现
// --------------------------------------------------------------------------------

// 点缓冲区交集判断接口（POST /api/airRoute/conflictCheck/pointBuffer）
// 对指定点位（经纬高）生成球形缓冲区的网格编码，与传入编码集求交集，判断是否冲突
void api_airRoute_pointConflictCheck::pointBuffer(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback)
{
    auto respJson = Json::Value(Json::objectValue);  // 初始化响应 JSON 对象

    try
    {
        auto body = req->getJsonObject();  // 获取请求体中的 JSON 对象
        if (!body)  // 如果请求体不是 JSON 格式
        {
            respJson["status"] = "error";  // 设置状态为错误
            respJson["message"] = "请求体必须为JSON";  // 错误信息
            auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
            resp->setStatusCode(k400BadRequest);  // 设置 HTTP 状态码为 400
            callback(resp);  // 执行回调返回响应
            return;  // 结束处理
        }

        // 校验必需字段：经度、纬度、高度、半径、编码集合
        const char* requiredKeys[] = {"lon", "lat", "height", "radius", "codes"};
        for (const char* k : requiredKeys) {  // 遍历所有必需字段
            if (!body->isMember(k)) {  // 如果字段不存在
                respJson["status"] = "error";  // 设置状态为错误
                respJson["message"] = std::string("缺少必需参数: ") + k;  // 错误信息
                auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
                resp->setStatusCode(k400BadRequest);  // 设置 HTTP 状态码为 400
                callback(resp);  // 执行回调返回响应
                return;  // 结束处理
            }
        }

        if (!(*body)["codes"].isArray()) {  // 检查 codes 字段是否为数组
            respJson["status"] = "error";  // 设置状态为错误
            respJson["message"] = "codes 必须为数组";  // 错误信息
            auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
            resp->setStatusCode(k400BadRequest);  // 设置 HTTP 状态码为 400
            callback(resp);  // 执行回调返回响应
            return;  // 结束处理
        }

        // 读取并解析请求参数
        const double lon = (*body)["lon"].asDouble();  // 经度
        const double lat = (*body)["lat"].asDouble();  // 纬度
        const double hgt = (*body)["height"].asDouble();  // 高度
        const double radius = (*body)["radius"].asDouble();  // 半径（米）
        int level = body->isMember("level") ? (*body)["level"].asInt() : 14;  // 网格层级，默认 14

        // 验证参数有效性
        if (!std::isfinite(lon) || !std::isfinite(lat) || !std::isfinite(hgt) || !std::isfinite(radius) || radius <= 0.0) {
            respJson["status"] = "error";  // 设置状态为错误
            respJson["message"] = "lon/lat/height/radius 必须为有效数字，且 radius > 0";  // 错误信息
            auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
            resp->setStatusCode(k400BadRequest);  // 设置 HTTP 状态码为 400
            callback(resp);  // 执行回调返回响应
            return;  // 结束处理
        }
        if (level < 1 || level > 21) {  // 验证层级范围
            respJson["status"] = "error";  // 设置状态为错误
            respJson["message"] = "level 必须在 1..21 范围内";  // 错误信息
            auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
            resp->setStatusCode(k400BadRequest);  // 设置 HTTP 状态码为 400
            callback(resp);  // 执行回调返回响应
            return;  // 结束处理
        }

        const BaseTile& baseTile = ::getProjectBaseTile();  // 获取项目基础瓦片
        std::vector<PointLBHd> pts = { PointLBHd{lon, lat, hgt} };  // 创建三维点位数组

        // 依据点位与半径生成缓冲区覆盖的局部网格编码集合
        const auto buffers = getPointsBuffer(pts, static_cast<uint8_t>(level), radius, baseTile);

        // 收集缓冲区编码
        std::unordered_set<std::string> bufferCodes;  // 缓冲区编码集合
        if (!buffers.empty()) {  // 如果缓冲区非空
            for (const auto& item : buffers.front()) {  // 遍历缓冲区元素
                if (!item.code.empty()) bufferCodes.insert(item.code);  // 添加非空编码到集合
            }
        }

        // 输入编码集合
        std::unordered_set<std::string> inputCodes;  // 输入编码集合
        for (const auto& v : (*body)["codes"]) {  // 遍历请求中的编码
            if (v.isString()) inputCodes.insert(v.asString());  // 添加字符串类型的编码
        }

        // 与输入编码集求交集
        std::vector<std::string> intersect;  // 交集结果数组
        for (const auto& c : inputCodes) {  // 遍历输入编码
            if (bufferCodes.find(c) != bufferCodes.end()) intersect.push_back(c);  // 如果在缓冲区中，添加到交集
        }

        // 构建成功响应
        respJson["status"] = "success";  // 状态为成功
        respJson["data"]["conflict"] = !intersect.empty();  // 是否存在冲突（交集非空）
        respJson["data"]["intersectCount"] = static_cast<Json::UInt64>(intersect.size());  // 交集数量
        Json::Value arr(Json::arrayValue);  // 创建 JSON 数组
        for (const auto& c : intersect) arr.append(c);  // 添加交集编码到数组
        respJson["data"]["intersectCodes"] = arr;  // 交集编码数组
        respJson["data"]["bufferCount"] = static_cast<Json::UInt64>(bufferCodes.size());  // 缓冲区编码总数

        auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
        resp->setStatusCode(k200OK);  // 设置 HTTP 状态码为 200
        callback(resp);  // 执行回调返回响应
    }
    catch (const std::exception &e)  // 捕获异常
    {
        respJson["status"] = "error";  // 设置状态为错误
        respJson["message"] = std::string("服务器内部错误: ") + e.what();  // 错误信息
        auto resp = HttpResponse::newHttpJsonResponse(respJson);  // 创建 JSON 响应
        resp->setStatusCode(k500InternalServerError);  // 设置 HTTP 状态码为 500
        callback(resp);  // 执行回调返回响应
    }
}
