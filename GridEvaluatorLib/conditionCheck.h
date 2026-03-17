#pragma once

#include <drogon/nosql/RedisClient.h>
#include <json/json.h>
#include <dqg/GlobalBaseTile.h>
#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace plancheck {

	// 单个网格冲突信息
	struct ConflictInfo {
		std::string code;
		std::string reason;
		size_t index{0};
	};

	// 网格冲突结果载体
	struct ConflictResult {
		bool pass{true};
		std::vector<ConflictInfo> conflicts; // 所有冲突的网格信息

		// 保持向后兼容的方法
		std::string code; // 废弃，仅保持向后兼容
		std::string reason; // 废弃，仅保持向后兼容
		size_t index{0}; // 废弃，仅保持向后兼容
	};

	/**
	 * @brief 将折线按指定层级转换为网格编码序列。
	 */
	std::vector<std::string> polylineToCodes(const Json::Value &points, int level, const BaseTile &baseTile);

	/**
	 * @brief 针对网格编码序列执行冲突检测 (异步回调版本)。
	 * @param codes 网格编码列表
	 * @param startTimeMsOrSec 起飞时间
	 * @param options 检测配置选项
	 * @param redis Redis客户端智能指针
	 * @param callback 检查完成后的回调函数，参数为 ConflictResult
	 */
	void checkLineConflict(const std::vector<std::string> &codes,
						   double startTimeMsOrSec,
						   const Json::Value &options,
						   const std::shared_ptr<drogon::nosql::RedisClient> &redis,
						   std::function<void(ConflictResult)> callback);

	/**
	 * @brief 针对网格编码序列执行冲突检测，遇到第一个冲突即返回 (异步回调版本)。
	 * @param codes 网格编码列表
	 * @param startTimeMsOrSec 起飞时间
	 * @param options 检测配置选项
	 * @param redis Redis客户端智能指针
	 * @param callback 检查完成后的回调函数，参数为 ConflictResult
	 */
	void checkLineConflictFirst(const std::vector<std::string> &codes,
								 double startTimeMsOrSec,
								 const Json::Value &options,
								 const std::shared_ptr<drogon::nosql::RedisClient> &redis,
								 std::function<void(ConflictResult)> callback);

} // namespace plancheck