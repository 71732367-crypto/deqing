#ifndef DATA_UTILS_HPP
#define DATA_UTILS_HPP



/// @brief 3D 计算的默认高度
constexpr double HEIGHT = 10000000.0;

/// @brief 2D 网格的最大层级
constexpr int kMaxLevel2D = 30;
constexpr int kMaxLevel = 30;

/// @brief 无符号整数的绝对差值
#define UINT_SUBSTRACT(x, y) ((x) > (y) ? ((x) - (y)) : ((y) - (x)))

/// @brief 获取 64 位整数的八个高位（octant）
#define GET_OCT(x) ((x) >> 61)

/// @brief 移除 64 位整数的八个高位
#define REMOVE_OCT(x) ((x) & (0x1FFFFFFFFFFFFFFF))

/// @brief 设置 64 位整数的八个高位
#define SET_OCT(x, oct) (REMOVE_OCT(x) | ((uint64_t)(oct) << 61))

/// @brief 在给定层级下重定位 Morton 编码
#define MORTON_RELEVEL(x, y) (REMOVE_OCT(x) & ((__ONE__ << (2 * (y))) - 1))

/// @brief 在给定层级下编码（重定位） Morton 编码
#define CODE_RELEVEL(x, y) ((x & 0xE000000000000000) | MORTON_RELEVEL(x, y))

/// @brief 误差限制（用于浮点数比较）
constexpr double ERR_LIM = 1e-9;

/// @brief 将 Morton 编码向左移动到指定层级
#define MOVE_MORTON_TO_LEFT(morton, level) ((morton) << (64 - (level) * 2))

/// @brief 生成高位的 2^k
#define TOP_2K_BIT_ON(k) (((0xFFFFFFFFFFFFFFFF) >> (64 - (k) * 2)) << (64 - (k) * 2))

/// @brief 比较两个浮点数是否相等，使用误差限制
#define DOUBLE_EQUAL(x, y) ((x) > (y) ? ((x) - (y) < ERR_LIM) : ((y) - (x) < ERR_LIM))

/// @brief 计算两个值的最大值
#define MAX(x, y) ((x) > (y) ? (x) : (y))

/// @brief 计算一个值的绝对值
#define ABS(x) ((x) > 0 ? (x) : -(x))

#endif // DATA_UTILS_HPP

