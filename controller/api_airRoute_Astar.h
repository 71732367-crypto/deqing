// === 头文件保护 ===
// 使用 #pragma once 防止头文件被重复包含
// 这是 C++ 中常见的头文件保护机制
#pragma once

// === 头文件引入区域 ===

// 引入 Drogon HTTP 控制器基类
// HttpController 提供了 HTTP 路由和请求处理的基础框架
#include <drogon/HttpController.h>

// 引入 Drogon 协程支持库
// 用于实现异步的 A* 路径规划算法，允许在处理过程中使用 co_await
#include <drogon/utils/coroutine.h>

// 使用 Drogon 命名空间，简化代码
// 这样可以直接使用 HttpRequestPtr、HttpResponsePtr、Task 等类型
using namespace drogon;

// === 命名空间定义 ===
namespace api
{
    // airRoute 命名空间：航空路径规划相关功能
    // 将所有航路规划相关的类和函数组织在一个命名空间中
    namespace airRoute
    {
        // === HTTP 控制器类定义 ===
        // Astar 类继承自 drogon::HttpController
        // 这是一个 RESTful API 控制器，用于处理航路规划请求
        class Astar : public drogon::HttpController<Astar>
        {
        public:
            // === 路由映射表开始 ===
            // METHOD_LIST_BEGIN 和 METHOD_LIST_END 之间的宏定义
            // 用于将 HTTP 方法映射到成员函数
            METHOD_LIST_BEGIN

            // 路由映射：将 POST /AstarPathPlane 请求映射到 AstarPathPlane 方法
            // METHOD_ADD 宏参数说明：
            //   第1个参数: 成员函数名（AstarPathPlane）
            //   第2个参数: 路由路径（/AstarPathPlane）
            //   第3个参数: HTTP 方法（Post）
            //
            // 完整的 API 路径为: http://域名/AstarPathPlane
            // 请求方法: POST
            // 请求体格式: JSON
            METHOD_ADD(Astar::AstarPathPlane, "/AstarPathPlane", Post);

            // === 路由映射表结束 ===
            METHOD_LIST_END

            // === HTTP 请求处理方法声明 ===
            // 方法名称: AstarPathPlane
            // 功能: 处理 A* 航路规划请求
            //
            // 参数说明:
            //   req: HTTP 请求指针（drogon::HttpRequestPtr）
            //        包含请求头、请求体、查询参数等信息
            //   callback: HTTP 响应回调函数
            //             用于异步发送响应给客户端
            //             类型: std::function<void(const drogon::HttpResponsePtr&)>
            //
            // 返回类型: Task<void>
            //   Task 是 Drogon 提供的协程返回类型
            //   返回 Task<void> 表示这是一个协程函数，可以在内部使用 co_await
            //   void 表示不直接返回值，而是通过 callback 发送响应
            //
            // 协程特性:
            //   - 使用 co_await 等待异步操作（如网格检查）
            //   - 避免阻塞线程，提高并发性能
            //   - 适合处理耗时的路径计算任务
            static Task<void> AstarPathPlane(const drogon::HttpRequestPtr req,
                                  std::function<void (const drogon::HttpResponsePtr &)> callback);
        };

        // === A* 算法全局配置 ===
        // 从配置文件加载的 A* 搜索步数上限
        // 用于防止 A* 算法无限循环或搜索时间过长
        extern int g_maxSearchSteps;

        // === 配置初始化函数 ===
        // 功能: 从 config.json 的 custom_config 中加载 A* 算法配置参数
        // 调用时机: 在 main.cc 中服务启动时调用
        // 参数: 无
        // 返回值: 无
        // 注意: 如果配置文件中不存在该参数，使用默认值 100000
        void initializeAstarConfig();
    }
}