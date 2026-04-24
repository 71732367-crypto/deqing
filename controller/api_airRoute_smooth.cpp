#include "api_airRoute_smooth.h"
#include "GridEvaluator.h"
#include "LineToGrids.h"
#include <dqg/DQG3DBasic.h>
#include <dqg/GlobalBaseTile.h>
#include <coroutine>
#include <unordered_set>
namespace api
{
namespace airRoute
{
    const vector<array<int, 3>> DIRECTIONS = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
        {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
        {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
        {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
        {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
        {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
    };
    // 【新增】：将协程适配器补充到这里，让当前文件也能认识 co_await GridCheckAwaiter
    struct GridCheckAwaiter {
    std::shared_ptr<GridEvaluator> evaluator;
    const std::vector<CandidateInfo>& candidates;
    std::shared_ptr<std::unordered_map<std::string, GridEvaluator::CheckResult>> result;

    bool await_ready() { return false; }
    void await_suspend(std::coroutine_handle<> h) {
        evaluator->checkCandidates(candidates, [this, h](const auto& res) mutable {
            this->result = std::make_shared<std::unordered_map<std::string, GridEvaluator::CheckResult>>(res);
            h.resume();
        });
    }
    auto await_resume() { return result; }
};

// 建议使用带缓冲区的版本，确保不撞墙
Task<std::vector<std::string>> performStringPulling(
    const std::vector<std::string>& rawPath,
    std::shared_ptr<GridEvaluator> evaluator,
    int level,
    const BaseTile& baseTile
) {
    if (rawPath.size() <= 2) co_return rawPath;
    std::vector<std::string> smoothed = {rawPath.front()};
    int current = 0;

    while (current < rawPath.size() - 1) {
        int bestNext = current + 1;
        for (int target = rawPath.size() - 1; target > current + 1; --target) {
            // 获取直线穿过的网格
            LatLonHei s = getLocalTileLatLon(rawPath[current], baseTile);
            LatLonHei e = getLocalTileLatLon(rawPath[target], baseTile);
            std::vector<std::string> coreGrids = singleLineToGrids2({{s.longitude, s.latitude, s.height}, {e.longitude, e.latitude, e.height}}, level, baseTile);

            // 检查直线网格及其 26 邻居（缓冲区）
            std::unordered_set<std::string> checkSet;
            for (auto& code : coreGrids) {
                checkSet.insert(code);
                IJH ijh = getLocalTileRHC(code);
                for (auto& d : DIRECTIONS) { // 需确保 DIRECTIONS 可见
                    checkSet.insert(rchToCode({ijh.row + d[1], ijh.column + d[0], ijh.layer + d[2]}, level));
                }
            }

            std::vector<CandidateInfo> cands;
            for (auto& c : checkSet) cands.push_back({c, 0, 0, "", false});
            auto results = co_await GridCheckAwaiter{evaluator, cands};

            bool collision = false;
            for (auto& c : checkSet) {
                if (results->count(c) && !(*results)[c].pass) { collision = true; break; }
            }
            if (!collision) { bestNext = target; break; }
        }
        smoothed.push_back(rawPath[bestNext]);
        current = bestNext;
    }
    co_return smoothed;
}
Task<void> Smooth::process(const HttpRequestPtr req,
                          std::function<void (const HttpResponsePtr &)> callback)
{
    auto json = req->getJsonObject();
    if (!json || !json->isMember("path")) {
        auto res = HttpResponse::newHttpJsonResponse(Json::Value());
        res->setStatusCode(k400BadRequest);
        callback(res); co_return;
    }

    try {
        int level = (*json).get("level", 14).asInt();
        std::vector<std::string> rawPath;
        for (auto& code : (*json)["path"]) rawPath.push_back(code.asString());

        Json::Value options = json->get("condition", Json::Value(Json::objectValue));
        auto evaluator = GridEvaluator::create(options);
        const auto& baseTile = getProjectBaseTile();

        // 执行拉线优化
        auto smoothedCodes = co_await performStringPulling(rawPath, evaluator, level, baseTile);

        // 封装返回结果
        Json::Value result;
        result["status"] = "success";
        for (auto& code : smoothedCodes) {
            LatLonHei llh = getLocalTileLatLon(code, baseTile);
            Json::Value node;
            node["code"] = code;
            node["center"] = Json::Value(Json::arrayValue);
            node["center"].append(llh.longitude);
            node["center"].append(llh.latitude);
            node["center"].append(llh.height);
            result["data"].append(node);
        }
        callback(HttpResponse::newHttpJsonResponse(result));
    } catch (const std::exception& e) {
        callback(HttpResponse::newCustomHttpResponse(Json::Value(e.what())));
    }
    co_return;
}

}
}
