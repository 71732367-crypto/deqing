#include<iostream>
#include<vector>
#include<string>
#include<cmath>
#include <dqg/DQG3DBasic.h>
#include <dqg/GlobalBaseTile.h>
#include <dqg/DQG2D.h>
#include<json/json.h>
#include "api_multiSource_topology.h"
 using namespace api::multiSource;
//====================================算法层=================================================

///计算同层级邻居网格函数
//判断请求是否正确
void sendBadRequest(const std::function<void(const HttpResponsePtr&)>& callback, const std::string& message)
{
 Json::Value response;
 response["status"] = "error";
 response["message"] = message;
 auto resp = HttpResponse::newHttpJsonResponse(response);
 resp->setStatusCode(k400BadRequest);
 callback(resp);
}

//判断是否为有效网格编码
bool validateOctalCode(const std::string& code, std::string& errorMsg)
{
 if (code.empty()){
  errorMsg="网格编码不能为空";
  return false;
 }
 if (code.length() <1|| code.length() > 21){
  errorMsg="无效编码长度";
  return false;
 }
 if (code.find_first_not_of("01234567") != std::string::npos) {
  errorMsg = "网格编码只能包含 0-7";
  return false;
 }
return true;
}

Json::Value calculateNeighbors(const std::string& gridCode,int connectivity){
 const uint8_t level=static_cast<uint8_t>(gridCode.length());
 IJH currentIJH= ::getLocalTileRHC(gridCode);
 const int64_t curRow = currentIJH.row;       // Y 轴 (行，纬度)
 const int64_t curCol = currentIJH.column;    // X 轴 (列，经度)
 const int64_t curLayer = currentIJH.layer;   // Z 轴 (层，高程)

 //  计算当前层级单轴的最大合法索引: 2^level - 1
 const int64_t maxCoord=(level>63)? INT64_MAX:((1LL << level) - 1);
 Json::Value neighborsArray(Json::arrayValue);
 // 遍历三维 26 个方向偏移 (dx: 列/经度, dy: 行/纬度, dz: 层/高程)
 for (int dz=-1; dz<=1; dz++)
 {
  for (int dy=-1; dy<=1; dy++)
  {
   for (int dx=-1; dx<=1; dx++)
   {
    if (dz==0&&dy==0&&dx==0)continue;
    double dist=std::abs(dz)+std::abs(dy)+std::abs(dx);
    if (connectivity == 6 && dist > 1) continue;   // 6连通: 仅面邻接 (dist == 1)
    if (connectivity == 18 && dist > 2) continue;  // 18连通: 面+棱邻接 (dist <= 2)
    std::string relation = (dist == 1) ? "face" : ((dist == 2) ? "edge" : "vertex");
    // 计算邻居坐标
    int64_t nCol = curCol + dx;
    int64_t nRow = curRow + dy;
    int64_t nLayer = curLayer + dz;
    // 边界保护：超出局部基础瓦片空间边界的邻居直接丢弃
    if (nCol < 0 || nCol > maxCoord ||
        nRow < 0 || nRow > maxCoord ||
        nLayer < 0 || nLayer > maxCoord) {
     continue;
        }
    IJH nIJH{
     static_cast<uint32_t>(nRow),
     static_cast<uint32_t>(nCol),
     static_cast<uint32_t>(nLayer)
 };
    std::string neighborCode = ::rchToCode(nIJH, level);
    // 组装单个邻居对象
    Json::Value item;
    item["code"] = neighborCode;
    item["relation"] = relation;
    neighborsArray.append(item);

   }
  }
 }
return neighborsArray;

}


///判断两个网格的空间关系函数
struct SpatialRelationResult {
    std::string relation;  // "equal", "contains", "within", "face", "edge", "vertex", "disjoint"
    bool isAdjacent;       // 是否为外接触邻接 (face / edge / vertex 为 true)
};

// 辅助结构：网格在细层级上的三维整数闭区间包围盒
struct VoxelIntervalBox {
    int64_t minR, maxR;
    int64_t minC, maxC;
    int64_t minH, maxH;
};
// 将任意层级的局部八进制网格投影映射到指定精细层级 targetLevel 的整数闭区间
static VoxelIntervalBox getVoxelBoxAtLevel(const std::string& code, int targetLevel) {
    IJH ijh = ::getLocalTileRHC(code);
    int curLevel = static_cast<int>(code.length());
    int levelDiff = targetLevel - curLevel;
    int64_t scale = 1LL << levelDiff;
    VoxelIntervalBox box;
    box.minR = static_cast<int64_t>(ijh.row) * scale;
    box.maxR = (static_cast<int64_t>(ijh.row) + 1) * scale - 1;
    box.minC = static_cast<int64_t>(ijh.column) * scale;
    box.maxC = (static_cast<int64_t>(ijh.column) + 1) * scale - 1;
    box.minH = static_cast<int64_t>(ijh.layer) * scale;
    box.maxH = (static_cast<int64_t>(ijh.layer) + 1) * scale - 1;
    return box;
}
// 计算两闭区间的一维外距离 (0 表示重叠相交，1 表示紧贴相接，>1 表示有空隙相离)
static int64_t getIntervalDistance(int64_t minA, int64_t maxA, int64_t minB, int64_t maxB) {
    if (maxA < minB) return minB - maxA;
    if (maxB < minA) return minA - maxB;
    return 0; // 重叠
}
// 核心计算：计算两个网格的空间拓扑关系（支持不同层级）
SpatialRelationResult calculateSpatialRelation(const std::string& codeA, const std::string& codeB) {
    const size_t lenA = codeA.length();
    const size_t lenB = codeB.length();
    // 1. 完全相同
    if (lenA == lenB && codeA == codeB) {
        return {"equal", false};
    }
    // 2. 八叉树父子包含关系（利用前缀包含判定）
    if (lenA < lenB && codeB.rfind(codeA, 0) == 0) {
        return {"contains", false}; // A 包含 B
    }
    if (lenA > lenB && codeA.rfind(codeB, 0) == 0) {
        return {"within", false};   // A 被 B 包含
    }
    // 3. 外接触 / 邻接 / 相离 判定（将两网格投影到更细的 max(lenA, lenB) 层级闭区间）
    int targetLevel = std::max(static_cast<int>(lenA), static_cast<int>(lenB));
    VoxelIntervalBox boxA = getVoxelBoxAtLevel(codeA, targetLevel);
    VoxelIntervalBox boxB = getVoxelBoxAtLevel(codeB, targetLevel);
    int64_t dR = getIntervalDistance(boxA.minR, boxA.maxR, boxB.minR, boxB.maxR);
    int64_t dC = getIntervalDistance(boxA.minC, boxA.maxC, boxB.minC, boxB.maxC);
    int64_t dH = getIntervalDistance(boxA.minH, boxA.maxH, boxB.minH, boxB.maxH);
    int64_t maxDist = std::max({dR, dC, dH});
    // 最大轴间距 > 1，代表在三维空间中相离
    if (maxDist > 1) {
        return {"disjoint", false};
    }
    // 最大轴间距 == 1，代表外表面接触，统计恰好间距为 1 的轴个数
    if (maxDist == 1) {
        int touchAxes = (dR == 1 ? 1 : 0) + (dC == 1 ? 1 : 0) + (dH == 1 ? 1 : 0);
        if (touchAxes == 1) {
            return {"face", true};   // 1个轴间距为1，其余轴重叠 -> 面邻接
        } else if (touchAxes == 2) {
            return {"edge", true};   // 2个轴间距为1，1个轴重叠 -> 棱邻接
        } else {
            return {"vertex", true}; // 3个轴间距均为1 -> 顶点邻接
        }
    }
    return {"disjoint", false};
}

///计算多个网格公共父网格
std::string calculateCommonAncestor(const std::vector<std::string>& codes) {
    if (codes.empty()) return "";
    std::string commonPrefix = codes[0];
    for (size_t i = 1; i < codes.size(); ++i) {
        size_t minLen = std::min(commonPrefix.length(), codes[i].length());
        size_t matchLen = 0;
        while (matchLen < minLen && commonPrefix[matchLen] == codes[i][matchLen]) {
            ++matchLen;
        }
        commonPrefix = commonPrefix.substr(0, matchLen);
        if (commonPrefix.empty()) break; // 若已无公共前缀，提前熔断
    }
    return commonPrefix;
}
//===================================接口层=========================================

void topology::getNeighbors(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 1. 请求体 JSON 校验
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            sendBadRequest(callback, "请求体必须是有效的JSON格式");
            return;
        }
        // 2. 校验参数 gridCode
        if (!jsonBody->isMember("gridCode") || !(*jsonBody)["gridCode"].isString()) {
            sendBadRequest(callback, "缺少必需参数 gridCode，且必须为字符串");
            return;
        }
        std::string gridCode = (*jsonBody)["gridCode"].asString();
        std::string errorMsg;
        if (!validateOctalCode(gridCode, errorMsg)) {
            sendBadRequest(callback, errorMsg);
            return;
        }
        // 3. 解析连通度 connectivity (默认 26)
        int connectivity = 26;
        if (jsonBody->isMember("connectivity")) {
            if (!(*jsonBody)["connectivity"].isInt()) {
                sendBadRequest(callback, "connectivity 必须为整数");
                return;
            }
            connectivity = (*jsonBody)["connectivity"].asInt();
            if (connectivity != 6 && connectivity != 18 && connectivity != 26) {
                sendBadRequest(callback, "connectivity 仅支持 6、18 或 26");
                return;
            }
        }
        // 4. 计算邻居
        Json::Value neighbors = calculateNeighbors(gridCode, connectivity);
        // 5. 构造成功响应
        Json::Value response;
        response["status"] = "success";
        response["data"]["gridCode"] = gridCode;
        response["data"]["level"] = static_cast<int>(gridCode.length());
        response["data"]["connectivity"] = connectivity;
        response["data"]["count"] = neighbors.size();
        response["data"]["neighbors"] = neighbors;
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);
    } catch (const std::exception& e) {
        Json::Value res;
        res["status"] = "error";
        res["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

void topology::getRelation(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 1. JSON 请求体校验
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            sendBadRequest(callback, "请求体必须是有效的JSON格式");
            return;
        }
        // 2. 校验参数 codeA 和 codeB
        if (!jsonBody->isMember("codeA") || !(*jsonBody)["codeA"].isString() ||
            !jsonBody->isMember("codeB") || !(*jsonBody)["codeB"].isString()) {
            sendBadRequest(callback, "缺少必需参数 codeA 与 codeB，且必须为字符串");
            return;
        }
        std::string codeA = (*jsonBody)["codeA"].asString();
        std::string codeB = (*jsonBody)["codeB"].asString();
        std::string errorMsg;
        if (!validateOctalCode(codeA, errorMsg)) {
            sendBadRequest(callback, "codeA 错误: " + errorMsg);
            return;
        }
        if (!validateOctalCode(codeB, errorMsg)) {
            sendBadRequest(callback, "codeB 错误: " + errorMsg);
            return;
        }
        // 3. 计算两网格空间关系
        SpatialRelationResult result = calculateSpatialRelation(codeA, codeB);
        // 4. 组装响应数据
        Json::Value response;
        response["status"] = "success";
        response["data"]["codeA"] = codeA;
        response["data"]["codeB"] = codeB;
        response["data"]["levelA"] = static_cast<int>(codeA.length());
        response["data"]["levelB"] = static_cast<int>(codeB.length());
        response["data"]["relation"] = result.relation;
        response["data"]["isAdjacent"] = result.isAdjacent;
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);
    } catch (const std::exception& e) {
        Json::Value res;
        res["status"] = "error";
        res["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}

void topology::getCommonAncestor(const HttpRequestPtr& req, std::function<void (const HttpResponsePtr &)> &&callback) const
{
    try {
        // 1. JSON 请求体校验
        auto jsonBody = req->getJsonObject();
        if (!jsonBody) {
            sendBadRequest(callback, "请求体必须是有效的JSON格式");
            return;
        }
        // 2. 校验参数 gridCodes
        if (!jsonBody->isMember("gridCodes") || !(*jsonBody)["gridCodes"].isArray() || (*jsonBody)["gridCodes"].empty()) {
            sendBadRequest(callback, "缺少必需参数 gridCodes，且必须为非空字符串数组");
            return;
        }
        const auto& arr = (*jsonBody)["gridCodes"];
        if (arr.size() < 2) {
            sendBadRequest(callback, "gridCodes 至少需要包含 2 个网格编码");
            return;
        }
        // 3. 提取并逐一校验网格编码
        std::vector<std::string> codes;
        codes.reserve(arr.size());
        for (Json::ArrayIndex i = 0; i < arr.size(); ++i) {
            if (!arr[i].isString()) {
                sendBadRequest(callback, "gridCodes 数组元素必须全部为字符串");
                return;
            }
            std::string code = arr[i].asString();
            std::string errorMsg;
            if (!validateOctalCode(code, errorMsg)) {
                sendBadRequest(callback, "索引 [" + std::to_string(i) + "] 编码错误: " + errorMsg);
                return;
            }
            codes.push_back(code);
        }
        // 4. 调用算法层计算最小包围祖先网格
        std::string ancestorCode = calculateCommonAncestor(codes);
        // 5. 组装响应数据
        Json::Value response;
        response["status"] = "success";
        response["data"]["ancestorCode"] = ancestorCode;
        response["data"]["level"] = static_cast<int>(ancestorCode.length());
        response["data"]["inputCount"] = static_cast<int>(codes.size());
        auto resp = HttpResponse::newHttpJsonResponse(response);
        resp->setStatusCode(k200OK);
        callback(resp);
    } catch (const std::exception& e) {
        Json::Value res;
        res["status"] = "error";
        res["message"] = std::string("服务器内部错误: ") + e.what();
        auto resp = HttpResponse::newHttpJsonResponse(res);
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
    }
}