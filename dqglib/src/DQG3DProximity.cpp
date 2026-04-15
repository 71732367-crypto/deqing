/*
Copyright (c) [2025] [AnYiZong]
All rights reserved.
No part of this code may be copied or distributed in any form without permission.
*/

//TODO：不完整
#include <dqg/DQG3DProximity.h>
#include <stdint.h>
/**
 * @brief 生成相邻网格的 DQG 编码
 * @param code 当前网格的 DQG 编码
 * @param row 目标网格的行号
 * @param col 目标网格的列号
 * @param hei 目标网格的层号
 * @return string 相邻网格的 DQG 编码
 */
string getAdjacentCode(const string& code, uint64_t row, uint64_t col, uint64_t hei) {
    if (code.empty()) {
        throw invalid_argument("Invalid DQG code");
    }

    int level = code.length() - 1;

    // 生成3D Morton编码
    uint64_t morton = mortonEncode_3D_LUT(row, col, hei);

    // 转换为八进制字符串
    ostringstream oss;
    oss << oct << setw(level) << setfill('0') << morton;
    string mortonStr = oss.str();

    // 确保字符串长度不超过当前层级
    if (mortonStr.length() > static_cast<size_t>(level)) {
        mortonStr = mortonStr.substr(mortonStr.length() - level);
    }

    // 组合最终编码
    return string(1, code[0]) + mortonStr;
}
/**
 * @brief 计算给定网格的所有相邻网格编码
 *
 * @param code 当前网格的 DQG 编码
 * @return vector<string> 相邻网格的 DQG 编码集合
 */
vector<string> faceAdjacentSearch(const string& code) {
    vector<string> adjacentCodeSet;
    if (code.empty()) throw invalid_argument("Invalid DQG code");

    // 解析基础参数
    const char Q = code[0];
    const int level = code.length() - 1;
    const uint64_t rowMax = (1ULL << level) - 1;
    const uint64_t heiMax = rowMax;

    // 解码莫顿码
    const string octPart = code.substr(1);
    const uint64_t morton = stoull(octPart, nullptr, 8);
    const IJH obj = Morto2IJH(morton);
    const uint64_t row = obj.row;
    const uint64_t col = obj.column;
    const uint64_t hei = obj.layer;
    const uint64_t nextRow = row + 1;
    const uint64_t nextCol = col + 1;

    // 计算列最大值
    const uint64_t colMax = (1ULL << log2_d(row + 1)) - 1;

    // 退化网格判断（需提前定义）
    static const vector<uint64_t> degenerateTop = {/*/////////////////////////////////////////*/ };
    static const vector<uint64_t> degenerateDown = {/*/////////////////////////////////////////*/ };

    // 类型A网格处理
    if (row == 0 && col == 0 && hei == 0) {
        const string topCode = getAdjacentCode(code, row, col, hei + 1);
        const string frontCode1 = getAdjacentCode(code, nextRow, col, hei);
        const string frontCode2 = getAdjacentCode(code, nextRow, nextCol, hei);

        switch (Q) {
        case '0':
            adjacentCodeSet = { topCode, frontCode1, frontCode2,
                              "3" + octPart, "1" + octPart };
            break;
        case '1':
            adjacentCodeSet = { topCode, frontCode1, frontCode2,
                              "0" + octPart, "2" + octPart };
            break;
        case '2':
            adjacentCodeSet = { topCode, frontCode1, frontCode2,
                              "1" + octPart, "3" + octPart };
            break;
        case '3':
            adjacentCodeSet = { topCode, frontCode1, frontCode2,
                              "2" + octPart, "0" + octPart };
            break;
        case '4': {
            const string backCode1 = getAdjacentCode(code, nextRow, col, hei);
            const string backCode2 = getAdjacentCode(code, nextRow, nextCol, hei);
            adjacentCodeSet = { topCode, backCode1, backCode2,
                              "7" + octPart, "5" + octPart };
            break;
        }
        case '5': {
            const string backCode1 = getAdjacentCode(code, nextRow, col, hei);
            const string backCode2 = getAdjacentCode(code, nextRow, nextCol, hei);
            adjacentCodeSet = { topCode, backCode1, backCode2,
                              "4" + octPart, "6" + octPart };
            break;
        }
        case '6': {
            const string backCode1 = getAdjacentCode(code, nextRow, col, hei);
            const string backCode2 = getAdjacentCode(code, nextRow, nextCol, hei);
            adjacentCodeSet = { topCode, backCode1, backCode2,
                              "5" + octPart, "7" + octPart };
            break;
        }
        case '7': {
            const string backCode1 = getAdjacentCode(code, nextRow, col, hei);
            const string backCode2 = getAdjacentCode(code, nextRow, nextCol, hei);
            adjacentCodeSet = { topCode, backCode1, backCode2,
                              "6" + octPart, "4" + octPart };
            break;
        }
        }
    }
    // 上层三角形网格 (row == 0 && col == 0 && hei == heiMax)
    else if (row == 0 && col == 0 && hei == heiMax) {
        const string bottomCode = getAdjacentCode(code, row, col, hei - 1);
        const string frontCode1 = getAdjacentCode(code, nextRow, col, hei);
        const string frontCode2 = getAdjacentCode(code, nextRow, nextCol, hei);

        switch (Q) {
        case '0':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "3" + octPart, "1" + octPart };
            break;
        case '1':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "0" + octPart, "2" + octPart };
            break;
        case '2':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "1" + octPart, "3" + octPart };
            break;
        case '3':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "2" + octPart, "0" + octPart };
            break;
        case '4':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "7" + octPart, "5" + octPart };
            break;
        case '5':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "4" + octPart, "6" + octPart };
            break;
        case '6':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "5" + octPart, "7" + octPart };
            break;
        case '7':
            adjacentCodeSet = { bottomCode, frontCode1, frontCode2,
                              "6" + octPart, "4" + octPart };
            break;
        }
    }
    // 中层三角形网格 (0 < hei < heiMax)
    else if (row == 0 && col == 0 && hei > 0 && hei < heiMax) {
        const string topCode = getAdjacentCode(code, row, col, hei + 1);
        const string bottomCode = getAdjacentCode(code, row, col, hei - 1);
        const string frontCode1 = getAdjacentCode(code, nextRow, col, hei);
        const string frontCode2 = getAdjacentCode(code, nextRow, nextCol, hei);

        switch (Q) {
        case '0':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "3" + octPart, "1" + octPart };
            break;
        case '1':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "0" + octPart, "2" + octPart };
            break;
        case '2':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "1" + octPart, "3" + octPart };
            break;
        case '3':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "2" + octPart, "0" + octPart };
            break;
        case '4':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "7" + octPart, "5" + octPart };
            break;
        case '5':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "4" + octPart, "6" + octPart };
            break;
        case '6':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "5" + octPart, "7" + octPart };
            break;
        case '7':
            adjacentCodeSet = { topCode, bottomCode, frontCode1, frontCode2,
                              "6" + octPart, "4" + octPart };
            break;
        }
    }

    return adjacentCodeSet;
}