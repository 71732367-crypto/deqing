
#ifndef DQG_ADJACENT_SEARCH_HPP
#define DQG_ADJACENT_SEARCH_HPP

#include <dqg/DQG3DBasic.h>
#include <sstream>
#include <stdexcept>
#include <iomanip>


///@brief  生成相邻网格的 DQG 编码
string getAdjacentCode(const string& code, uint64_t row, uint64_t col, uint64_t hei);

///@brief 计算给定网格的所有相邻网格编码
vector<string> faceAdjacentSearch(const string& code);
#endif // DQG_ADJACENT_SEARCH_HPP