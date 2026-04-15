/*
Copyright (c) [2025] [AnYiZong]
All rights reserved.
No part of this code may be copied or distributed in any form without permission.
*/


#include <dqg/DQG2D.h>
#include <stdint.h>


//DQG2D编码
string LB2DQG_str(double l, double b, int level) {
    // Step 1: 将经纬度转换为DQG_octant_ij结构体
    IJ_oct_int ij_o_int = LB2DQG_ij_oct_int(l, b, level);

    // Step 2: 将结构体转换为字符串编码
    return IJ_oct_int2str(ij_o_int);
}



//DQG2D解码
PointLBd DQG2LB_b(uint64_t DQGCode,int level){
    PointLBd lb;
    int octant=  DQGCode>>61;
    DQGCode&=0x1fffffffffffffff;
    IJ ij=Morto2IJ(DQGCode);
    double interval_B=90.0/(1<<level);
    double interval_L=90.0/(1<<(log2_d(ij.row+1)));
    lb.Lng=ij.column*interval_L;
    lb.Lat=90.0-ij.row*interval_B;
    LBinGlobal(lb.Lng,lb.Lat,octant);
    return lb;
  }

//父子网格计算
uint64_t getDQGCellFatherBylevel_MTCode(uint64_t code, uint8_t level) {
    try{
      if((code&0x1fffffffffffffff)==0){
        string msg = "code is octant,no father code??";
        throw msg;
      }
      //cal the Multi-DQG Level
      int i=0;
      uint64_t scode=code;
      while (!(scode &1))  { i++;  scode>>=2;}
      int clevel= kMaxLevel2D-i;
  
      if(level>=clevel){
        string msg = "input level  can not be bigger than code level ??";
        throw msg;
      }
      int num=61-level*2;
      uint64_t ffatherCode=((code>>num)<<num)|(1<<(num-1));
      return ffatherCode;
    }
    catch(const string msg){
      cerr << msg << std::endl;
      getchar();
      return 0; // 添加返回值以避免警告
    }
  }
  
  void getChildCode(uint64_t code,code_data* childrencodes) {
    uint64_t one=1;
    int i=0;
    uint64_t scode=code;
    while (!(scode &1))  { i++;  scode>>=1;}
    uint64_t ldm=one<<i;
    uint64_t lsp=code&((~code)+1);
    if(lsp==ldm)//triangle
    {
      childrencodes->size=3;
      childrencodes->data[0] = code^((uint64_t)5<<(i-2));
      childrencodes->data[1] = code^((uint64_t)1<<(i-2));
      childrencodes->data[2] = code^((uint64_t)3<<(i-2));
    }
    else{
      childrencodes->size=4;
      childrencodes->data[0] = code^((uint64_t)5<<(i-2));
      childrencodes->data[1] = code^((uint64_t)7<<(i-2));
      childrencodes->data[2] = code^((uint64_t)1<<(i-2));
      childrencodes->data[3] = code^((uint64_t)3<<(i-2));
    }
  }
  

//2D编码聚合
//使用了 unordered_map 作为哈希表，
//并使用 vector<string> 作为子网格数组的存储容器。
//同时，递归调用 multi_Codes 进行聚合。
unordered_map<string, vector<string>> multi_Codes(const vector<string>& arr) {
    unordered_map<string, vector<string>> groups;
    
    for (const auto& str : arr) {
        if (str.length() >= 2) {
            char secondLastChar = str[str.length() - 2];
            string key = str.substr(0, str.length() - 1);
            
            groups[key].push_back(str);
        }
    }
    
    vector<string> keys;
    for (const auto& pair : groups) {
        if (pair.second.size() == 8) {
            keys.push_back(pair.first);
        }
    }
    
    if (keys.size() > 1) {
        unordered_map<string, vector<string>> newGroups = multi_Codes(keys);
        
        for (const auto& pair : newGroups) {
            groups[pair.first] = pair.second;
        }
    }
    
    for (const auto& key : keys) {
        groups.erase(key);
    }
    
    return groups;
  }
  
  //2d编码分解
  vector<string> decompose_Codes(const unordered_map<string, vector<string>>& groups) {
    vector<string> decomposed;
    
    for (const auto& pair : groups) {
        if (!pair.second.empty()) {
            decomposed.insert(decomposed.end(), pair.second.begin(), pair.second.end());
        } else {
            decomposed.push_back(pair.first);
        }
    }
    
    return decomposed;
  }
  
  //2D编码排序
// 1. 按字符串长度排序，长度短的排前面
// 2. 若长度相同，则按字典序排序
bool sort_codes(string a, string b) {
    if (a.size() != b.size()) {
        return a.size() < b.size();
    } else {
        return a < b;
    }
  }
  
  
  // 对编码进行排序
  bool sort_in_level(vector<string>& grids_codes) {
    try {
        if (grids_codes.empty()) {
            string s = "Error: List of grid codes is empty!";
            throw s;
        }
        sort(grids_codes.begin(), grids_codes.end(), sort_codes);
        return true;
    } catch (string tips) {
        cerr << tips << endl;
        return false;
    }
  }
  
  // 
  bool sort_in_num(vector<string>& grids_codes) {
    try {
        if (grids_codes.empty()) {
            string s = "Error: List of grid codes is empty!";
            throw s;
        }
        sort(grids_codes.begin(), grids_codes.end());
        return true;
    } catch (string tips) {
        cerr << tips << endl;
        return false;
    }
  }


  /**
 * 二维直线编码算法
 * @param point1 起点行列号坐标
 * @param point2 终点行列号坐标
 * @returns 直线上所有点的坐标
 */
 /**
 * 二维直线编码算法
 * @param point1 起点行列号坐标
 * @param point2 终点行列号坐标
 * @returns 直线上所有点的坐标
 */
  vector<string> bresenham2D(PointLBd& point1, PointLBd& point2, int level) {

      std::vector<IJ> IJret;
      std::vector<IJ_oct_int> IJ_oct_ret;

      std::vector<string> ret;

      IJ p1, p2;
      int Oct = LB2Oct(point1.Lng, point1.Lat);
      p1.row = LB2IJ(point1.Lng, point1.Lat, level).row;
      p1.column = LB2IJ(point1.Lng, point1.Lat, level).column;
      p2.row = LB2IJ(point1.Lng, point1.Lat, level).row;
      p2.column = LB2IJ(point1.Lng, point1.Lat, level).column;
      int dx = ABS(static_cast<int>(p2.column) - static_cast<int>(p1.column));
      int dy = ABS(static_cast<int>(p2.row) - static_cast<int>(p1.row));
      int sx = (p1.column < p2.column) ? 1 : -1;
      int sy = (p1.row < p2.row) ? 1 : -1;
      int err = dx - dy;

      IJ current = p1; // 使用一个临时点来进行操作

      while (true) {
          IJret.push_back({ current.row, current.column }); // 改为 IJ 类型
          if (current.column == p2.column && current.row == p2.row) break;
          int err2 = err * 2;
          if (err2 > -dy) {
              err -= dy;
              current.column += sx;
          }
          if (err2 < dx) {
              err += dx;
              current.row += sy;
          }
      }
      for (int i = 0;i < IJret.size();++i)
      {
          IJ_oct_ret[i].i = IJret[i].row;
          IJ_oct_ret[i].j = IJret[i].column;
          IJ_oct_ret[i].oct = Oct;
          IJ_oct_ret[i].level = level;

          ret.push_back(IJ_oct_int2str(IJ_oct_ret[i]));
      }
      return ret;

  }

  /// @brief 二维扫描线填充算法
/// @param 
/// @return 
vector<string> rasterizeAndEncode(const vector<PointLBd>& lineLB, int level) {
  // 1. 经纬度转换为 IJ 坐标
  vector<IJ> ijPoints;
  for (const auto& pt : lineLB) {
      ijPoints.push_back(LB2IJ(pt.Lng, pt.Lat, level));
  }
  
  // 2. 扫描线填充
  unordered_map<uint32_t, vector<uint32_t>> groupedByRow;
  for (const auto& mapItem : ijPoints) {
      groupedByRow[mapItem.row].push_back(mapItem.column);
  }
  
  vector<IJ> faceArray;
  for (const auto& [row, columns] : groupedByRow) {
      auto minCol = *min_element(columns.begin(), columns.end());
      auto maxCol = *max_element(columns.begin(), columns.end());
      for (uint32_t col = minCol; col <= maxCol; ++col) {
          faceArray.push_back({row, col});
      }
  }
  
  // 3. 生成 Morton 编码
  vector<string> codes;
  for (const auto& ij : faceArray) {
      uint64_t morton = mortonEncode_2D_LUT(ij.column, ij.row);
      string code(level + 1, '0');
      code[0] = '0' + level;
      for (int i = 0; i < level; ++i) {
          int shift = 2 * (level - 1 - i);
          uint64_t digit = (morton >> shift) & 0x3;
          code[i + 1] = '0' + digit;
      }
      codes.push_back(code);
  }
  
  return codes;
}