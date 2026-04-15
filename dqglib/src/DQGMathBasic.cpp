#include <dqg/DQGMathBasic.h>
#include <stdint.h>


/**
 * @brief        计算八分体
 * @param[in]   double Lng, double Lat
 * @return      uint16_t Oct
 * @author       安一宗
 * @date         2024.10.18
 **************************************
 * @note
 *
 **/

uint8_t LB2Oct(double Lng, double Lat)
{
     uint16_t OctNum = -1; // 默认值，表示无效的八分区

     if (Lat < -90 || Lat > 90 || Lng < -180 || Lng > 180)
     {
         return -1; // 无效的坐标
     }

     // 计算八分区编号
     OctNum = ((Lng > 0) ? (Lng > 90 ? 1 : 0) : (Lng >= -90 ? 3 : 2)) +
         ((Lat < 0) ? (Lat > -90 ? 4 : 6) : 0);
     return OctNum; // 返回八分区编号
}

/**
 * @brief        全局经纬度转局部经纬度
 * @param[in]   {uint64_t} i 行号
 * @return      {uint64_t}  经度差
 * @author       AnYiZong
 * @date         2024.10.18
 **************************************
 * @note
 *
 **/
PointLBd LBinOctant(double Lng, double Lat, uint8_t octNum)
{
  PointLBd P;
   if (octNum == 0)
    {
        // octNum == 0: 不变
        Lng = Lng;
        Lat = Lat;
    }
    else if (octNum == 1)
    {
        Lng -= 90;
        Lat = Lat;
    }
    else if (octNum == 2)
    {
        Lng += 180;
        Lat = Lat;
    }
    else if (octNum == 3)
    {
        Lng += 90;
        Lat = Lat;
    }
    else if (octNum == 4)
    {
        Lng = 90 - Lng;
        Lat = -Lat;
    }
    else if (octNum == 5)
    {
        Lng = 180 - Lng;
        Lat = -Lat;
    }
    else if (octNum == 6)
    {
        Lng = -Lng - 90;
        Lat = -Lat;
    }
    else
    {
        Lng = -Lng;
        Lat = -Lat;
    }
    P.Lng = Lng;
    P.Lat = Lat;
    return P;
}


//由八分体编号和恢复点的真实经纬度

void LBinGlobal(double& l,double& b, uint8_t oct){
  switch (oct)
  {
    case 0:
      break;
    case 1:
      l+=90;
      break;
    case 2:
      l-=180;
      break;
    case 3:
      l-=90;
      break;
    case 4:
      l=l;  b=-b;
      break;
    case 5:
      l+=90; b=-b;
      break;
    case 6:
      l-=180; b=-b;
      break;
    case 7:
      l-=90; b=-b;
      break;
    default:
      break;
  }
  return;
}


//经纬度转行列号

IJ LB2IJ(double l,double b, uint8_t level)
{
  IJ ij;
  uint16_t octNum = LB2Oct(l,b);                   // 1
    // Step 2: 将经纬度转换到八分区的局部坐标系（例如：将全局坐标映射到[0, 90]范围的局部坐标）
  PointLBd local_point = LBinOctant(l, b,octNum);  // 修改l和b的值，使其在八分区内归一化

    // Step 3: 计算纬度方向的分辨率（每个网格的高度间隔）
    double latVar = 90.0 / (1 << level);
  
    // Step 4: 计算行号
    int i = (1 << level) - ceil(local_point.Lat / latVar);  // 根据纬度值计算网格行号（从北向南排列）
  
    // Step 5: 计算经度差
    // 注意：log2(i+1.0)确保i=0时的层级为0，避免除零错误
    double lngVar = 90.0 / (1 << (int)ceil(log2(i + 1.0)));  // 经度间隔随i的层级动态调整
  
    // Step 6:  计算列号
    int j = floor(local_point.Lng / lngVar);  // 根据经度值计算网格列号（从西向东排列）
    ij.row=i;
    ij.column=j;
    return ij;
}
//经纬度高度转行列号层号
IJH LBH2IJH(double l,double b,double hei, uint8_t level)
{
  IJH ijh;
  uint16_t octNum = LB2Oct(l, b);                   // 1
  // Step 2: 将经纬度转换到八分区的局部坐标系（例如：将全局坐标映射到[0, 90]范围的局部坐标）
  PointLBd local_point = LBinOctant(l, b, octNum);  // 修改l和b的值，使其在八分区内归一化

  // Step 3: 计算纬度方向的分辨率（每个网格的高度间隔）
  double latVar = 90.0 / (1 << level);

  // Step 4: 计算行号
  int i = (1 << level) - ceil(local_point.Lat / latVar);  // 根据纬度值计算网格行号（从北向南排列）

  // Step 5: 计算经度差
  // 注意：log2(i+1.0)确保i=0时的层级为0，避免除零错误
  double lngVar = 90.0 / (1 << (int)ceil(log2(i + 1.0)));  // 经度间隔随i的层级动态调整

  // Step 6:  计算列号
  int j = floor(local_point.Lng / lngVar);  // 根据经度值计算网格列号（从西向东排列）

    double heighVar =(double)(10000000.0 / (1 << level));               // 7
    int Layer =static_cast<uint16_t>(std::floor(hei / heighVar));;        // 8

    ijh.row= i;
    ijh.column= j;
    ijh.layer= Layer;
    return ijh;

}

//莫顿码转行列号
IJ Morto2IJ(uint64_t GridMorton) {
  uint32_t  useless1 = 0;
  uint32_t  useless2 = 0;
  uint32_t  useless3 = 0;
 while (GridMorton & 0xffffffffffffffff) {
   useless1 |= (GridMorton & 1) << useless3;
   useless2 |= (GridMorton & 2) << useless3;
   GridMorton >>= 2;
   ++useless3;
 }

   IJ result;
   result.row = useless2 >> 1;
   result.column = useless1;
   return result;
}
//3d
IJH Morto2IJH(uint64_t morton) {
  uint64_t useless1 = 0;
  uint64_t useless2 = 0;
  uint64_t useless3 = 0;
  uint64_t useless4 = 0;

  while (morton) {
      useless1 |= (morton & 1) << useless4;
      useless2 |= (morton & 2) << useless4;
      useless3 |= (morton & 4) << useless4;
      morton >>= 3;
      ++useless4;
  }

  IJH result;
  result.layer = useless3 >> 2;
  result.row = useless2 >> 1;
  result.column = useless1;
  return result;
}

//行列号转莫顿码
uint64_t mortonEncode_2D_LUT(uint32_t x, uint32_t y){
  uint64_t z{};
  z = morton256_z[(x&0xff000000)>>24] <<49 |
      morton256_z[(y&0xff000000) >> 24]  << 48 |
      morton256_z[(x&0x00ff0000) >> 16]   << 33 |
      morton256_z[(y&0x00ff0000) >> 16]   << 32 |
      morton256_z[(x&0x0000ff00) >> 8 ]   << 17 |
      morton256_z[(y&0x0000ff00) >> 8 ]   << 16 |
      morton256_z[x & 0xFF] << 1 |
      morton256_z[y & 0xFF];
  return z;
}

uint64_t mortonEncode_3D_LUT(uint32_t y, uint32_t x, uint32_t z)
{
    uint64_t morton = 0;
    morton |= (morton256_z[(z & 0xff0000) >> 16] << 48);
    morton |= (morton256_z[(y & 0xff0000) >> 16] << 47);
    morton |= (morton256_z[(x & 0xff0000) >> 16] << 46);
    morton |= (morton256_z[(z & 0x00ff00) >> 8] << 24);
    morton |= (morton256_z[(y & 0x00ff00) >> 8] << 23);
    morton |= (morton256_z[(x & 0x00ff00) >> 8] << 22);
    morton |= morton256_z[z & 0x0000ff];
    morton |= (morton256_z[y & 0x0000ff] >> 1);
    morton |= (morton256_z[x & 0x0000ff] >> 2);

    return morton;
}


// 2D和3D经纬度转IJ和八分体号
IJ_oct_int LB2DQG_ij_oct_int(double l, double b, uint8_t level) {
    IJ_oct_int re;  // 存储结果的IJ索引、八分区、层级和间隔信息
    re.oct = LB2Oct(l, b);  // 调用函数计算八分区编号（例如：LB2Oct可能根据经纬度范围返回0-7）
    // Step 7: 填充结果结构体
    re.i = LB2IJ(l, b, level).column;             // 存储纬度索引
    re.j = LB2IJ(l, b, level).column;             // 存储经度索引
    re.level = level;     // 存储当前层级
    return re;
}

IJH_oct_int LBH2DQG_ijh_oct_int(double l, double b, double hei, uint8_t level) {
    IJH_oct_int re;  // 存储结果的IJ索引、八分区、层级和间隔信息

    // Step 1: 确定经纬度所在的八分区（0-7）
    re.oct = LB2Oct(l, b);  // 调用函数计算八分区编号（例如：LB2Oct可能根据经纬度范围返回0-7）

    auto ijhoct = LBH2IJH(l, b, hei, level);
    re.i = ijhoct.row;             // 存储纬度索引
    re.j = ijhoct.column;
    re.h = ijhoct.layer;
    re.level = level;     // 存储当前层级
    return re;
}



//2D和3DIJ和八分体号转DQG字符串
string IJ_oct_int2str(IJ_oct_int ij)
{
    uint64_t morton = mortonEncode_2D_LUT(ij.i, ij.j);
    //convert binary morton code to string
    //whose first character is octant in ascii
    string str(ij.level + 1, 0);
    str[0] = (ij.oct & 0x7) + 48; // 使用位掩码代替位移操作
    //the following character is morton code in quaternary
    for (int i = ij.level;i > 0;i--) {
        str[i] = (morton % 4) + 48;
        morton = morton >> 2;
    }
    return str;
}

string IJH_oct_int2str(IJH_oct_int IJH) {
    uint64_t MortonCode = mortonEncode_3D_LUT(IJH.i, IJH.j, IJH.h);
    std::string code;
    std::string Morton = toOctalString(MortonCode);
    if (Morton.length() != IJH.level)
    {
        Morton = std::string(IJH.level - Morton.length(), '0') + Morton; // 补零
    }
    // 拼接结果
    code = std::to_string(IJH.oct) + Morton;

    return code;
}


//3D morton码转字符串
 
std::string toOctalString(uint64_t number)
{
    try {
        if (number == 0)
            return "0"; // 处理零的特殊情况，返回"0"而不是空格
        
        std::vector<int> octalDigits;
        // 转换为八进制
        while (number > 0)
        {
            octalDigits.push_back(number % 8); // 取余
            number /= 8;                       // 除以 8
        }
        
        // 反转并构建结果字符串
        std::string octalStr;
        for (auto it = octalDigits.rbegin(); it != octalDigits.rend(); ++it)
        {
            octalStr += std::to_string(*it);
        }

        return octalStr;
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Error in toOctalString: ") + e.what());
    }
}
