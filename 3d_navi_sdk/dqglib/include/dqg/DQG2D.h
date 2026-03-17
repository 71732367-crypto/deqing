#include"DQGMathBasic.h"
//DQG2D
string LB2DQG_str(double l, double b, int level);

PointLBd DQG2LB_b(uint64_t DQGCode, int level);

uint64_t getDQGCellFatherBylevel_MTCode(uint64_t code, uint8_t level);

void getChildCode(uint64_t code, code_data* childrencodes);



unordered_map<string, vector<string>> multi_Codes(const vector<string>& arr);
vector<string> decompose_Codes(const unordered_map<string, vector<string>>& groups);

bool sort_codes(string a, string b);
bool sort_in_level(vector<string>& grids_codes);
bool sort_in_num(vector<string>& grids_codes);

std::vector<IJ> bresenham2D(const IJ& p1, const IJ& p2);
vector<string> bresenham2D(PointLBd& point1, PointLBd& point2, int level);

vector<string> rasterizeAndEncode(const vector<PointLBd>& lineLB, int level);