#ifndef GROUND_HPP
#define GROUND_HPP 

class Ground;

#include <vector>
#include <iostream>
#include <array>
#include <algorithm>

#include "csvReader/reader.hpp"
#include "aStar/graph.hpp"
#include "robot/foot.hpp"
#include "utils/geometry.hpp"

/**
 * @brief 地面类
 * 表示机器人的工作环境，包含地形高度数据和相关分析功能
 */
class Ground {
public:
    /**
     * @brief 构造函数，从指定文件读取地形数据
     * @param filename 包含地形高度数据的CSV文件路径
     */
    Ground(std::string filename);

    std::vector<std::vector<double>> map;    ///< 地形高度数据
    std::vector<std::vector<double>> costs;  ///< 地形代价数据

    /**
     * @brief 通过指定区域的点计算最优平面
     * @param area 包含地面点坐标的向量
     * @return 表示最优平面的CuPlain对象
     */
    CuPlain trip(std::vector<Point>& area);
    
    /**
     * @brief 通过指定区域的点计算最优平面的法线
     * @param area 包含地面点坐标的向量
     * @return 表示最优平面法线的CuDot对象
     */
    CuDot normal(std::vector<Point>& area);
    
    /**
     * @brief 通过指定区域的点计算凸面（未实现）
     * @param area 包含地面点坐标的向量
     * @return 表示凸面的CuPlain对象
     */
    CuPlain convex_trip(std::vector<Point>& area);
    
    /**
     * @brief 计算指定区域的站立角度
     * @param area 包含地面点坐标的向量
     * @return 区域的站立角度（弧度）
     */
    double stand_angle(std::vector<Point>& area);

    std::array<int, 2> shape() const;
};

#endif