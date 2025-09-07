#ifndef GROUND_HPP
#define GROUND_HPP 

class Ground;

#include <vector>
#include <iostream>
#include <array>
#include <algorithm>

#include "csvReader/reader.hpp"
#include "robot/foot.hpp"
#include "utils/geometry.hpp"

/**
 * @brief 地面类
 * 表示机器人的工作环境，包含地形高度数据和相关分析功能
 * 
 * @details Ground类封装了地形高度数据和相关分析方法，用于支持机器人路径规划和稳定性分析。
 *          地形数据以二维网格形式存储，每个网格点包含该位置的高度信息。
 */
class Ground {
public:
    /**
     * @brief 构造函数，从指定文件读取地形数据
     * @param filename 包含地形高度数据的CSV文件路径
     * 
     * @details 该构造函数从CSV文件中读取地形高度数据，并初始化地图和代价矩阵。
     *          代价矩阵基于高度数据计算得出，用于路径规划。
     */
    Ground(std::string filename);

    SqPlain map;    ///< 地形高度数据
    SqPlain costs;  ///< 地形代价数据

    /**
     * @brief 通过指定区域的点计算最优平面
     * @param area 包含地面点坐标的向量
     * @return 表示最优平面的CuPlain对象
     * 
     * @details 该方法使用三点迭代算法计算最优平面：
     *          1. 将输入的点按照高度排序
     *          2. 选择初始的三个点定义初始平面
     *          3. 遍历剩余的点，如果发现有更优的点组合，
     *             则用该点替换当前平面的一个点并重新计算平面
     *          4. 返回最终的最优平面
     */
    CuPlain trip(std::vector<SqDot>& area);
    
    /**
     * @brief 通过指定区域的点计算最优平面的法线
     * @param area 包含地面点坐标的向量
     * @return 表示最优平面法线的CuDot对象
     * 
     * @details 该方法首先调用trip方法计算最优平面，然后获取该平面的法向量。
     */
    CuDot normal(std::vector<SqDot>& area);
    
    /**
     * @brief 通过指定区域的点计算凸面（未实现）
     * @param area 包含地面点坐标的向量
     * @return 表示凸面的CuPlain对象
     * 
     * @deprecated 此方法尚未实现
     */
    CuPlain convex_trip(std::vector<SqDot>& area);
    
    /**
     * @brief 计算指定区域的站立角度
     * @param area 包含地面点坐标的向量
     * @return 区域的站立角度（弧度）
     * 
     * @details 站立角度是指区域最优平面法向量与z轴（重力方向）的夹角。
     *          该角度用于评估足部放置的稳定性，角度越小表示越稳定。
     */
    double stand_angle(std::vector<SqDot>& area);

    /**
     * @brief 获取地面地图的尺寸
     * @return 包含行数和列数的数组
     */
    std::array<int, 2> shape() const;

    /**
     * @brief 检查地面地图是否为空
     * @return 如果地图为空返回true，否则返回false
     */
    bool empty() const;
};

#endif