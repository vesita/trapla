#ifndef FAST_FLATNESS_HPP
#define FAST_FLATNESS_HPP

#include "utils/geometry.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * @brief 快速评估区域平整度的工具类
 * 该类提供在地图缩放过程中快速评估区域平整度的方法
 */
class FastFlatnessEvaluator {
public:
    /**
     * @brief 快速评估指定区域的平整度
     * @param map 地图数据
     * @param center 区域中心点
     * @param side_length 区域边长
     * @return 平整度评估值，值越小表示越平整
     * 
     * @details 该方法通过计算区域内高度值的统计特征来快速评估平整度:
     *          1. 计算区域内所有点的高度标准差
     *          2. 计算区域内最大高度差
     *          3. 综合这两个指标得到平整度评估值
     *          
     *          相比于trip方法，该方法不需要进行复杂的平面拟合计算，
     *          大大提高了计算速度，适合在地图缩放过程中使用
     */
    static double evaluate(const SqPlain& map, const SqDot& center, int side_length);
    
    /**
     * @brief 快速评估指定区域的平整度（重载版本）
     * @param heights 区域内各点的高度值
     * @return 平整度评估值，值越小表示越平整
     * 
     * @details 该方法直接使用给定的高度值进行计算，避免重复访问地图数据
     */
    static double evaluate(const std::vector<double>& heights);
};

#endif // FAST_FLATNESS_HPP