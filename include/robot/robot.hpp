#ifndef ROBOT_HPP
#define ROBOT_HPP

enum class WhichFoot;

#include <array>
#include <cmath>
#include <random>
#include <unordered_set>

#include "robot/foot.hpp"

enum class WhichFoot { Left, Right };

/**
 * @brief 机器人主类
 * 表示双足机器人，包含两个足部
 */
class Robot {
public:
    std::array<Foot, 2> feet;  ///< 机器人的两个足部
    double max_stride;              ///< 最大步长
    double max_turn;                ///< 最大转向角
    double max_foot_separation;     ///< 最大双足间距
    double min_foot_separation;     ///< 最小双足间距

    WhichFoot now_which_foot_to_move;

    
    /**
     * @brief 默认构造函数，创建一个机器人并初始化两个足部
     * @param max_stride 最大步长
     * @param max_turn 最大转向角(弧度)
     * @param max_foot_separation 最大双足间距
     * @param min_foot_separation 最小双足间距
     */
    Robot(double max_stride=40, double max_turn= M_PI * 75/180, double max_foot_separation=10, double min_foot_separation=2);

    void walk_update();

    std::vector<Point> ideal_walk(const Ground& ground);

    std::vector<Point> about_area(Point& target);

    /**
     * @brief 获取摆动足（即将移动的脚）
     * @return 摆动足的引用
     */
    Foot& getSwingFoot();

    /**
     * @brief 获取支撑足（当前支撑身体的脚）
     * @return 支撑足的引用
     */
    Foot& getSupportFoot();
};

#endif