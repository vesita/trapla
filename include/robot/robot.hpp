#ifndef ROBOT_HPP
#define ROBOT_HPP

enum class WhichFoot;

#include <array>
#include <cmath>
#include <random>
#include <unordered_set>

#include "robot/foot.hpp"
#include "utils/geometry.hpp"
#include "aStar/aStar.hpp"


enum class WhichFoot { Left, Right };

/**
 * @brief 机器人主类
 * 表示双足机器人，包含两个足部
 * 
 * @details Robot类封装了双足机器人的状态和行为，包括两个足部的位置和姿态，
 *          以及机器人的运动参数。该类提供了路径规划、步态生成和稳定性控制等功能。
 */
class Robot {
public:
    std::array<Foot, 2> feet;  ///< 机器人的两个足部
    double max_stride;              ///< 最大步长
    double max_turn;                ///< 最大转向角
    double max_foot_separation;     ///< 最大双足间距
    double min_foot_separation;     ///< 最小双足间距
    FootShape foot_shape;        ///< 足部形状参数

    WhichFoot now_which_foot_to_move;

    
    /**
     * @brief 默认构造函数，创建一个机器人并初始化两个足部
     * @param max_stride 最大步长
     * @param max_turn 最大转向角(弧度)
     * @param max_foot_separation 最大双足间距
     * @param min_foot_separation 最小双足间距
     * @param foot_length 足部长度
     * @param foot_width 足部宽度
     * 
     * @details 该构造函数初始化机器人的所有参数，包括运动约束和足部形状。
     *          默认参数适用于一般场景，可根据具体需求进行调整。
     */
    Robot(double max_stride=40, double max_turn= M_PI * 75/180, double max_foot_separation=10, double min_foot_separation=2,
        double foot_length=5, double foot_width=3);

    /**
     * @brief 更新机器人行走状态
     * 
     * @details 该方法在机器人完成一步行走后调用，用于更新机器人的状态，
     *          包括切换支撑足和摆动足。
     */
    void walk_update();

    /**
     * @brief 生成理想行走路径
     * @param ground 地面对象
     * @return 包含路径点的向量
     * 
     * @details 该方法生成从机器人当前位置到目标位置的理想行走路径，
     *          路径点表示足部的理想落足位置。
     */
    std::vector<SqDot> ideal_walk(const Ground& ground);

    /**
     * @brief 检查新位置是否满足足部限制条件
     * @param new_pos 新位置点
     * @return 如果满足限制条件返回true，否则返回false
     * 
     * @details 该方法检查新位置是否满足以下条件：
     *          1. 步长不超过最大步长
     *          2. 双足间距在允许范围内
     *          3. 转向角度不超过最大转向角
     */
    bool satisfy_foot_limits(const SqDot& new_pos);

    /**
     * @brief 获取摆动足（即将移动的脚）
     * @return 摆动足的引用
     * 
     * @details 摆动足是机器人行走过程中即将移动的脚，与支撑足相对。
     */
    Foot& getSwingFoot();

    /**
     * @brief 获取支撑足（当前支撑身体的脚）
     * @return 支撑足的引用
     * 
     * @details 支撑足是机器人行走过程中当前支撑身体的脚，与摆动足相对。
     */
    Foot& getSupportFoot();

    /**
     * @brief 调整足部覆盖区域以提高稳定性
     * @param area 足部当前覆盖的地面点区域
     * @param ground 地面对象，包含地形高度信息
     * @return SlideResult 操作结果枚举值
     * 
     * @details 该方法调用FootShape类的slide方法，对足部覆盖区域进行稳定性优化。
     */
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);

    /**
     * @brief 根据引导点行走
     * @param ground 地面对象
     * @param goal 目标点
     * @return 实际落足点
     * 
     * @details 该方法根据引导路径点计算实际的足部落足点，
     *          考虑地形约束和机器人运动学约束。
     */
    SqDot walk_with_guide(const Ground& ground, const SqDot& goal);

    /**
     * @brief 调整目标点以适应地形约束
     * @param ground 地面对象
     * @param goal 目标点
     * @return 调整后的目标点
     * 
     * @details 当目标点不可达时，该方法计算最接近目标点的可行走位置。
     */
    SqDot fit_target(const Ground& ground, const SqDot& goal);

    /**
     * @brief 计算直接目标点
     * @param ground 地面对象
     * @param goal 目标点
     * @return 直接目标点
     * 
     * @details 该方法计算从支撑足到目标点方向上的最远可达点。
     */
    SqDot direct_target(const Ground& ground, const SqDot& goal);

    /**
     * @brief 查找路径
     * @param ground 地面对象
     * @param goal 目标点
     * @return 路径点序列
     * 
     * @details 该方法使用A*算法在地面地图上查找从当前位置到目标位置的路径。
     */
    std::vector<SqDot> find_path(const Ground& ground, const SqDot& goal);
};

#endif