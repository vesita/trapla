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

/**
 * @brief 足部枚举，表示左脚或右脚
 */
enum class WhichFoot { Left, Right };

/**
 * @brief 机器人类
 * 实现双足机器人的运动控制和路径规划功能
 */
class Robot {
public:
    /**
     * @brief 机器人的两只脚
     * 使用数组存储左右两只脚，通过索引访问
     */
    std::array<Foot, 2> feet;

    /**
     * @brief 最大步长
     * 机器人单步行走的最大距离
     */
    double max_stride;

    /**
     * @brief 最大转向角
     * 机器人单步行走的最大转向角度（弧度）
     */
    double max_turn;

    /**
     * @brief 最大足部间距
     * 机器人两足之间的最大距离
     */
    double max_foot_separation;

    /**
     * @brief 最小足部间距
     * 机器人两足之间的最小距离
     */
    double min_foot_separation;


    /**
     * @brief 当前需要移动的脚
     * 表示当前作为摆动脚的脚（Left或Right）
     */
    WhichFoot now_which_foot_to_move;

    
    
    /**
     * @brief 构造函数，初始化机器人参数
     * 
     * @param max_stride 最大步长
     * @param max_turn 最大转向角（弧度）
     * @param max_foot_separation 最大足部间距
     * @param min_foot_separation 最小足部间距
     * @param foot_length 足部长度
     * @param foot_width 足部宽度
     */
    Robot(double max_stride=40.0, double max_turn= M_PI * 75.0/180.0, double max_foot_separation=10.0, double min_foot_separation=2.0,
        double foot_length=5.0, double foot_width=3.0);

    
    /**
     * @brief 更新行走状态，切换支撑脚和摆动脚
     */
    void walk_update();

    /**
     * @brief 获取摆动脚的x坐标引用
     * 
     * @return 摆动脚的x坐标引用
     */
    double& sw_x();

    /**
     * @brief 获取摆动脚的y坐标引用
     * 
     * @return 摆动脚的y坐标引用
     */
    double& sw_y();

    /**
     * @brief 获取摆动脚的朝向角引用
     * 
     * @return 摆动脚的朝向角引用
     */
    double& sw_rz();

    /**
     * @brief 获取支撑脚的x坐标引用
     * 
     * @return 支撑脚的x坐标引用
     */
    double& sp_x();

    /**
     * @brief 获取支撑脚的y坐标引用
     * 
     * @return 支撑脚的y坐标引用
     */
    double& sp_y();

    /**
     * @brief 获取支撑脚的朝向角引用
     * 
     * @return 支撑脚的朝向角引用
     */
    double& sp_rz();
    
    /**
     * @brief 计算理想行走区域
     * 
     * 根据当前支撑脚的位置和朝向，计算摆动脚可能的落足点区域
     * 
     * @param ground 地形对象
     * @return 可能的落足点区域
     */
    std::vector<SqDot> ideal_walk(const Ground& ground);

    /**
     * @brief 获取当前摆动脚的引用
     * 
     * @return 当前摆动脚的引用
     */
    Foot& get_swing_foot();

    /**
     * @brief 获取当前支撑脚的引用
     * 
     * @return 当前支撑脚的引用
     */
    Foot& get_support_foot();

    /**
     * @brief 计算到新位置的距离
     * 
     * @param new_pos 新位置
     * @return 到新位置的距离
     */
    double distance(const SqDot& new_pos);
    
    /**
     * @brief 检查新位置是否满足足部间距限制条件
     * 
     * @param new_pos 新位置
     * @return 如果满足限制条件返回true，否则返回false
     */
    bool satisfy_spacing(const SqDot& new_pos);

    /**
     * @brief 检查新位置是否满足步长限制条件
     * 
     * @param new_pos 新位置
     * @return 如果满足限制条件返回true，否则返回false
     */
    bool satisfy_stride(const SqDot& new_pos);

    /**
     * @brief 检查新位置是否满足转向限制条件
     * 
     * @param new_pos 新位置
     * @return 如果满足限制条件返回true，否则返回false
     */
    bool satisfy_turn(const SqDot& new_pos);
    
    /**
     * @brief 滑动调整足部落足区域
     * 
     * 根据地面法向量调整足部的落足区域，以获得更好的稳定性
     * 
     * @param area 足部落足区域
     * @param ground 地形对象
     * @return 滑动调整结果
     */
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);

    
    /**
     * @brief 根据引导点行走
     * 
     * @param ground 地形对象
     * @param goal 引导点
     * @return 目标落足点
     */
    SqDot walk_with_guide(const Ground& ground, const SqDot& goal);

    
    /**
     * @brief 调整目标点以适应地形约束
     * 
     * @param ground 地形对象
     * @param goal 原始目标点
     * @return 调整后的目标点
     */
    SqDot fit_target(const Ground& ground, const SqDot& goal);

    
    /**
     * @brief 计算直接目标点
     * 
     * 根据当前位置和目标点计算下一步的直接目标点
     * 
     * @param ground 地形对象
     * @param goal 最终目标点
     * @return 直接目标点
     */
    SqDot direct_target(const Ground& ground, const SqDot& goal);

    
    /**
     * @brief 查找从当前位置到目标点的路径
     * 
     * @param ground 地形对象
     * @param goal 目标点
     * @return 路径点序列
     */
    std::vector<SqDot> find_path(const Ground& ground, const SqDot& goal);
};

#endif