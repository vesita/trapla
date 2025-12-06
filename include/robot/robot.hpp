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
#include "aStar/direction.hpp"

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
    double active_distance = 0.0;

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

    void walk_update(const SqDot& new_pos);

    void last_walk_update(const SqDot& new_pos);

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
    const Foot& get_swing_foot() const;

    /**
     * @brief 获取当前支撑脚的引用
     * 
     * @return 当前支撑脚的引用
     */
    Foot& get_support_foot();
    const Foot& get_support_foot() const;

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

    bool position_check(const SqDot& new_pos);
    
    SqDot position();

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
    SqDot walk_to(const Ground& ground, const SqDot& guide_point);

    SqDot get_target(const SqDot& guide_point);
    
    /**
     * @brief 计算直接目标点
     * 
     * 根据当前位置和目标点计算下一步的直接目标点
     * 
     * @param ground 地形对象
     * @param goal 最终目标点
     * @return 直接目标点
     */
    SqDot direct_target(const SqDot& guide_point);

    SqDot find_target(const SqDot& guide_point);

    SqDot little_step();

    /**
     * @brief 根据引导点选择合适的摆动脚
     * 
     * @param first_guide 第一个引导点
     */
    void select_swing_foot(const SqDot& first_guide);

    /**
     * @brief 查找从当前位置到目标点的路径
     * 
     * @param ground 地形对象
     * @param goal 目标点
     * @return 路径点序列
     */
    std::vector<SqDot> find_path(const Ground& ground, const SqDot& goal);

    std::vector<SqDot> rot_neighbour(const SqDot& dot, double rot, double R);

    SqDot bfs(const Ground& ground, const SqDot& target);

    bool reach_target(const SqDot& target);

    double central_angle(const SqDot& dot);

    SqDot guide_bias(const SqDot& dot);

    SqDot arc_guide(const SqDot& dot);

    bool fit_foot(const SqDot& dot);
    
    /**
     * @brief 根据新位置计算足部的下一步状态
     * 
     * @param new_pos 新位置
     * @return 计算出的足部下一步状态
     */
    Foot next(const SqDot& new_pos) const;

    Foot last_next(const SqDot& new_pos) const;

    /**
     * @brief 检测两脚是否会发生交叉
     * 
     * @param left_foot_pos 左脚目标位置
     * @param right_foot_pos 右脚目标位置
     * @return 如果会发生交叉返回true，否则返回false
     */
    bool will_feet_cross(std::vector<SqDot>& pends);

    /**
     * @brief 交换左右脚的目标位置以避免交叉
     * 
     * @param left_foot_pos 左脚目标位置（会被修改）
     * @param right_foot_pos 右脚目标位置（会被修改）
     */
    void avoid_feet_cross(const Ground& ground, std::vector<SqDot>& pends);

    std::vector<SqDot> walk_candidates();
    
    void change_feet();

    /**
     * @brief 根据左右脚倾向生成候选点
     * 
     * @return 满足约束条件的候选点列表
     */
    std::vector<SqDot> biased_walk_candidates();
};

#endif