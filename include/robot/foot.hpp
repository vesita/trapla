#ifndef FOOT_HPP
#define FOOT_HPP

#include <array>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "utils/geometry.hpp"
#include "ground/ground.hpp"

enum class SlideResult { Modified, NoModification, NotApplicable };

class Foot;

#include "robot/foot.hpp"

/**
 * @brief 足部形状类
 * 定义足部的几何形状和相关操作
 */
class FootShape {
public:
    double length{};
    double width{};

    
    /**
     * @brief 默认构造函数，创建一个位于原点的足部形状对象
     */
    FootShape();
    
    
    /**
     * @brief 构造函数，创建指定尺寸的足部形状对象
     * 
     * @param length 足部长度
     * @param width 足部宽度
     */
    FootShape(double length, double width);
    
    
    /**
     * @brief 检查指定点是否在足部范围内
     * 
     * @param l_side 长度方向坐标
     * @param w_side 宽度方向坐标
     * @return 如果点在足部范围内返回true，否则返回false
     */
    bool inside(double l_side, double w_side);

    
    /**
     * @brief 滑动调整足部落足区域以提高稳定性
     * 
     * 根据地面法向量调整足部的落足区域，以获得更好的稳定性
     * 
     * @param area 足部落足区域
     * @param ground 地形对象
     * @return 滑动调整结果
     */
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);
};

/**
 * @brief 足部类
 * 表示机器人的单个足部，包含位置、朝向和形状信息
 */
class Foot {
public:
    /**
     * @brief 足部位置坐标
     * 使用SqDot类型表示足部在地面上的坐标位置
     */
    SqDot position{};
    
    /**
     * @brief 足部绕Z轴的旋转角度（弧度）
     * 表示足部的朝向角度
     */
    double rz{};

    /**
     * @brief 足部形状
     * 定义足部的几何形状
     */
    FootShape shape{};

    /**
     * @brief 默认构造函数，创建一个位于原点的足部对象
     */
    Foot();

    Foot(SqDot position, double rz);

    /**
     * @brief 构造函数，创建一个指定位置和朝向的足部对象
     * 
     * @param position 足部位置
     * @param rz 足部朝向角（弧度）
     * @param shape_length 足部长度
     * @param shape_width 足部宽度
     */
    Foot(SqDot position, double rz, double shape_length, double shape_width);

    /**
     * @brief 设置足部位置和朝向
     * 
     * @param x x坐标
     * @param y y坐标
     * @param rz 朝向角（弧度）
     */
    void set(double x, double y, double rz);

    Foot next(const SqDot& new_pos) const;

    double direction_delta(const Foot& other) const;

    /**
     * @brief 计算足部在特定朝向下的覆盖区域
     * 
     * @param rz_of_foot 足部朝向角（弧度）
     * @return 足部覆盖的点集合
     */
    std::vector<SqDot> cover() const;

    std::vector<SqDot> corner() const;

    /**
     * @brief 让足部走向指定位置
     * 
     * @param ground 地形对象
     * @return 如果成功走向指定位置返回true，否则返回false
     */
    bool walkto(Ground& ground);
};

#endif