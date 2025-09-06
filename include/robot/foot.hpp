#ifndef FOOT_HPP
#define FOOT_HPP

class Foot;
class FootShape;

#include <cmath>
#include <unordered_set>

#include "ground/ground.hpp"
#include "utils/geometry.hpp"

/**
 * @brief slide操作结果的枚举类型
 * 表示slide操作的不同结果状态
 */
enum class SlideResult {
    NoModification,    ///< 未发生修改
    Modified,          ///< 发生了修改
    NotApplicable      ///< 不适用（如地面数据无效）
};

/**
 * @brief 足部形状类
 * 定义足部的几何形状参数和相关操作
 */
class FootShape {
public:
    double length{};  ///< 足部长度
    double width{};   ///< 足部宽度

    /**
     * @brief 默认构造函数，初始化足部形状参数为0
     */
    FootShape();
    
    /**
     * @brief 构造函数，使用指定的长度和宽度初始化足部形状
     * @param length 足部长度
     * @param width 足部宽度
     */
    FootShape(double length, double width);
    
    /**
     * @brief 判断指定坐标点是否在足部形状内部
     * @param l_side 相对于足部中心的纵向坐标
     * @param w_side 相对于足部中心的横向坐标
     * @return 如果点在足部范围内返回true，否则返回false
     */
    bool inside(double l_side, double w_side);
    
    /**
     * @brief 计算足部覆盖的地面点集合
     * @param rz_of_foot 足部绕垂直轴的旋转角度(弧度)
     * @return 包含所有被足部覆盖的地面点坐标的向量
     */
    std::vector<SqDot> cover(double& rz_of_foot);

    /**
     * @brief 调整足部覆盖区域以提高稳定性
     * @param area 足部当前覆盖的地面点区域
     * @param ground 地面对象，包含地形高度信息
     * @return SlideResult 操作结果枚举值
     * 
     * @details 该方法通过分析足部覆盖区域的地形数据，计算最优的接触平面，
     *          并根据平面的倾斜角度决定是否需要调整足部位置以提高稳定性。
     *          如果当前区域已经足够平坦，则不进行调整；
     *          如果需要调整，则尝试在法线指示的方向上寻找更平坦的区域。
     */
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);
};

/**
 * @brief 足部类
 * 表示机器人足部的位置、姿态和行为
 */
class Foot {
public:
    double x{};   ///< 足部在世界坐标系中的x坐标
    double y{};   ///< 足部在世界坐标系中的y坐标
    double rz{};  ///< 足部绕垂直轴的旋转角度(弧度)

    /**
     * @brief 默认构造函数，初始化足部位置和姿态为(0,0,0)
     */
    Foot();

    /**
     * @brief 设置足部的位置和姿态
     * @param x 足部在世界坐标系中的x坐标
     * @param y 足部在世界坐标系中的y坐标
     * @param rz 足部绕垂直轴的旋转角度(弧度)
     */
    void set(double x, double y, double rz);

    /**
     * @brief 模拟足部行走动作，检查足部是否能稳定地放置在指定地面位置
     * @param ground 地面对象，包含地形高度信息
     * @param shape 足部形状参数
     * @return 如果足部可以稳定放置返回true，否则返回false
     */
    bool walkto(Ground& ground, FootShape& shape);
};

#endif