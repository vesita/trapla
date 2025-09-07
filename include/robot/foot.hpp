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
 * 
 * @details FootShape类描述了机器人足部的几何形状，包括长度和宽度。
 *          该类提供了判断点是否在足部范围内以及计算足部覆盖区域的方法。
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
     * 
     * @details 该方法用于检查指定点是否在足部矩形范围内。
     *          纵向坐标(l_side)对应足部长度方向，横向坐标(w_side)对应足部宽度方向。
     *          点在足部范围内的条件是：|l_side| <= length/2 且 |w_side| <= width/2
     */
    bool inside(double l_side, double w_side);
    
    /**
     * @brief 计算足部覆盖的地面点集合
     * @param rz_of_foot 足部绕垂直轴的旋转角度(弧度)
     * @return 包含所有被足部覆盖的地面点坐标的向量
     * 
     * @details 该方法根据足部的位置、姿态和形状计算足部与地面的接触区域。
     *          足部旋转角度影响足部在地面上的投影形状和位置。
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
     *          
     *          滑动策略：
     *          1. 计算当前区域的最优接触平面和法向量
     *          2. 计算法向量与z轴的夹角（站立角度）
     *          3. 如果角度超过阈值，则沿法线方向滑动足部位置
     *          4. 在新位置重新计算最优平面，比较稳定性
     *          5. 如果新位置更稳定，则更新足部位置
     */
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);
};

/**
 * @brief 足部类
 * 表示机器人足部的位置、姿态和行为
 * 
 * @details Foot类描述了机器人单个足部的状态，包括位置坐标(x,y)和绕垂直轴的旋转角度(rz)。
 *          该类提供了足部移动和稳定性检查的方法。
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
     * 
     * @details 该方法用于精确设置足部的位置和姿态，通常在足部移动或调整时使用。
     */
    void set(double x, double y, double rz);

    /**
     * @brief 模拟足部行走动作，检查足部是否能稳定地放置在指定地面位置
     * @param ground 地面对象，包含地形高度信息
     * @param shape 足部形状参数
     * @return 如果足部可以稳定放置返回true，否则返回false
     * 
     * @details 该方法模拟足部放置过程，包括以下步骤：
     *          1. 根据足部位置、姿态和形状计算足部覆盖区域
     *          2. 检查覆盖区域的稳定性（站立角度是否在允许范围内）
     *          3. 如果不稳定，尝试调整足部位置以提高稳定性
     *          4. 返回最终的稳定性检查结果
     */
    bool walkto(Ground& ground, FootShape& shape);
};

#endif