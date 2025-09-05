#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

enum class CuPos;

class CuPlain;
class CuLine;
class CuDot;

#include <vector>
#include <array>
#include <cmath>

#include "aStar/graph.hpp"

/**
 * @brief 点相对于平面位置的枚举类型
 * Above: 点在平面法向量指向的一侧
 * Below: 点在平面法向量相反的一侧
 * Inside: 点在平面上
 */
enum class CuPos { Above, Below, Inside };

/**
 * @brief 三维空间中的点类
 * 表示一个具有x、y、z坐标的点
 */
class CuDot {
public:
    double x;  ///< 点的x坐标
    double y;  ///< 点的y坐标
    double z;  ///< 点的z坐标

    /**
     * @brief 构造函数，创建一个点
     * @param x 点的x坐标，默认为0.0
     * @param y 点的y坐标，默认为0.0
     * @param z 点的z坐标，默认为0.0
     */
    CuDot(double x=0.0, double y=0.0, double z=0.0);
    
    /**
     * @brief 重载加法运算符，实现两个点坐标的相加
     * @param dot 另一个点
     * @return 两个点坐标相加后的新点
     */
    CuDot operator+(const CuDot& dot) const;

    Point slide();
};

/**
 * @brief 三维空间中的直线类
 * 通过一个点来表示直线的方向
 */
class CuLine {
public: 
    CuDot point;  ///< 直线上的点

    /**
     * @brief 构造函数，创建一条直线
     * @param point 直线上的点，默认为原点
     */
    CuLine(const CuDot& point=CuDot());

    /**
     * @brief 计算点相对于直线的偏移位置
     * @param dot 参考点
     * @return 偏移后的新点
     */
    CuDot offset(const CuDot& dot);

    /**
     * @brief 获取直线上的点
     * @return 直线上的点
     */
    CuDot get();

    /**
     * @brief 计算两条直线的法向量（叉积）
     * @param other 另一条直线
     * @return 表示法向量的CuLine对象
     */
    CuLine normal_vector(const CuLine& other) const;
};

/**
 * @brief 三维空间中的平面类
 * 使用平面方程Ax + By + Cz + D = 0表示平面
 */
class CuPlain {
public:
    double A;  ///< 平面方程系数A
    double B;  ///< 平面方程系数B
    double C;  ///< 平面方程系数C
    double D;  ///< 平面方程系数D

    int define_extend;  ///< 扩展定义标志

    /**
     * @brief 构造函数，创建一个平面
     * @param A 平面方程系数A，默认为0.0
     * @param B 平面方程系数B，默认为0.0
     * @param C 平面方程系数C，默认为0.0
     * @param D 平面方程系数D，默认为0.0
     */
    CuPlain(double A=0.0, double B=0.0, double C=0.0, double D=0.0);
    
    /**
     * @brief 通过三个点定义一个平面
     * @param dot 包含三个点的数组
     * @return 如果成功定义平面返回true，如果三点共线返回false
     */
    bool define_plaine(const std::array<CuDot, 3>& dot);
    
    /**
     * @brief 判断点相对于平面的位置关系
     * @param dot 待判断的点
     * @return 返回点相对于平面的位置枚举值
     */
    CuPos get_pos(const CuDot& dot) const;
    
    /**
     * @brief 获取平面的法向量
     * @return 表示平面法向量的CuDot对象
     */
    CuDot normal_vector() const;
    
    /**
     * @brief 计算平面法向量与z轴的夹角
     * @return 平面法向量与z轴的夹角（弧度）
     */
    double normal_angle() const;
};


#endif