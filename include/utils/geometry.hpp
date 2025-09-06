#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

enum class Cupos;
struct SqDotHash;
class SqDot;
class SqPlain;
class CuDot;
class CuLine;
class CuPlain;

#include <vector>
#include <array>
#include <cmath>

/**
 * @brief 点相对于平面位置的枚举类型
 * Above: 点在平面法向量指向的一侧
 * Below: 点在平面法向量相反的一侧
 * Inside: 点在平面上
 */
enum class CuPos { Above, Below, Inside };


struct SqDotHash { 
    /**
     * @brief 计算SqDot对象的哈希值
     * @param p 待计算哈希值的点
     * @return 点的哈希值
     */
    std::size_t operator()(const SqDot& p) const;
};

/**
 * @brief 二维整数坐标点类
 * 表示一个具有x、y坐标的整数点，主要用于地图网格位置
 */
class SqDot { 
public:
    int x;  ///< 点的x坐标（行索引）
    int y;  ///< 点的y坐标（列索引）

    /**
     * @brief 构造函数，创建一个点
     * @param x 点的x坐标，默认为0
     * @param y 点的y坐标，默认为0
     */
    SqDot(int x=0, int y=0);
    
    /**
     * @brief 重载相等运算符，判断两个点是否相同
     * @param other 另一个点
     * @return 如果两点坐标相同返回true，否则返回false
     */
    bool operator==(const SqDot& other) const;
    
    /**
     * @brief 重载不等于运算符，判断两个点是否不同
     * @param other 另一个点
     * @return 如果两点坐标不同返回true，否则返回false
     */
    bool operator!=(const SqDot& other) const;
    
    // 注意：operator< 已移除，避免对点的自然排序产生依赖
};

/**
 * @brief 二维地图结构类
 * 表示包含地形代价信息的二维地图结构
 */
class SqPlain { 
public:
    std::vector<std::vector<double>> map;  ///< 地图数据，二维向量

    /**
     * @brief 构造函数，创建一个图结构
     * @param map 二维向量表示的地图数据
     */
    SqPlain(std::vector<std::vector<double>> map);

    /**
     * @brief 检查指定点是否在地图范围内且不是障碍物
     * @param point 待检查的点
     * @return 如果点在地图范围内且不是障碍物返回true，否则返回false
     */
    bool edge_allowed(SqDot point);

    /**
     * @brief 获取指定点的邻居节点
     * @param point 当前点
     * @param idx 邻居节点索引(0-3)，分别代表上、下、左、右四个方向
     * @return 邻居点的坐标
     */
    SqDot get_neighbour(SqDot point, int idx);
    
    /**
     * @brief 使用A*算法查找从起点到终点的路径
     * @param start 起始点
     * @param goal 目标点
     * @return 包含路径点坐标的数组，格式为{{x1,x2,...}, {y1,y2,...}}
     *         如果未找到路径，则返回两个空向量
     */
    std::vector<SqDot> find_path(SqDot start, SqDot goal);
    
    /**
     * @brief 将地图按指定比例缩放
     * @param scale 缩放比例因子
     * @return 缩放后的SqPlain对象
     */
    SqPlain scale_graph(double scale);

    /**
     * @brief 计算指定区域内的最小代价值
     * @param center 区域中心点
     * @param side_length 区域边长
     * @return 区域内的最小代价值
     */
    double summary(SqDot& center, int side_length);
};

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

    /**
     * @brief 计算点的滑动向量
     * @return 表示滑动方向和大小的SqDot对象
     * 
     * @details 该方法基于法线向量计算滑动向量，使得：
     *          1. z轴偏角越大，滑动向量的绝对值越大
     *          2. 滑动向量的方向与法线在xy平面上的投影一致
     *          3. 当法线垂直时（水平面），滑动向量长度为0
     *          4. 当法线水平时（垂直面），滑动向量长度为1
     */
    SqDot slide();
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
     * @brief 计算点到平面的距离
     * @param dot 待计算距离的点
     * @return 点到平面的距离（绝对值）
     */
    double distance(const CuDot& dot) const;
    
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