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
     * 
     * @details 此函数用于在基于哈希的容器中使用SqDot对象，
     *          如std::unordered_map和std::unordered_set
     */
    std::size_t operator()(const SqDot& p) const;
};

/**
 * @brief 二维整数坐标点类
 * 表示一个具有x、y坐标的整数点，主要用于地图网格位置
 * 
 * @details SqDot类提供了点的基本操作，包括构造、比较、缩放等。
 *          该类支持在基于哈希的STL容器中使用，通过特化的SqDotHash结构实现。
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
     * @brief 按比例缩放点坐标
     * @param scale 缩放比例因子
     * @return 缩放后的点坐标
     * 
     * @details 该方法用于地图缩放操作，将点坐标按指定比例进行缩放。
     *          通常用于生成低分辨率的引导路径图。
     */
    SqDot scale(double scale) const;
    
    /**
     * @brief 将缩放后的点坐标还原到原始地图坐标
     * @param scale 缩放比例因子
     * @param unit_size 单元格大小
     * @return 还原后的点坐标
     * 
     * @details 该方法用于将低分辨率地图中的点坐标映射回原始高分辨率地图坐标。
     *          通常在路径规划完成后，将引导点转换为实际落足点时使用。
     */
    SqDot central_restore(double scale, double unit_size) const;
    
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
    
    /**
     * @brief 重载小于运算符，用于优先队列排序
     * @param other 另一个点
     * @return 如果当前点小于另一个点返回true，否则返回false
     */
    bool operator<(const SqDot& other) const;
};

/**
 * @brief 二维地图结构类
 * 表示包含地形代价信息的二维地图结构
 * 
 * @details SqPlain类封装了二维地图数据和相关操作方法。
 *          地图数据以二维向量形式存储，每个单元格的值表示该位置的通行代价。
 *          负数值表示障碍物，非负数值表示可通行区域，数值越大表示通行代价越高。
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
     * @brief 默认构造函数，创建一个空的地图结构
     */
    SqPlain();

    /**
     * @brief 检查指定点是否在地图范围内且不是障碍物
     * @param point 待检查的点
     * @return 如果点在地图范围内且不是障碍物返回true，否则返回false
     * 
     * @details 该方法检查点是否满足以下条件：
     *          1. 点的坐标在地图边界内
     *          2. 点对应的代价值非负（不是障碍物）
     */
    bool edge_allowed(SqDot point);

    /**
     * @brief 获取指定点的邻居节点
     * @param point 当前点
     * @param idx 邻居节点索引(0-3)，分别代表上、下、左、右四个方向
     * @return 邻居点的坐标
     * 
     * @details 邻居节点索引对应关系：
     *          - 0: 上方节点 (x-1, y)
     *          - 1: 下方节点 (x+1, y)
     *          - 2: 左侧节点 (x, y-1)
     *          - 3: 右侧节点 (x, y+1)
     */
    SqDot get_neighbour(SqDot point, int idx);
    
    /**
     * @brief 使用A*算法查找从起点到终点的路径
     * @param start 起始点
     * @param goal 目标点
     * @return 包含路径点坐标的数组，格式为{{x1,x2,...}, {y1,y2,...}}
     *         如果未找到路径，则返回两个空向量
     * 
     * @deprecated 此方法已废弃，建议使用独立的a_star_search函数
     */
    std::vector<SqDot> find_path(SqDot start, SqDot goal);
    
    /**
     * @brief 将地图按指定比例缩放
     * @param scale 缩放比例因子
     * @return 缩放后的SqPlain对象
     * 
     * @details 缩放方法通过采样区域内的最小代价值来生成缩放后的地图，
     *          确保缩放后的地图保持原始地图的可通行性特征。
     */
    SqPlain scale_graph(double scale);

    /**
     * @brief 将地图缩放到指定比例，使用方差作为cost计算的一部分
     * @param scale 缩放因子
     * @return 缩放后的地图
     * 
     * @details 该方法不仅考虑区域内的平均代价值，还考虑方差。
     *          方差越大表示该区域内地形变化越大，对机器人行走越不利。
     *          缩放后的代价值计算公式为：mean + variance_weight * variance
     */
    SqPlain scale_graph_variance(double scale);
    
    /**
     * @brief 计算指定区域内的平整度评估值
     * @param center 区域中心点
     * @param side_length 区域边长
     * @return 区域的平整度评估值，值越小表示越平整
     * 
     * @details 该方法用于地图缩放操作，通过快速评估指定区域的平整度
     *          来决定缩放后地图中对应位置的代价值。
     */
    double summary(SqDot& center, int side_length);

    /**
     * @brief 检查地图是否为空
     * @return 如果地图为空返回true，否则返回false
     */
    bool empty() const;
    
    /**
     * @brief 获取地图行数
     * @return 地图行数
     */
    int rows() const;

    /**
     * @brief 获取地图列数
     * @return 地图列数
     */
    int cols() const;

    /**
     * @brief 重载下标运算符，用于访问指定行的数据
     * @param index 行索引
     * @return 指定行的引用
     */
    std::vector<double>& operator[](int index);
    
    /**
     * @brief 重载下标运算符(const版本)，用于访问指定行的数据
     * @param index 行索引
     * @return 指定行的const引用
     */
    const std::vector<double>& operator[](int index) const;
};

/**
 * @brief 三维空间中的点类
 * 表示一个具有x、y、z坐标的点
 * 
 * @details CuDot类用于表示三维空间中的点，主要用于地面点和法向量的表示。
 *          该类提供了基本的向量运算操作，如加法和滑动计算。
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
     * 
     * @details 该操作用于向量加法运算，将两个点的坐标分量分别相加。
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
     *          
     *          滑动向量用于足部稳定性优化，当足部放置区域不稳定时，
     *          根据滑动向量调整足部位置以提高稳定性。
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
 * 
 * @details CuPlain类用于表示三维空间中的平面，主要用于地面区域最优平面的计算。
 *          平面由方程Ax + By + Cz + D = 0定义，其中(A,B,C)是平面的法向量。
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
     * 
     * @details 该方法使用三个点计算平面方程的系数。
     *          通过向量叉积计算平面的法向量，然后根据法向量和其中一个点确定D系数。
     */
    bool define_plaine(const std::array<CuDot, 3>& dot);
    
    /**
     * @brief 判断点相对于平面的位置关系
     * @param dot 待判断的点
     * @return 返回点相对于平面的位置枚举值
     * 
     * @details 通过计算点到平面的有符号距离判断位置关系：
     *          - 如果距离>0，点在平面法向量指向的一侧
     *          - 如果距离<0，点在平面法向量相反的一侧
     *          - 如果距离=0，点在平面上
     */
    CuPos get_pos(const CuDot& dot) const;
    
    /**
     * @brief 计算点到平面的距离
     * @param dot 待计算距离的点
     * @return 点到平面的距离（绝对值）
     * 
     * @details 使用点到平面距离公式计算：|Ax + By + Cz + D| / √(A² + B² + C²)
     */
    double distance(const CuDot& dot) const;
    
    /**
     * @brief 获取平面的法向量
     * @return 表示平面法向量的CuDot对象
     * 
     * @details 法向量为(A, B, C)，表示平面方程Ax + By + Cz + D = 0中的系数。
     *          该向量垂直于平面，指向平面的正方向。
     */
    CuDot normal_vector() const;
    
    /**
     * @brief 计算平面法向量与z轴的夹角
     * @return 平面法向量与z轴的夹角（弧度）
     * 
     * @details 该角度用于评估地面区域的稳定性，夹角越小表示地面越平坦，
     *          足部放置越稳定。夹角为0表示水平面，夹角为π/2表示垂直面。
     */
    double normal_angle() const;
};

#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif