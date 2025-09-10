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
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>

#include "utils/scale.hpp"

/**
 * @brief 计算两点间的曼哈顿距离
 * 
 * @param a 第一个点
 * @param b 第二个点
 * @return 曼哈顿距离值
 */
double manhattan_distance(const SqDot& a, const SqDot& b);

/**
 * @brief 计算两点间的欧几里得距离
 * 
 * @param a 第一个点
 * @param b 第二个点
 * @return 欧几里得距离值
 */
double euclidean_distance(const SqDot& a, const SqDot& b);

/**
 * @brief 位置枚举，表示点相对于平面的位置关系
 */
enum class CuPos { Above, Below, Inside };

/**
 * @brief 二维整数点的哈希函数
 * 用于在哈希表中存储SqDot对象
 */
struct SqDotHash { 
    
    
    std::size_t operator()(const SqDot& p) const;
};

/**
 * @brief 以origin为原点旋转input点
 * 
 * @param origin 原点
 * @param input 输入点
 * @param angle 旋转角度（弧度）
 * @return 旋转后的点
 */
SqDot rotate_dot(const SqDot& origin, const SqDot& input, double angle);

/**
 * @brief 二维坐标点类
 * 用于表示地图上的坐标点
 */
class SqDot { 
public:
    /**
     * @brief x坐标
     */
    double x;

    /**
     * @brief y坐标
     */
    double y;

    
    /**
     * @brief 构造函数，创建一个二维坐标点
     * 
     * @param x x坐标
     * @param y y坐标
     */
    SqDot(double x=0.0, double y=0.0);
    
    /**
     * @brief 构造函数，从整数坐标创建点
     * 
     * @param x x坐标
     * @param y y坐标
     */
    SqDot(int x, int y);

    void set(double x, double y);
    
    /**
     * @brief 获取x坐标的整数索引（四舍五入）
     * 
     * @return x坐标的整数索引
     */
    int x_index() const;
    
    /**
     * @brief 获取y坐标的整数索引（四舍五入）
     * 
     * @return y坐标的整数索引
     */
    int y_index() const;
    
    /**
     * @brief 按比例缩放坐标点
     * 
     * @param scale 缩放比例
     * @return 缩放后的坐标点
     */
    SqDot scale(double scale) const;
    
    SqDot up_rotate(const double& angle) const;

    SqDot down_rotate(const double& angle) const;

    SqDot up_rot_round(const double& angle, const SqDot& center) const;

    SqDot down_rot_round(const double& angle, const SqDot& center) const;

    /**
     * @brief 将缩放后的坐标点恢复到原始地图的中心点
     * 
     * @param scale 缩放比例
     * @return 原始地图上的中心点坐标
     */
    SqDot central_restore(const double& scale) const;
    
    
    /**
     * @brief 判断两个点是否相等
     * 
     * @param other 另一个点
     * @return 如果两点坐标相同则返回true，否则返回false
     */
    bool operator==(const SqDot& other) const;
    
    
    /**
     * @brief 判断两个点是否不相等
     * 
     * @param other 另一个点
     * @return 如果两点坐标不同则返回true，否则返回false
     */
    bool operator!=(const SqDot& other) const;

    
    /**
     * @brief 比较两个点的大小（用于排序）
     * 
     * @param other 另一个点
     * @return 比较结果
     */
    bool operator<(const SqDot& other) const;

    SqDot operator+(const SqDot& other) const;

    SqDot operator-(const SqDot& other) const;
    
    /**
     * @brief 计算x坐标在缩放后的索引
     * 
     * @param scale 缩放比例
     * @return 缩放后的索引
     */
    int sx(double scale) const;
    
    
    /**
     * @brief 计算y坐标在缩放后的索引
     * 
     * @param scale 缩放比例
     * @return 缩放后的索引
     */
    int sy(double scale) const;
    
    
    /**
     * @brief 按比例缩放坐标点（引用版本）
     * 
     * @param scale 缩放比例
     * @return 缩放后的坐标点
     */
    SqDot scale(double scale);

    
    /**
     * @brief 计算两个点的中心点
     * 
     * @param other 另一个点
     * @return 两个点的中心点
     */
    SqDot center(const SqDot& other);

    
    /**
     * @brief 获取指定方向的邻居点
     * 
     * @param index 方向索引（0-7）
     * @return 指定方向的邻居点
     */
    SqDot get_neighbour(int index) const;

    
    /**
     * @brief 获取所有邻居点
     * 
     * @return 所有邻居点的向量
     */
    std::vector<SqDot> get_neighbour() const;

    
    /**
     * @brief 获取指定范围内的邻居点
     * 
     * @param x_ceil x方向上限
     * @param y_ceil y方向上限
     * @return 指定范围内的邻居点
     */
    std::vector<SqDot> get_neighbour(int x_ceil, int y_ceil) const;

    
    /**
     * @brief 计算到另一个点的距离
     * 
     * @param other 另一个点
     * @return 到另一个点的距离
     */
    double distance(const SqDot& other) const;

    /**
     * @brief 计算从当前点到另一点的向量角度
     * 
     * @param other 另一个点
     * @return 从当前点到另一点的向量与x轴正方向的夹角（弧度）
     */
    double angle(const SqDot& other) const;
    
    /**
     * @brief 计算两点连线的绝对角度
     * 
     * @param other 另一个点
     * @return 两点连线与x轴正方向的夹角（弧度）
     */
    double line_angle(const SqDot& other) const;

};

class SqLine {
public:
    double a;
    double b;
    double c;

    SqLine();
    SqLine(double a, double b, double c);
    SqLine(const SqDot& p1, const SqDot& p2);
    SqLine(const SqDot& point, double angle);

    double distance(const SqDot& dot) const;
};

/**
 * @brief 二维平面类
 * 用于表示和操作二维地图数据
 */
class SqPlain { 
public:
    /**
     * @brief 地图数据
     * 存储二维地图的高度或其他属性数据
     */
    std::vector<std::vector<double>> map;

    
    /**
     * @brief 构造函数，从给定的地图数据创建平面
     * 
     * @param map 地图数据
     */
    SqPlain(std::vector<std::vector<double>> map);

    
    /**
     * @brief 构造函数，创建指定大小的平面
     * 
     * @param rows 行数
     * @param cols 列数
     * @param value 初始值
     */
    SqPlain(int rows, int cols, double value=0.0);

    
    /**
     * @brief 默认构造函数
     */
    SqPlain();

    
    /**
     * @brief 获取地图边界内的点
     * 
     * @param point 原始点
     * @return 边界内的点
     */
    SqDot orth_near(const SqDot& point) const;

    
    /**
     * @brief 计算两点定义区域的中心点
     * 
     * @param fi 第一个点
     * @param se 第二个点
     * @return 区域的中心点
     */
    SqDot local_center(SqDot& fi, SqDot& se) const;

    
    /**
     * @brief 检查指定点是否允许边缘通过
     * 
     * @param point 检查点
     * @return 如果允许边缘通过返回true，否则返回false
     */
    bool edge_allowed(const SqDot& point) const;

    
    /**
     * @brief 获取指定点的邻居点
     * 
     * @param point 指定点
     * @param idx 邻居点索引
     * @return 邻居点
     */
    SqDot get_neighbour(const SqDot& point, int idx) const;

    
    /**
     * @brief 获取指定点的所有邻居点
     * 
     * @param point 指定点
     * @return 所有邻居点
     */
    std::vector<SqDot> get_neighbour(const SqDot& point) const;
    
    
    /**
     * @brief 获取指定点的有效邻居点
     * 
     * @param point 指定点
     * @return 有效邻居点
     */
    std::vector<SqDot> get_valid_neighbours(const SqDot& point) const;
    
    
    /**
     * @brief 使用A*算法查找从起点到终点的路径
     * 
     * @param start 起点
     * @param goal 终点
     * @return 路径点序列
     */
    std::vector<SqDot> find_path(SqDot start, SqDot goal) const;
    
    
    /**
     * @brief 缩放地图
     * 
     * @param scale 缩放比例
     * @return 缩放后的地图
     */
    SqPlain scale_graph(const double& scale) const;

    
    /**
     * @brief 缩放地图并计算方差
     * 
     * @param scale 缩放比例
     * @return 缩放后的地图方差
     */
    SqPlain scale_graph_variance(double scale) const;
    
    
    /**
     * @brief 计算指定区域的汇总值
     * 
     * @param center 中心点
     * @param side_length 边长
     * @return 汇总值
     */
    double summary(SqDot& center, int side_length) const;

    
    /**
     * @brief 检查地图是否为空
     * 
     * @return 如果地图为空返回true，否则返回false
     */
    bool empty() const;
    
    
    /**
     * @brief 获取地图行数
     * 
     * @return 地图行数
     */
    int rows() const;

    
    /**
     * @brief 计算缩放后的行数
     * 
     * @param scale 缩放比例
     * @return 缩放后的行数
     */
    int row_scale(double& scale) const;

    
    /**
     * @brief 获取地图列数
     * 
     * @return 地图列数
     */
    int cols() const;

    
    /**
     * @brief 计算缩放后的列数
     * 
     * @param scale 缩放比例
     * @return 缩放后的列数
     */
    int col_scale(double& scale) const;

    
    /**
     * @brief 重载下标运算符，用于访问指定行
     * 
     * @param index 行索引
     * @return 指定行的引用
     */
    std::vector<double>& operator[](int index);
    
    
    /**
     * @brief 重载下标运算符，用于访问指定行（const版本）
     * 
     * @param index 行索引
     * @return 指定行的const引用
     */
    const std::vector<double>& operator[](int index) const;

    
    /**
     * @brief 计算从一个点到另一个点的代价
     * 
     * @param at 起点
     * @param to 终点
     * @return 移动代价
     */
    double cost(const SqDot& at, const SqDot& to) const;

    
    /**
     * @brief 恢复缩放点到原始地图坐标
     * 
     * @param dot 缩放点
     * @param scale 缩放比例
     * @return 原始地图坐标点
     */
    SqDot restore_dot(SqDot& dot, double scale) const;

    
    /**
     * @brief 恢复缩放块到原始地图坐标
     * 
     * @param block 缩放块
     * @param scale 缩放比例
     * @return 原始地图坐标块
     */
    std::pair<SqDot, SqDot> restore(const SqDot& block, double scale) const;

    
    /**
     * @brief 检查两个点是否在同一个缩放块中
     * 
     * @param a 第一个点
     * @param b 第二个点
     * @param scale 缩放比例
     * @return 如果在同一个块中返回true，否则返回false
     */
    bool in_same_block(const SqDot& a, const SqDot& b, double scale) const;
};

/**
 * @brief 三维坐标点类
 * 用于表示三维空间中的点
 */
class CuDot {
public:
    /**
     * @brief x坐标
     */
    double x;

    /**
     * @brief y坐标
     */
    double y;

    /**
     * @brief z坐标
     */
    double z;

    
    /**
     * @brief 构造函数，创建一个三维坐标点
     * 
     * @param x x坐标
     * @param y y坐标
     * @param z z坐标
     */
    CuDot(double x=0.0, double y=0.0, double z=0.0);
    
    
    /**
     * @brief 重载加法运算符
     * 
     * @param dot 另一个点
     * @return 两个点的和
     */
    CuDot operator+(const CuDot& dot) const;

    
    /**
     * @brief 投影到二维平面
     * 
     * @return 投影后的二维点
     */
    SqDot slide();
};

/**
 * @brief 三维直线类
 * 用于表示三维空间中的直线
 */
class CuLine {
public: 
    /**
     * @brief 直线上的点
     */
    CuDot point;

    
    /**
     * @brief 构造函数，创建一条直线
     * 
     * @param point 直线上的点
     */
    CuLine(const CuDot& point=CuDot());

    
    /**
     * @brief 计算偏移量
     * 
     * @param dot 偏移点
     * @return 偏移后的点
     */
    CuDot offset(const CuDot& dot);

    
    /**
     * @brief 获取直线上的点
     * 
     * @return 直线上的点
     */
    CuDot get();

    
    /**
     * @brief 计算与另一条直线的法向量
     * 
     * @param other 另一条直线
     * @return 法向量
     */
    CuLine normal_vector(const CuLine& other) const;
};

/**
 * @brief 三维平面类
 * 用于表示三维空间中的平面
 */
class CuPlain {
public:
    /**
     * @brief 平面方程系数A
     */
    double A;

    /**
     * @brief 平面方程系数B
     */
    double B;

    /**
     * @brief 平面方程系数C
     */
    double C;

    /**
     * @brief 平面方程系数D
     */
    double D;

    /**
     * @brief 定义扩展
     */
    int define_extend;

    
    /**
     * @brief 构造函数，创建一个三维平面
     * 
     * @param A 平面方程系数A
     * @param B 平面方程系数B
     * @param C 平面方程系数C
     * @param D 平面方程系数D
     */
    CuPlain(double A=0.0, double B=0.0, double C=0.0, double D=0.0);
    
    
    /**
     * @brief 通过三个点定义平面
     * 
     * @param dot 三个点
     * @return 如果定义成功返回true，否则返回false
     */
    bool define_plaine(const std::array<CuDot, 3>& dot);
    
    
    /**
     * @brief 获取点相对于平面的位置
     * 
     * @param dot 检查点
     * @return 点相对于平面的位置
     */
    CuPos get_pos(const CuDot& dot) const;
    
    
    /**
     * @brief 计算点到平面的距离
     * 
     * @param dot 检查点
     * @return 点到平面的距离
     */
    double distance(const CuDot& dot) const;
    
    
    /**
     * @brief 计算平面的法向量
     * 
     * @return 平面的法向量
     */
    CuDot normal_vector() const;
    
    
    /**
     * @brief 计算平面法向量与z轴的夹角
     * 
     * @return 法向量与z轴的夹角（弧度）
     */
    double normal_angle() const;
};

#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif