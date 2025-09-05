#include "utils/geometry.hpp"
#include "aStar/graph.hpp"
#include <cmath>

/**
 * @brief 平面类构造函数
 * @param A 平面方程Ax + By + Cz + D = 0中的系数A
 * @param B 平面方程Ax + By + Cz + D = 0中的系数B
 * @param C 平面方程Ax + By + Cz + D = 0中的系数C
 * @param D 平面方程Ax + By + Cz + D = 0中的系数D
 */
CuPlain::CuPlain(double A, double B, double C, double D)
    : A(A), B(B), C(C), D(D), define_extend(0) {}

/**
 * @brief 通过三个点定义一个平面
 * @param dot 包含三个点的数组，每个点由(x,y,z)坐标表示
 * @return 如果成功定义平面返回true，如果三点共线返回false
 * 
 * @details 该函数使用向量叉积的方法计算通过三个点的平面方程系数。
 *          首先从三个点构造两个向量，然后计算这两个向量的叉积得到平面的法向量，
 *          最后利用点法式方程构造平面方程。
 * 
 * @note 平面方程的标准形式为 Ax + By + Cz + D = 0，其中(A,B,C)是平面的法向量。
 * 
 * @note 算法步骤：
 *       1. 从输入的三个点 P1(x1,y1,z1), P2(x2,y2,z2), P3(x3,y3,z3) 构造两个向量：
 *          V1 = P2 - P1 = (x2-x1, y2-y1, z2-z1)
 *          V2 = P3 - P1 = (x3-x1, y3-y1, z3-z1)
 *       2. 计算法向量 N = V1 × V2 = (nx, ny, nz)：
 *          nx = (y2-y1)(z3-z1) - (z2-z1)(y3-y1)
 *          ny = (z2-z1)(x3-x1) - (x2-x1)(z3-z1)
 *          nz = (x2-x1)(y3-y1) - (y2-y1)(x3-x1)
 *       3. 检查三点是否共线（法向量为零向量）
 *       4. 构造平面方程：A=nx, B=ny, C=nz, D=-(nx*x1 + ny*y1 + nz*z1)
 */
bool CuPlain::define_plaine(const std::array<CuDot, 3>& dot) {
    // 获取三个点的坐标
    double x1 = dot.at(0).x, y1 = dot.at(0).y, z1 = dot.at(0).z;
    double x2 = dot.at(1).x, y2 = dot.at(1).y, z2 = dot.at(1).z;
    double x3 = dot.at(2).x, y3 = dot.at(2).y, z3 = dot.at(2).z;
    
    // 计算两个向量
    double v1x = x2 - x1, v1y = y2 - y1, v1z = z2 - z1;
    double v2x = x3 - x1, v2y = y3 - y1, v2z = z3 - z1;
    
    // 计算法向量（两个向量的叉积）
    double nx = v1y * v2z - v1z * v2y;
    double ny = v1z * v2x - v1x * v2z;
    double nz = v1x * v2y - v1y * v2x;
    
    // 检查三点是否共线（法向量为零向量）
    if (nx == 0 && ny == 0 && nz == 0) {
        return false;
    }
    
    // 平面方程为: A(x-x1) + B(y-y1) + C(z-z1) = 0
    // 展开得到: Ax + By + Cz - (Ax1 + By1 + Cz1) = 0
    // 所以: A=nx, B=ny, C=nz, D=-(nx*x1 + ny*y1 + nz*z1)
    A = nx;
    B = ny;
    C = nz;
    D = -(nx * x1 + ny * y1 + nz * z1);
    
    return true;
}

/**
 * @brief 判断点相对于平面的位置关系
 * @param dot 待判断的点
 * @return 返回点相对于平面的位置枚举值：
 *         CuPos::Above - 点在平面法向量指向的一侧
 *         CuPos::Below - 点在平面法向量相反的一侧
 *         CuPos::Inside - 点在平面上
 */
CuPos CuPlain::get_pos(const CuDot& dot) const {
    double x = dot.x, y = dot.y, z = dot.z;
    double t = -(A * x + B * y + C * z + D) / (A * A + B * B + C * C);
    if (t > 0) {
        return CuPos::Above;
    } else if (t < 0) {
        return CuPos::Below;
    } else {
        return CuPos::Inside;
    }
}

/**
 * @brief 获取平面的法向量
 * @return 表示平面法向量的CuDot对象
 */
CuDot CuPlain::normal_vector() const {
    return CuDot(A, B, C);
}

/**
 * @brief 直线类构造函数
 * @param point 直线上的一点
 */
CuLine::CuLine(const CuDot &point): point(point) {}

/**
 * @brief 计算点相对于直线的偏移位置
 * @param dot 参考点
 * @return 偏移后的新点
 */
CuDot CuLine::offset(const CuDot &dot) {
    return point + dot;
}

/**
 * @brief 获取直线上的点
 * @return 直线上的点
 */
CuDot CuLine::get() {
    return point;
}

/**
 * @brief 计算两条直线的法向量（叉积）
 * @param other 另一条直线
 * @return 表示法向量的CuLine对象
 */
CuLine CuLine::normal_vector(const CuLine &other) const {
    CuDot normal_point(
        point.x*other.point.y   -   point.y*other.point.x,
        - point.z*other.point.x +   point.x*other.point.z,
        point.y*other.point.z   -   point.z*other.point.y
    );
    return CuLine(normal_point);
}

/**
 * @brief 计算平面法向量与z轴的夹角
 * @return 平面法向量与z轴的夹角（弧度）
 */
double CuPlain::normal_angle() const {
    double bottom = sqrt(A*A + B*B);
    return atan2(bottom, C);
}

/**
 * @brief 点类构造函数
 * @param x 点的x坐标
 * @param y 点的y坐标
 * @param z 点的z坐标
 */
CuDot::CuDot(double x, double y, double z) : x(x), y(y), z(z) {}

/**
 * @brief 重载加法运算符，实现两个点的坐标相加
 * @param other 另一个点
 * @return 两个点坐标相加后的新点
 */
CuDot CuDot::operator+(const CuDot &other) const {
    return CuDot(x + other.x, y + other.y, z + other.z);
}

/**
 * @brief 计算点的滑动向量
 * @return 表示滑动方向和大小的Point对象
 * 
 * @details 该方法基于法线向量计算滑动向量，使得：
 *          1. z轴偏角越大，滑动向量的绝对值越大
 *          2. 滑动向量的方向与法线在xy平面上的投影一致
 *          3. 当法线垂直时（水平面），滑动向量长度为0
 *          4. 当法线水平时（垂直面），滑动向量长度为1
 */
Point CuDot::slide() {
    // 计算法线向量的大小
    double normal_magnitude = sqrt(x*x + y*y + z*z);
    
    // 避免除零错误
    if (normal_magnitude == 0) {
        return Point(0, 0);
    }
    
    // 计算法线向量与z轴的夹角余弦值
    double cos_theta = std::abs(z) / normal_magnitude;
    
    // 计算倾斜程度：当完全垂直时为0，当水平时为1
    // 使用sin(theta)表示倾斜程度，因为sin(0°)=0, sin(90°)=1
    double tilt_factor = sqrt(1 - cos_theta * cos_theta); // sin(theta)
    
    // 计算法线在xy平面上的投影
    double proj_x = x;
    double proj_y = y;
    
    // 投影向量的长度
    double proj_magnitude = sqrt(proj_x * proj_x + proj_y * proj_y);
    
    // 如果投影长度为0，说明法线垂直向上，无需滑动
    if (proj_magnitude == 0) {
        return Point(0, 0);
    }
    
    // 归一化投影向量
    double unit_proj_x = proj_x / proj_magnitude;
    double unit_proj_y = proj_y / proj_magnitude;
    
    // 计算滑动向量：方向与投影方向一致，大小与倾斜程度成正比
    int slide_x = static_cast<int>(round(unit_proj_x * tilt_factor));
    int slide_y = static_cast<int>(round(unit_proj_y * tilt_factor));
    
    return Point(slide_x, slide_y);
}