#include "utils/geometry.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <functional>
#include <limits>

/**
 * @brief 构造函数，创建一个点
 * @param x 点的x坐标，默认为0
 * @param y 点的y坐标，默认为0
 */
SqDot::SqDot(int x, int y): x(x), y(y) {
    return;
}

/**
 * @brief 重载相等运算符，判断两个点是否相同
 * @param other 另一个点
 * @return 如果两点坐标相同返回true，否则返回false
 */
bool SqDot::operator==(const SqDot& other) const {
    return x == other.x && y == other.y;
}

/**
 * @brief 重载不等于运算符，判断两个点是否不同
 * @param other 另一个点
 * @return 如果两点坐标不同返回true，否则返回false
 */
bool SqDot::operator!=(const SqDot& other) const {
    return !(*this == other);
}


/**
 * @brief 计算SqDot对象的哈希值
 * @param p 待计算哈希值的点
 * @return 点的哈希值
 */
std::size_t SqDotHash::operator()(const SqDot& p) const {
    // 使用标准哈希函数计算哈希值
    // 将x和y坐标组合成一个唯一的哈希值
    return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
}

/**
 * @brief 构造函数，创建一个图结构
 * @param map 二维向量表示的地图数据
 */
SqPlain::SqPlain(std::vector<std::vector<double>> map): map(std::move(map)) {
    return;
}

/**
 * @brief 检查指定点是否在地图范围内且不是障碍物
 * @param point 待检查的点
 * @return 如果点在地图范围内且不是障碍物返回true，否则返回false
 */
bool SqPlain::edge_allowed(SqDot point) {
    // 检查点是否在地图范围内
    if (point.x < 0 || point.x >= map.size() || point.y < 0 || point.y >= map[0].size()) {
        return false;
    }
    
    // 检查点是否为障碍物（代价为无穷大）
    if (map[point.x][point.y] == std::numeric_limits<double>::infinity()) {
        return false;
    }
    
    return true;
}

/**
 * @brief 获取指定点的邻居节点
 * @param point 当前点
 * @param idx 邻居节点索引(0-3)，分别代表上、下、左、右四个方向
 * @return 邻居点的坐标
 */
SqDot SqPlain::get_neighbour(SqDot point, int idx) {
    // 定义四个方向的偏移量：上、下、左、右
    static const std::array<int, 4> dx = {-1, 1, 0, 0};
    static const std::array<int, 4> dy = {0, 0, -1, 1};
    
    // 计算邻居点坐标
    return SqDot(point.x + dx[idx], point.y + dy[idx]);
}

/**
 * @brief 使用A*算法查找从起点到终点的路径
 * @param start 起始点
 * @param goal 目标点
 * @return 包含路径点坐标的数组，格式为{{x1,x2,...}, {y1,y2,...}}
 *         如果未找到路径，则返回两个空向量
 */
std::vector<SqDot> SqPlain::find_path(SqDot start, SqDot goal) {
    // 优先队列，用于存储待访问的节点，按照f_score排序（小顶堆）
    std::priority_queue<std::pair<double, SqDot>, 
                        std::vector<std::pair<double, SqDot>>, 
                        std::greater<std::pair<double, SqDot>>> open_set;
    
    // 存储每个节点的g_score（从起点到当前节点的实际代价）
    std::unordered_map<SqDot, double, SqDotHash> g_score;
    
    // 存储每个节点的f_score（g_score + h_score，即预估总代价）
    std::unordered_map<SqDot, double, SqDotHash> f_score;
    
    // 存储每个节点的前驱节点，用于路径重建
    std::unordered_map<SqDot, SqDot, SqDotHash> came_from;
    
    // 记录已经访问过的节点
    std::unordered_set<SqDot, SqDotHash> closed_set;
    
    // 初始化起点
    g_score[start] = 0;
    f_score[start] = std::sqrt(std::pow(start.x - goal.x, 2) + std::pow(start.y - goal.y, 2));
    open_set.push({f_score[start], start});
    
    // 主循环
    while (!open_set.empty()) {
        // 取出f_score最小的节点
        SqDot current = open_set.top().second;
        open_set.pop();
        
        // 如果到达终点，重建路径并返回
        if (current == goal) {
            std::vector<SqDot> path;
            SqDot node = current;
            while (came_from.find(node) != came_from.end()) {
                path.push_back(node);
                node = came_from[node];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // 将当前节点加入已访问集合
        closed_set.insert(current);
        
        // 遍历当前节点的所有邻居
        for (int i = 0; i < 4; i++) {
            SqDot neighbor = get_neighbour(current, i);
            
            // 检查邻居是否可通行
            if (!edge_allowed(neighbor)) {
                continue;
            }
            
            // 如果邻居已经访问过，跳过
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            // 计算从起点到邻居的代价
            double tentative_g_score = g_score[current] + map[neighbor.x][neighbor.y];
            
            // 如果找到了更优的路径，或者首次访问该邻居
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + std::sqrt(std::pow(neighbor.x - goal.x, 2) + std::pow(neighbor.y - goal.y, 2));
                
                // 将邻居加入待访问队列
                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }
    
    // 未找到路径，返回空vector
    return std::vector<SqDot>();
}

/**
 * @brief 将地图按指定比例缩放
 * @param scale 缩放比例因子
 * @return 缩放后的SqPlain对象
 */
SqPlain SqPlain::scale_graph(double scale) {
    // 计算缩放后的地图尺寸
    int new_rows = static_cast<int>(std::ceil(map.size() / scale));
    int new_cols = static_cast<int>(std::ceil(map[0].size() / scale));
    
    // 创建新的地图
    std::vector<std::vector<double>> new_map(new_rows, std::vector<double>(new_cols, std::numeric_limits<double>::infinity()));
    
    // 对每个新的单元格计算代价值
    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {
            // 计算在原地图中对应的区域
            int start_row = static_cast<int>(i * scale);
            int end_row = static_cast<int>(std::min((i + 1) * scale, static_cast<double>(map.size())));
            int start_col = static_cast<int>(j * scale);
            int end_col = static_cast<int>(std::min((j + 1) * scale, static_cast<double>(map[0].size())));
            
            // 在区域内寻找最小代价值
            double min_cost = std::numeric_limits<double>::infinity();
            for (int row = start_row; row < end_row; row++) {
                for (int col = start_col; col < end_col; col++) {
                    if (map[row][col] < min_cost) {
                        min_cost = map[row][col];
                    }
                }
            }
            
            // 将最小代价值赋给新的单元格
            new_map[i][j] = min_cost;
        }
    }
    
    return SqPlain(std::move(new_map));
}

double SqPlain::summary(SqDot& center, int side_length) {
    // 计算区域边界
    int half_side = side_length / 2;
    int start_row = std::max(0, center.x - half_side);
    int end_row = std::min(static_cast<int>(map.size()) - 1, center.x + half_side);
    int start_col = std::max(0, center.y - half_side);
    int end_col = std::min(static_cast<int>(map[0].size()) - 1, center.y + half_side);
    
    // 计算区域内的最小代价值
    double min_cost = std::numeric_limits<double>::infinity();
    for (int i = start_row; i <= end_row; i++) {
        for (int j = start_col; j <= end_col; j++) {
            if (map[i][j] < min_cost) {
                min_cost = map[i][j];
            }
        }
    }
    
    return min_cost;
}

/**
 * @brief 构造函数，创建一个点
 * @param x 点的x坐标，默认为0.0
 * @param y 点的y坐标，默认为0.0
 * @param z 点的z坐标，默认为0.0
 */
CuDot::CuDot(double x, double y, double z): x(x), y(y), z(z) {
    return;
}

/**
 * @brief 重载加法运算符，实现两个点坐标的相加
 * @param dot 另一个点
 * @return 两个点坐标相加后的新点
 */
CuDot CuDot::operator+(const CuDot& dot) const {
    return CuDot(x + dot.x, y + dot.y, z + dot.z);
}

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
SqDot CuDot::slide() {
    // 计算法线向量的大小
    double magnitude = sqrt(x*x + y*y + z*z);
    
    // 如果法线向量大小为0，返回零向量
    if (magnitude == 0) {
        return SqDot(0, 0);
    }
    
    // 计算法线向量在xy平面上的投影长度
    double xy_magnitude = sqrt(x*x + y*y);
    
    // 计算滑动向量的大小
    // 当xy_magnitude为0时（法线垂直），滑动向量大小为0
    // 当xy_magnitude为1时（法线水平），滑动向量大小为1
    double slide_magnitude = xy_magnitude;
    
    // 如果投影长度为0，返回零向量
    if (xy_magnitude == 0) {
        return SqDot(0, 0);
    }
    
    // 计算单位投影向量
    double unit_x = x / xy_magnitude;
    double unit_y = y / xy_magnitude;
    
    // 计算滑动向量
    int slide_x = static_cast<int>(round(unit_x * slide_magnitude));
    int slide_y = static_cast<int>(round(unit_y * slide_magnitude));
    
    return SqDot(slide_x, slide_y);
}

/**
 * @brief 构造函数，创建一条直线
 * @param point 直线上的点，默认为原点
 */
CuLine::CuLine(const CuDot& point): point(point) {
    return;
}

/**
 * @brief 计算点相对于直线的偏移位置
 * @param dot 参考点
 * @return 偏移后的新点
 */
CuDot CuLine::offset(const CuDot& dot) {
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
CuLine CuLine::normal_vector(const CuLine& other) const {
    // 计算叉积得到法向量
    // 叉积公式：a × b = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
    CuDot result(point.y * other.point.z - point.z * other.point.y,
                 point.z * other.point.x - point.x * other.point.z,
                 point.x * other.point.y - point.y * other.point.x);
    return CuLine(result);
}

/**
 * @brief 构造函数，创建一个平面
 * @param A 平面方程系数A，默认为0.0
 * @param B 平面方程系数B，默认为0.0
 * @param C 平面方程系数C，默认为0.0
 * @param D 平面方程系数D，默认为0.0
 */
CuPlain::CuPlain(double A, double B, double C, double D): A(A), B(B), C(C), D(D), define_extend(0) {
    return;
}

/**
 * @brief 通过三个点定义一个平面
 * @param dot 包含三个点的数组
 * @return 如果成功定义平面返回true，如果三点共线返回false
 */
bool CuPlain::define_plaine(const std::array<CuDot, 3>& dot) {
    // 计算两个向量
    CuDot v1(dot[1].x - dot[0].x, dot[1].y - dot[0].y, dot[1].z - dot[0].z);
    CuDot v2(dot[2].x - dot[0].x, dot[2].y - dot[0].y, dot[2].z - dot[0].z);
    
    // 计算叉积得到法向量
    CuDot normal(v1.y * v2.z - v1.z * v2.y,
                 v1.z * v2.x - v1.x * v2.z,
                 v1.x * v2.y - v1.y * v2.x);
    
    // 检查法向量是否为零向量（三点共线）
    if (normal.x == 0 && normal.y == 0 && normal.z == 0) {
        return false;
    }
    
    // 设置平面方程系数
    A = normal.x;
    B = normal.y;
    C = normal.z;
    D = -(A * dot[0].x + B * dot[0].y + C * dot[0].z);
    
    return true;
}

/**
 * @brief 判断点相对于平面的位置关系
 * @param dot 待判断的点
 * @return 返回点相对于平面的位置枚举值
 */
CuPos CuPlain::get_pos(const CuDot& dot) const {
    // 计算点到平面的距离
    double distance = A * dot.x + B * dot.y + C * dot.z + D;
    
    // 根据距离判断点的位置
    if (distance > 1e-9) {  // 考虑浮点数精度误差
        return CuPos::Above;
    } else if (distance < -1e-9) {
        return CuPos::Below;
    } else {
        return CuPos::Inside;
    }
}

/**
 * @brief 计算点到平面的距离
 * @param dot 待计算距离的点
 * @return 点到平面的距离（绝对值）
 */
double CuPlain::distance(const CuDot& dot) const {
    // 使用点到平面距离公式: |Ax + By + Cz + D| / sqrt(A² + B² + C²)
    double numerator = std::abs(A * dot.x + B * dot.y + C * dot.z + D);
    double denominator = std::sqrt(A * A + B * B + C * C);
    
    // 避免除零错误
    if (denominator < 1e-9) {
        return 0.0;
    }
    
    return numerator / denominator;
}

/**
 * @brief 获取平面的法向量
 * @return 表示平面法向量的CuDot对象
 */
CuDot CuPlain::normal_vector() const {
    return CuDot(A, B, C);
}

/**
 * @brief 计算平面法向量与z轴的夹角
 * @return 平面法向量与z轴的夹角（弧度）
 */
double CuPlain::normal_angle() const {
    // 计算法向量的大小
    double magnitude = sqrt(A*A + B*B + C*C);
    
    // 如果法向量大小为0，返回0
    if (magnitude == 0) {
        return 0.0;
    }
    
    // 计算与z轴(0,0,1)的夹角
    // 使用点积公式: cos(θ) = (a·b) / (|a|*|b|)
    // 其中b是z轴单位向量(0,0,1)
    double cos_theta = C / magnitude;
    
    // 确保cos_theta在有效范围内
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    
    // 计算夹角
    double bottom = sqrt(A*A + B*B);
    return atan2(bottom, C);
}