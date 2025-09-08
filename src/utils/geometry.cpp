#include "utils/geometry.hpp"

/**
 * @brief 计算两点间的曼哈顿距离
 * 
 * @param a 第一个点
 * @param b 第二个点
 * @return 曼哈顿距离值
 */
double manhattan_distance(SqDot a, SqDot b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/**
 * @brief 计算两点间的欧几里得距离
 * 
 * @param a 第一个点
 * @param b 第二个点
 * @return 欧几里得距离值
 */
double euclidean_distance(SqDot a, SqDot b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

/**
 * @brief 构造函数，创建一个二维整数坐标点
 * 
 * @param x x坐标
 * @param y y坐标
 */
SqDot::SqDot(int x, int y): x(x), y(y) {
    return;
}

/**
 * @brief 按比例缩放坐标点
 * 
 * @param scale 缩放比例
 * @return 缩放后的坐标点
 */
SqDot SqDot::scale(double scale) const {
    return SqDot(static_cast<int>(std::round(x * scale)), static_cast<int>(std::round(y * scale)));
}

/**
 * @brief 将缩放后的坐标点恢复到原始地图的中心点
 * 
 * @param scale 缩放比例
 * @return 原始地图上的中心点坐标
 */
SqDot SqDot::central_restore(const double& scale) const {
    auto offset = 0.5 / scale;
    return SqDot(static_cast<int>(std::round(x / scale + offset - 0.5)), 
                static_cast<int>(std::round(y / scale + offset - 0.5)));
}

/**
 * @brief 判断两个点是否相等
 * 
 * @param other 另一个点
 * @return 如果两点坐标相同则返回true，否则返回false
 */
bool SqDot::operator==(const SqDot& other) const {
    return x == other.x && y == other.y;
}

/**
 * @brief 判断两个点是否不相等
 * 
 * @param other 另一个点
 * @return 如果两点坐标不同则返回true，否则返回false
 */
bool SqDot::operator!=(const SqDot& other) const {
    return !(*this == other);
}

/**
 * @brief 比较两个点的大小（用于排序）
 * 
 * @param other 另一个点
 * @return 比较结果
 */
bool SqDot::operator<(const SqDot& other) const {
    return (x < other.x) || (x == other.x && y < other.y);
}

/**
 * @brief 计算x坐标在缩放后的索引
 * 
 * @param scale 缩放比例
 * @return 缩放后的索引
 */
int SqDot::sx(double& scale) const { 
    return index_scale(x, scale);
}

/**
 * @brief 计算y坐标在缩放后的索引
 * 
 * @param scale 缩放比例
 * @return 缩放后的索引
 */
int SqDot::sy(double& scale) const { 
    return index_scale(y, scale);
}

/**
 * @brief 按比例缩放坐标点
 * 
 * @param scale 缩放比例
 * @return 缩放后的坐标点
 */
SqDot SqDot::scale(double& scale) { 
    return SqDot(sx(scale), sy(scale));
}

/**
 * @brief 计算两个点的中心点
 * 
 * @param other 另一个点
 * @return 两点的中心点
 */
SqDot SqDot::center(const SqDot& other) {
    return SqDot(x + other.x / 2, y + other.y / 2);
}

/**
 * @brief 获取指定方向的邻居点
 * 
 * @param index 方向索引(0-3)
 * @return 对应方向的邻居点
 */
SqDot SqDot::get_neighbour(int index) const { 
    static const std::array<int, 4> dx = {-1, 1, 0, 0};
    static const std::array<int, 4> dy = {0, 0, -1, 1};
    return SqDot(x + dx[index], y + dy[index]);
}

/**
 * @brief 获取所有四个方向的邻居点
 * 
 * @return 四个方向邻居点的向量
 */
std::vector<SqDot> SqDot::get_neighbour() const { 
    std::vector<SqDot> neighbours;
    for (int idx = 0; idx < 4; idx++) {
        neighbours.emplace_back(get_neighbour(idx));
    }
    return neighbours;
}

/**
 * @brief 获取指定范围内的邻居点
 * 
 * @param x_ceil x方向上限
 * @param y_ceil y方向上限
 * @return 范围内的邻居点向量
 */
std::vector<SqDot> SqDot::get_neighbour(int x_ceil, int y_ceil) const { 
    std::vector<SqDot> neighbours;
    for (int idx = 0; idx < 4; idx++) {
        SqDot neighbour = get_neighbour(idx);

        if (neighbour.x >= 0 && neighbour.x < x_ceil && neighbour.y >= 0 && neighbour.y < y_ceil) {
            neighbours.push_back(neighbour);
        }
    }
    return neighbours;
}

/**
 * @brief 计算到另一个点的距离
 * 
 * @param p 另一个点
 * @return 到另一个点的距离
 */
double SqDot::distance(const SqDot& p) const { 
    return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
}

/**
 * @brief 哈希函数，用于在哈希表中存储SqDot对象
 * 
 * @param p 点对象
 * @return 哈希值
 */
std::size_t SqDotHash::operator()(const SqDot& p) const {

    return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
}

/**
 * @brief 构造函数，使用地图数据创建二维平面
 * 
 * @param map 地图数据
 */
SqPlain::SqPlain(std::vector<std::vector<double>> map): map(std::move(map)) {
    return;
}

/**
 * @brief 构造函数，创建指定大小和初始值的二维平面
 * 
 * @param rows 行数
 * @param cols 列数
 * @param value 初始值
 */
SqPlain::SqPlain(int rows, int cols, double value) : map(rows, std::vector<double>(cols, value)) {}

/**
 * @brief 默认构造函数
 */
SqPlain::SqPlain() {}

/**
 * @brief 检查点是否在地图边缘且可通行
 * 
 * @param point 检查的点
 * @return 如果点在地图范围内且可通行返回true，否则返回false
 */
bool SqPlain::edge_allowed(SqDot point) {

    if (point.x < 0 || point.x >= map.size() || point.y < 0 || point.y >= map[0].size()) {
        return false;
    }
    

    if (map[point.x][point.y] == std::numeric_limits<double>::infinity()) {
        return false;
    }
    
    return true;
}

/**
 * @brief 获取指定点的邻居点
 * 
 * @param point 原始点
 * @param idx 方向索引
 * @return 对应方向的邻居点
 */
SqDot SqPlain::get_neighbour(SqDot point, int idx) {

    static const std::array<int, 4> dx = {-1, 1, 0, 0};
    static const std::array<int, 4> dy = {0, 0, -1, 1};
    

    return SqDot(point.x + dx[idx], point.y + dy[idx]);
}

/**
 * @brief 获取所有有效的邻居点
 * 
 * @param point 原始点
 * @return 有效邻居点的向量
 */
std::vector<SqDot> SqPlain::get_valid_neighbours(SqDot point) {
    std::vector<SqDot> valid_neighbours;
    for (int idx = 0; idx < 4; idx++) {
        SqDot neighbour = get_neighbour(point, idx);
        if (edge_allowed(neighbour)) {
            valid_neighbours.push_back(neighbour);
        }
    }
    return valid_neighbours;
}

/**
 * @brief 获取地图边界内的点
 * 
 * @param point 原始点
 * @return 边界内的点
 */
SqDot SqPlain::orth_near(const SqDot& point) const { 

    return SqDot(std::min(point.x, rows() - 1), std::min(point.y, cols() - 1));
}

/**
 * @brief 计算两点定义区域的中心点
 * 
 * @param fi 第一个点
 * @param se 第二个点
 * @return 区域的中心点
 */
SqDot SqPlain::local_center(SqDot& fi, SqDot& se) {

    SqDot bounded_first = orth_near(fi);
    SqDot bounded_second = orth_near(se);
    

    int min_x = std::min(bounded_first.x, bounded_second.x);
    int max_x = std::max(bounded_first.x, bounded_second.x);
    int min_y = std::min(bounded_first.y, bounded_second.y);
    int max_y = std::max(bounded_first.y, bounded_second.y);
    

    return SqDot((min_x + max_x) / 2, (min_y + max_y) / 2);
}

/**
 * @brief 获取所有邻居点
 * 
 * @param point 原始点
 * @return 邻居点的向量
 */
std::vector<SqDot> SqPlain::get_neighbour(SqDot point) {
    std::vector<SqDot> neighbours;
    for (int idx = 0; idx < 4; idx++) {
        neighbours.emplace_back(get_neighbour(point, idx));
    }
    return neighbours;
}

/**
 * @brief 在地图上查找从起点到终点的路径
 * 
 * @param start 起点
 * @param goal 终点
 * @return 路径点序列
 */
std::vector<SqDot> SqPlain::find_path(SqDot start, SqDot goal) {

    std::priority_queue<std::pair<double, SqDot>, 
                        std::vector<std::pair<double, SqDot>>, 
                        std::greater<std::pair<double, SqDot>>> open_set;
    

    std::unordered_map<SqDot, double, SqDotHash> g_score;
    

    std::unordered_map<SqDot, double, SqDotHash> f_score;
    

    std::unordered_map<SqDot, SqDot, SqDotHash> came_from;
    

    std::unordered_set<SqDot, SqDotHash> closed_set;
    

    g_score[start] = 0;
    f_score[start] = std::sqrt(std::pow(start.x - goal.x, 2) + std::pow(start.y - goal.y, 2));
    open_set.push({f_score[start], start});
    

    while (!open_set.empty()) {

        SqDot current = open_set.top().second;
        open_set.pop();
        

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
        

        closed_set.insert(current);
        

        for (int idx = 0; idx < 4; idx++) {
            SqDot neighbor = get_neighbour(current, idx);
            

            if (!edge_allowed(neighbor)) {
                continue;
            }
            

            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            

            double tentative_g_score = g_score[current] + map[neighbor.x][neighbor.y];
            

            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + std::sqrt(std::pow(neighbor.x - goal.x, 2) + std::pow(neighbor.y - goal.y, 2));
                

                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }
    

    return std::vector<SqDot>();
}

/**
 * @brief 缩放地图
 * 
 * @param scale 缩放比例
 * @return 缩放后的地图
 */
SqPlain SqPlain::scale_graph(double scale) {

    int new_rows = static_cast<int>(std::ceil(map.size() * scale));
    int new_cols = static_cast<int>(std::ceil(map[0].size() * scale));
    

    std::vector<std::vector<double>> new_map(new_rows, std::vector<double>(new_cols, std::numeric_limits<double>::infinity()));
    

    for (int i = 0; i < new_rows; i++) {
        for (int j = 0; j < new_cols; j++) {

            int center_x = static_cast<int>((i + 0.5) / scale);
            int center_y = static_cast<int>((j + 0.5) / scale);
            

            center_x = std::min(center_x, static_cast<int>(map.size() - 1));
            center_y = std::min(center_y, static_cast<int>(map[0].size() - 1));
            
            SqDot center(center_x, center_y);
            

            int side_length = static_cast<int>(1.0 / scale);
            side_length = std::max(1, side_length);
            

            new_map[i][j] = summary(center, side_length);
        }
    }
    
    return SqPlain(std::move(new_map));
}

/**
 * @brief 使用方差方法缩放地图
 * 
 * @param scale 缩放比例
 * @return 缩放后的地图
 */
SqPlain SqPlain::scale_graph_variance(double scale) {

    if (scale <= 0.0) {
        return *this;
    }
    
    int original_rows = map.size();
    int original_cols = map[0].size();
    

    int scaled_rows = static_cast<int>(std::ceil(original_rows * scale));
    int scaled_cols = static_cast<int>(std::ceil(original_cols * scale));
    

    std::vector<std::vector<double>> scaled_map(scaled_rows, std::vector<double>(scaled_cols, 0.0));
    

    for (int i = 0; i < scaled_rows; i++) {
        for (int j = 0; j < scaled_cols; j++) {

            int center_x = static_cast<int>((i + 0.5) / scale);
            int center_y = static_cast<int>((j + 0.5) / scale);
            

            center_x = std::min(center_x, static_cast<int>(map.size() - 1));
            center_y = std::min(center_y, static_cast<int>(map[0].size() - 1));
            
            SqDot center(center_x, center_y);
            

            int side_length = static_cast<int>(1.0 / scale);
            side_length = std::max(1, side_length);
            

            double sum = 0.0;
            double count = 0.0;
            

            for (int dx = -side_length; dx <= side_length; dx++) {
                for (int dy = -side_length; dy <= side_length; dy++) {
                    int current_x = center.x + dx;
                    int current_y = center.y + dy;
                    

                    if (current_x >= 0 && current_x < original_rows && 
                        current_y >= 0 && current_y < original_cols) {
                        sum += map[current_x][current_y];
                        count += 1.0;
                    }
                }
            }
            

            double mean = count > 0 ? sum / count : 0.0;
            

            if (std::abs(mean) < 1e-9) {
                scaled_map[i][j] = mean;
                continue;
            }
            

            double variance_sum = 0.0;
            count = 0.0;
            
            for (int dx = -side_length; dx <= side_length; dx++) {
                for (int dy = -side_length; dy <= side_length; dy++) {
                    int current_x = center.x + dx;
                    int current_y = center.y + dy;
                    

                    if (current_x >= 0 && current_x < original_rows && 
                        current_y >= 0 && current_y < original_cols) {
                        double diff = map[current_x][current_y] - mean;
                        variance_sum += diff * diff;
                        count += 1.0;
                    }
                }
            }
            

            double variance = count > 0 ? variance_sum / count : 0.0;
            

            scaled_map[i][j] = variance;
        }
    }
    
    return SqPlain(std::move(scaled_map));
}

/**
 * @brief 汇总指定区域的信息
 * 
 * @param center 中心点
 * @param side_length 区域边长
 * @return 区域汇总值
 */
double SqPlain::summary(SqDot& center, int side_length) {

    return FastFlatnessEvaluator::evaluate(*this, center, side_length);
}

/**
 * @brief 检查地图是否为空
 * 
 * @return 如果地图为空返回true，否则返回false
 */
bool SqPlain::empty() const {
    return map.empty() || map[0].empty();
}

/**
 * @brief 获取地图行数
 * 
 * @return 地图行数
 */
int SqPlain::rows() const {
    return map.size();
}

/**
 * @brief 计算缩放后的行数
 * 
 * @param scale 缩放比例
 * @return 缩放后的行数
 */
int SqPlain::row_scale(double& scale) const {
    return index_scale(rows(), scale);
}

/**
 * @brief 获取地图列数
 * 
 * @return 地图列数
 */
int SqPlain::cols() const {
    return map[0].size();
}

/**
 * @brief 计算缩放后的列数
 * 
 * @param scale 缩放比例
 * @return 缩放后的列数
 */
int SqPlain::col_scale(double& scale) const {
    return index_scale(cols(), scale);
}

/**
 * @brief 重载下标操作符，用于访问地图行数据
 * 
 * @param index 行索引
 * @return 对应行的引用
 */
std::vector<double>& SqPlain::operator[](int index) {
    return map[index];
}

/**
 * @brief 重载下标操作符，用于访问地图行数据（const版本）
 * 
 * @param index 行索引
 * @return 对应行的const引用
 */
const std::vector<double>& SqPlain::operator[](int index) const {
    return map[index];
}

/**
 * @brief 计算从一个点到另一个点的移动代价
 * 
 * @param at 起点
 * @param to 终点
 * @return 移动代价
 */
double SqPlain::cost(SqDot& at, SqDot& to) {
    return manhattan_distance(at, to) + map[to.x][to.y];
}

/**
 * @brief 将缩放地图上的点恢复到原始地图上的点
 * 
 * @param dot 缩放地图上的点
 * @param scale 缩放比例
 * @return 原始地图上的点
 */
SqDot SqPlain::restore_dot(SqDot& dot, double scale) const { 
    return orth_near(dot.central_restore(scale));
}

/**
 * @brief 恢复缩放地图上的区块到原始地图上的点对
 * 
 * @param block 缩放地图上的区块
 * @param scale 缩放比例
 * @return 原始地图上的点对
 */
std::pair<SqDot, SqDot> SqPlain::restore(const SqDot& block, double scale) const {
    SqDot fi = orth_near(block.scale(scale));
    SqDot se = orth_near(SqDot(block.x + 1, block.y + 1).scale(scale));
    return {fi, se};
}

/**
 * @brief 检查两个点是否在同一个缩放区块内
 * 
 * @param a 第一个点
 * @param b 第二个点
 * @param scale 缩放比例
 * @return 如果两点在同一个区块内返回true，否则返回false
 */
bool SqPlain::in_same_block(const SqDot& a, const SqDot& b, double scale) const {
    SqDot a_block = a.scale(scale);
    SqDot b_block = b.scale(scale);
    return a_block == b_block;
}

/**
 * @brief 构造函数，创建三维空间中的点
 * 
 * @param x x坐标
 * @param y y坐标
 * @param z z坐标
 */
CuDot::CuDot(double x, double y, double z): x(x), y(y), z(z) {
    return;
}

/**
 * @brief 重载加法操作符，实现两点相加
 * 
 * @param dot 另一个点
 * @return 相加后的点
 */
CuDot CuDot::operator+(const CuDot& dot) const {
    return CuDot(x + dot.x, y + dot.y, z + dot.z);
}

/**
 * @brief 计算点在xy平面上的投影向量
 * 
 * @return 投影向量对应的二维点
 */
SqDot CuDot::slide() {

    double magnitude = sqrt(x*x + y*y + z*z);
    

    if (magnitude == 0) {
        return SqDot(0, 0);
    }
    

    double xy_magnitude = sqrt(x*x + y*y);
    

    double slide_magnitude = xy_magnitude;
    

    if (xy_magnitude == 0) {
        return SqDot(0, 0);
    }
    

    double unit_x = x / xy_magnitude;
    double unit_y = y / xy_magnitude;
    

    int slide_x = static_cast<int>(round(unit_x * slide_magnitude));
    int slide_y = static_cast<int>(round(unit_y * slide_magnitude));
    
    return SqDot(slide_x, slide_y);
}

/**
 * @brief 构造函数，创建三维线对象
 * 
 * @param point 线上的点
 */
CuLine::CuLine(const CuDot& point): point(point) {
    return;
}

/**
 * @brief 偏移点
 * 
 * @param dot 偏移量
 * @return 偏移后的点
 */
CuDot CuLine::offset(const CuDot& dot) {
    return point + dot;
}

/**
 * @brief 获取线上的点
 * 
 * @return 线上的点
 */
CuDot CuLine::get() {
    return point;
}

/**
 * @brief 计算与另一条线的法向量
 * 
 * @param other 另一条线
 * @return 法向量
 */
CuLine CuLine::normal_vector(const CuLine& other) const {

    CuDot result(point.y * other.point.z - point.z * other.point.y,
                 point.z * other.point.x - point.x * other.point.z,
                 point.x * other.point.y - point.y * other.point.x);
    return CuLine(result);
}

/**
 * @brief 构造函数，创建三维平面对象
 * 
 * @param A 平面方程参数A
 * @param B 平面方程参数B
 * @param C 平面方程参数C
 * @param D 平面方程参数D
 */
CuPlain::CuPlain(double A, double B, double C, double D): A(A), B(B), C(C), D(D), define_extend(0) {
    return;
}

/**
 * @brief 通过三个点定义平面
 * 
 * @param dot 三个点的数组
 * @return 如果成功定义平面返回true，否则返回false
 */
bool CuPlain::define_plaine(const std::array<CuDot, 3>& dot) {

    CuDot v1(dot[1].x - dot[0].x, dot[1].y - dot[0].y, dot[1].z - dot[0].z);
    CuDot v2(dot[2].x - dot[0].x, dot[2].y - dot[0].y, dot[2].z - dot[0].z);
    

    CuDot normal(v1.y * v2.z - v1.z * v2.y,
                 v1.z * v2.x - v1.x * v2.z,
                 v1.x * v2.y - v1.y * v2.x);
    

    if (normal.x == 0 && normal.y == 0 && normal.z == 0) {
        return false;
    }
    

    A = normal.x;
    B = normal.y;
    C = normal.z;
    D = -(A * dot[0].x + B * dot[0].y + C * dot[0].z);
    
    return true;
}

/**
 * @brief 判断点相对于平面的位置
 * 
 * @param dot 点
 * @return 点相对于平面的位置（上方、下方或内部）
 */
CuPos CuPlain::get_pos(const CuDot& dot) const {

    double distance = A * dot.x + B * dot.y + C * dot.z + D;
    

    if (distance > 1e-9) {
        return CuPos::Above;
    } else if (distance < -1e-9) {
        return CuPos::Below;
    } else {
        return CuPos::Inside;
    }
}

/**
 * @brief 计算点到平面的距离
 * 
 * @param dot 点
 * @return 点到平面的距离
 */
double CuPlain::distance(const CuDot& dot) const {

    double numerator = std::abs(A * dot.x + B * dot.y + C * dot.z + D);
    double denominator = std::sqrt(A * A + B * B + C * C);
    

    if (denominator < 1e-9) {
        return 0.0;
    }
    
    return numerator / denominator;
}

/**
 * @brief 获取平面的法向量
 * 
 * @return 平面的法向量
 */
CuDot CuPlain::normal_vector() const {
    return CuDot(A, B, C);
}

/**
 * @brief 计算平面法向量与z轴的夹角
 * 
 * @return 夹角（弧度）
 */
double CuPlain::normal_angle() const {

    double magnitude = sqrt(A*A + B*B + C*C);
    

    if (magnitude == 0) {
        return 0.0;
    }
    

    double cos_theta = C / magnitude;
    

    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    

    double bottom = sqrt(A*A + B*B);
    return atan2(bottom, C);
}