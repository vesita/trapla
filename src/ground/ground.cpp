#include "ground/ground.hpp"

/**
 * @brief 构造函数，从文件加载地形数据
 * 
 * 该构造函数通过读取指定文件来初始化地形数据
 * 
 * @param filename 地形数据文件路径
 */
Ground::Ground(std::string filename) {
    CSVReader reader;
    try {
        if (!reader.readFromFile(filename)) {
            std::cout << "错误: 无法读取文件 " << filename << std::endl;
            return;
        } else {
            map = reader.getData();
        }
    } catch (std::exception& e) {
        std::cout << "错误: " << e.what() << std::endl;
        return;
    }
}

Ground::Ground(int rows, int cols) : map(rows, cols, 0.0) {
}

/**
 * @brief 计算指定区域的站立角度
 * 
 * 该函数通过三点拟合平面来计算区域的倾斜角度
 * 
 * @param area 区域内的点集合
 * @return 站立角度（弧度）
 */
double Ground::stand_angle(std::vector<SqDot> area) const {
    CuPlain plaine = trip(area);
    return plaine.normal_angle();
}

/**
 * @brief 获取地形图的形状（行数和列数）
 * 
 * @return 包含行数和列数的数组
 */
std::array<int, 2> Ground::shape() const {
    if (map.empty()) {
        return {0, 0};
    }
    return {static_cast<int>(map.rows()), static_cast<int>(map.cols())};
}

/**
 * @brief 通过区域内的点拟合三维平面
 * 
 * 该函数使用区域内的点数据，通过迭代优化算法拟合最佳平面
 * 
 * @param area 区域内的点集合
 * @return 拟合得到的三维平面
 */
CuPlain Ground::trip(std::vector<SqDot> area) const { 
    std::vector<CuDot> dots;
    for (const auto& point : area) {
        if (point.x < 0 || point.x >= map.rows() || point.y < 0 || point.y >= map.cols()) {
            return CuPlain();
        }
        dots.emplace_back(CuDot{static_cast<double>(point.x), static_cast<double>(point.y), map[point.x][point.y]});
    }
    
    if (dots.size() < 3) {
        return CuPlain();
    }
    

    if (dots.size() == 3) {
        std::array<CuDot, 3> results = {dots[0], dots[1], dots[2]};
        CuPlain plaine;
        plaine.define_plaine(results);
        return plaine;
    }
    
    auto height_cmp = [](const CuDot& a, const CuDot& b) { return a.z < b.z; };
    std::sort(dots.begin(), dots.end(), height_cmp);
    

    std::array<CuDot, 3> results;
    results[0] = dots.back();
    dots.pop_back();
    

    double max_dist_sq = -1;
    size_t second_idx = 0;
    for (size_t i = 0; i < dots.size(); i++) {
        double dx = dots[i].x - results[0].x;
        double dy = dots[i].y - results[0].y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq > max_dist_sq) {
            max_dist_sq = dist_sq;
            second_idx = i;
        }
    }
    results[1] = dots[second_idx];
    dots.erase(dots.begin() + second_idx);
    

    double max_area = -1;
    size_t third_idx = 0;
    for (size_t i = 0; i < dots.size(); i++) {

        double ax = results[1].x - results[0].x;
        double ay = results[1].y - results[0].y;
        double bx = dots[i].x - results[0].x;
        double by = dots[i].y - results[0].y;
        double area = std::abs(ax * by - ay * bx);
        if (area > max_area) {
            max_area = area;
            third_idx = i;
        }
    }
    results[2] = dots[third_idx];
    dots.erase(dots.begin() + third_idx);
    
    CuPlain plaine;
    plaine.define_plaine(results);
    

    bool changed = true;
    const int max_iterations = 100;
    int iterations = 0;
    
    while (changed && !dots.empty() && iterations < max_iterations) {
        changed = false;
        iterations++;
        

        double max_distance = 0;
        int best_point_idx = -1;
        CuPos best_point_pos;
        
        for (size_t i = 0; i < dots.size(); i++) {
            double distance = plaine.distance(dots[i]);
            CuPos pos = plaine.get_pos(dots[i]);
            

            if (pos == CuPos::Above && distance > max_distance) {
                max_distance = distance;
                best_point_idx = static_cast<int>(i);
                best_point_pos = pos;
            }
        }
        

        if (best_point_idx >= 0) {

            double best_improvement = 0;
            int best_replace_idx = -1;
            
            for (int i = 0; i < 3; i++) {

                std::array<CuDot, 3> temp_results = results;
                temp_results[i] = dots[best_point_idx];
                

                CuPlain temp_plane;
                temp_plane.define_plaine(temp_results);
                

                double original_error = 0;
                double new_error = 0;
                

                for (const auto& dot : dots) {
                    original_error += plaine.distance(dot);
                    new_error += temp_plane.distance(dot);
                }
                
                double improvement = original_error - new_error;
                if (improvement > best_improvement) {
                    best_improvement = improvement;
                    best_replace_idx = i;
                }
            }
            

            if (best_replace_idx >= 0) {
                results[best_replace_idx] = dots[best_point_idx];
                dots.erase(dots.begin() + best_point_idx);
                plaine.define_plaine(results);
                changed = true;
            } else {

                dots.erase(dots.begin() + best_point_idx);
            }
        } else {

            break;
        }
    }
    
    return plaine;
}

/**
 * @brief 计算指定区域的法向量
 * 
 * @param area 区域内的点集合
 * @return 区域的法向量
 */
CuDot Ground::normal(std::vector<SqDot> area) const {
    CuPlain plaine = trip(area);
    return plaine.normal_vector();
}

/**
 * @brief 使用凸包算法计算指定区域的接触平面
 * 
 * 该函数通过计算区域点的凸包，然后使用凸包上的点来拟合三维平面
 * 
 * @param area 区域内的点集合
 * @return 拟合得到的三维平面
 */
CuPlain Ground::convex(std::vector<SqDot> area) const {
    // 如果区域点数少于3个，无法构成平面
    if (area.size() < 3) {
        return CuPlain();
    }

    // 使用 Graham Scan 算法计算凸包
    // 1. 找到最下方的点（y最小，相同时x最小）
    size_t lowest_index = 0;
    for (size_t i = 1; i < area.size(); i++) {
        if (area[i].y < area[lowest_index].y || 
            (area[i].y == area[lowest_index].y && area[i].x < area[lowest_index].x)) {
            lowest_index = i;
        }
    }

    // 将最低点放到第一个位置
    if (lowest_index != 0) {
        std::swap(area[0], area[lowest_index]);
    }

    // 2. 根据相对于最低点的极角对其他点进行排序
    SqDot pivot = area[0];
    std::sort(area.begin() + 1, area.end(), [pivot](const SqDot& a, const SqDot& b) {
        // 计算向量与x轴的夹角
        double angle_a = atan2(a.y - pivot.y, a.x - pivot.x);
        double angle_b = atan2(b.y - pivot.y, b.x - pivot.x);
        
        // 如果角度相同，按距离排序
        if (std::abs(angle_a - angle_b) < 1e-9) {
            double dist_a = (a.x - pivot.x) * (a.x - pivot.x) + (a.y - pivot.y) * (a.y - pivot.y);
            double dist_b = (b.x - pivot.x) * (b.x - pivot.x) + (b.y - pivot.y) * (b.y - pivot.y);
            return dist_a < dist_b;
        }
        return angle_a < angle_b;
    });

    // 3. 构建凸包
    std::vector<SqDot> convex_hull;
    for (const auto& point : area) {
        // 删除导致不是左转的点
        while (convex_hull.size() > 1) {
            SqDot& a = convex_hull[convex_hull.size() - 2];
            SqDot& b = convex_hull[convex_hull.size() - 1];
            
            // 计算向量叉积判断转向
            double cross_product = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
            
            // 如果是右转或共线，移除最后一个点
            if (cross_product <= 0) {
                convex_hull.pop_back();
            } else {
                break;
            }
        }
        convex_hull.push_back(point);
    }

    // 4. 如果凸包点数少于3个，无法构成平面
    if (convex_hull.size() < 3) {
        return CuPlain();
    }

    // 5. 从凸包中选择合适的点来拟合平面
    // 选择凸包上距离最远的三个点
    double max_distance = -1;
    std::array<SqDot, 3> selected_points;
    
    for (size_t i = 0; i < convex_hull.size(); i++) {
        for (size_t j = i + 1; j < convex_hull.size(); j++) {
            for (size_t k = j + 1; k < convex_hull.size(); k++) {
                double dist_ij = convex_hull[i].distance(convex_hull[j]);
                double dist_ik = convex_hull[i].distance(convex_hull[k]);
                double dist_jk = convex_hull[j].distance(convex_hull[k]);
                double total_dist = dist_ij + dist_ik + dist_jk;
                
                if (total_dist > max_distance) {
                    max_distance = total_dist;
                    selected_points = {convex_hull[i], convex_hull[j], convex_hull[k]};
                }
            }
        }
    }

    // 6. 将二维点转换为三维点并拟合平面
    std::array<CuDot, 3> cu_points;
    for (int i = 0; i < 3; i++) {
        const SqDot& sq_point = selected_points[i];
        if (sq_point.x < 0 || sq_point.x >= map.rows() || sq_point.y < 0 || sq_point.y >= map.cols()) {
            return CuPlain();
        }
        cu_points[i] = CuDot{sq_point.x, sq_point.y, map[sq_point.x_index()][sq_point.y_index()]};
    }

    CuPlain plane;
    plane.define_plaine(cu_points);
    return plane;
}

/**
 * @brief 检查地形数据是否为空
 * 
 * @return 如果地形数据为空返回true，否则返回false
 */
bool Ground::empty() const { 
    return map.empty(); 
}

bool Ground::is_valid(const SqDot& point) const {
    return !map.empty() && point.x >= 0 && point.x < map.rows() && point.y >= 0 && point.y < map.cols();
}

bool Ground::is_valid(const int& x, const int& y) const { 
    return !map.empty() && x >= 0 && x < map.rows() && y >= 0 && y < map.cols();
}

bool Ground::obstacle(const int& x, const int& y) const {
    if (!is_valid(x, y)) {
        return true;
    }
    return map[x][y] < 0.0;
}

bool Ground::set_unit(const int& x, const int& y, bool is_obstacle) {
    if (!is_valid(x, y)) {
        return false;
    }
    map[x][y] = is_obstacle ? -1.0 : 0.0;
    return true;
}

int Ground::rows() const { 
    return map.rows(); 
}

int Ground::cols() const { 
    return map.cols(); 
}