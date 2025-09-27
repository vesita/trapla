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
    // 1. 收集区域内所有有效点的三维坐标
    std::vector<CuDot> dots;
    for (const auto& point : area) {
        if (!is_valid(point)) {
            return CuPlain();
        }
        dots.emplace_back(CuDot{static_cast<double>(point.x), static_cast<double>(point.y), map[point.x][point.y]});
    }
    
    // 2. 如果点数少于3，无法定义平面
    if (dots.size() < 3) {
        return CuPlain();
    }

    // 3. 使用Andrew's Monotone Chain算法计算凸包
    // 首先按照x坐标排序，如果x相同则按照y坐标排序
    std::sort(dots.begin(), dots.end(), [](const CuDot& a, const CuDot& b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    });

    // 构建下凸包
    std::vector<CuDot> lower_hull;
    for (const auto& dot : dots) {
        // 保持向量的叉积非负（逆时针方向）
        while (lower_hull.size() >= 2) {
            CuDot& a = lower_hull[lower_hull.size() - 2];
            CuDot& b = lower_hull[lower_hull.size() - 1];
            // 计算向量ab和bc的叉积
            double cross_product = (b.x - a.x) * (dot.y - a.y) - (dot.x - a.x) * (b.y - a.y);
            // 如果是顺时针转向，则移除b点
            if (cross_product < 0) {
                lower_hull.pop_back();
            } else {
                break;
            }
        }
        lower_hull.push_back(dot);
    }

    // 构建上凸包
    std::vector<CuDot> upper_hull;
    for (int i = static_cast<int>(dots.size()) - 1; i >= 0; i--) {
        const auto& dot = dots[i];
        // 保持向量的叉积非负（逆时针方向）
        while (upper_hull.size() >= 2) {
            CuDot& a = upper_hull[upper_hull.size() - 2];
            CuDot& b = upper_hull[upper_hull.size() - 1];
            // 计算向量ab和bc的叉积
            double cross_product = (b.x - a.x) * (dot.y - a.y) - (dot.x - a.x) * (b.y - a.y);
            // 如果是顺时针转向，则移除b点
            if (cross_product < 0) {
                upper_hull.pop_back();
            } else {
                break;
            }
        }
        upper_hull.push_back(dot);
    }

    // 合并上下凸包，去除重复点
    lower_hull.pop_back();
    upper_hull.pop_back();
    std::vector<CuDot> hull;
    hull.reserve(lower_hull.size() + upper_hull.size());
    hull.insert(hull.end(), lower_hull.begin(), lower_hull.end());
    hull.insert(hull.end(), upper_hull.begin(), upper_hull.end());

    // 4. 如果凸包点数少于3，无法定义平面
    if (hull.size() < 3) {
        return CuPlain();
    }

    // 5. 使用凸包中的前三个点初始化平面
    std::array<CuDot, 3> initial_points = {hull[0], hull[1], hull[2]};
    CuPlain plane;
    plane.define_plaine(initial_points);

    // 如果只有3个点，直接返回
    if (hull.size() == 3) {
        return plane;
    }

    // 6. 迭代优化平面，尝试用剩余的凸包点替换当前平面点以获得更好的拟合
    bool changed = true;
    const int max_iterations = 100;
    int iterations = 0;
    
    while (changed && iterations < max_iterations) {
        changed = false;
        iterations++;
        
        // 寻找距离当前平面最远的点
        double max_distance = 0;
        int best_point_idx = -1;
        
        for (size_t i = 0; i < hull.size(); i++) {
            double distance = plane.distance(hull[i]);
            if (distance > max_distance) {
                max_distance = distance;
                best_point_idx = static_cast<int>(i);
            }
        }
        
        // 如果找到了足够远的点，则尝试用它来改进平面
        if (best_point_idx >= 0 && max_distance > 1e-6) {
            double best_improvement = 0;
            int best_replace_idx = -1;
            
            // 尝试用该点替换当前平面的每个点
            for (int i = 0; i < 3; i++) {
                // 创建临时平面
                std::array<CuDot, 3> temp_points = {
                    (i == 0) ? hull[best_point_idx] : initial_points[0],
                    (i == 1) ? hull[best_point_idx] : initial_points[1],
                    (i == 2) ? hull[best_point_idx] : initial_points[2]
                };

                CuPlain temp_plane;
                if (temp_plane.define_plaine(temp_points)) {
                    // 计算原始平面和临时平面的总误差
                    double original_error = 0;
                    double new_error = 0;

                    for (const auto& dot : hull) {
                        original_error += plane.distance(dot);
                        new_error += temp_plane.distance(dot);
                    }
                    
                    // 计算改进值
                    double improvement = original_error - new_error;
                    if (improvement > best_improvement) {
                        best_improvement = improvement;
                        best_replace_idx = i;
                    }
                }
            }

            // 如果找到改进的替换方案，则更新平面
            if (best_replace_idx >= 0) {
                initial_points[best_replace_idx] = hull[best_point_idx];
                plane.define_plaine(initial_points);
                changed = true;
            }
        } else {
            // 没有找到足够远的点，停止迭代
            break;
        }
    }
    
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