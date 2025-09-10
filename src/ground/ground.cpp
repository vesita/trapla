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
double Ground::stand_angle(std::vector<SqDot>& area) {
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
CuPlain Ground::trip(std::vector<SqDot>& area) { 
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
CuDot Ground::normal(std::vector<SqDot>& area) {
    CuPlain plaine = trip(area);
    return plaine.normal_vector();
}

/**
 * @brief 使用凸包算法拟合三维平面（未实现）
 * 
 * @param area 区域内的点集合
 * @return 三维平面对象
 */
CuPlain Ground::convex_trip(std::vector<SqDot>& area) { 

    return CuPlain();
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