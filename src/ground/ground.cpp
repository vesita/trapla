#include "ground/ground.hpp"

/**
 * @brief 地面类构造函数，从指定文件读取地形数据
 * @param filename 包含地形高度数据的CSV文件路径
 * 
 * @details 该构造函数使用Reader类从CSV文件中读取地形高度数据，
 *          并初始化地图和代价矩阵。如果读取失败，会输出错误信息并返回。
 *          
 * @note CSV文件应包含表示地形高度的数值数据，每个数值代表一个网格点的高度。
 * @note 文件读取失败时不会抛出异常，而是通过输出错误信息提示用户。
 */
Ground::Ground(std::string filename) {
    Reader reader;
    try {
        if (!reader.readFromFile(filename)) {
            std::cout << "Error: Could not read file " << filename << std::endl;
            return;
        } else {
            map = reader.getData();
        }
    } catch (std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}

/**
 * @brief 计算指定区域的站立角度
 * @param area 包含地面点坐标的向量
 * @return 区域的站立角度（弧度）
 */
double Ground::stand_angle(std::vector<SqDot>& area) {
    CuPlain plaine = trip(area);
    return plaine.normal_angle();
}

std::array<int, 2> Ground::shape() const {
    if (map.empty()) {
        return {0, 0};
    }
    return {static_cast<int>(map.rows()), static_cast<int>(map.cols())};
}

/**
 * @brief 通过指定区域的点计算最优平面
 * @param area 包含地面点坐标的向量
 * @return 表示最优平面的CuPlain对象
 * 
 * @details 该函数通过以下步骤计算最优平面：
 *          1. 将输入的点转换为包含高度信息的三维点
 *          2. 按照z坐标（高度）对点进行排序
 *          3. 选择初始的三个点作为初始平面（使用更合理的方法）
 *          4. 遍历剩余的点，如果发现有更优的点组合，
 *             则用该点替换当前平面的一个点并重新计算平面
 *          5. 返回最终的最优平面
 *          
 *          初始点选择策略：
 *          - 首先选择最高点作为第一个点
 *          - 然后选择与第一个点距离最远的点作为第二个点
 *          - 最后选择能使三角形面积最大的点作为第三个点
 *          
 *          迭代优化策略：
 *          - 遍历剩余点，寻找对平面拟合改善最大的点
 *          - 使用点到平面的距离作为评估指标
 *          - 限制最大迭代次数防止无限循环
 *          
 * @note 该算法假定输入区域至少包含3个点，否则无法定义平面。
 * @note 算法复杂度为O(n²)，其中n为输入点的数量。
 */
CuPlain Ground::trip(std::vector<SqDot>& area) { 
    std::vector<CuDot> dots;
    for (const auto& point : area) {
        if (point.x < 0 || point.x >= map.rows() || point.y < 0 || point.y >= map.cols()) {
            return CuPlain(); // 点超出地图范围
        }
        dots.emplace_back(CuDot{static_cast<double>(point.x), static_cast<double>(point.y), map[point.x][point.y]});
    }
    
    if (dots.size() < 3) {
        return CuPlain(); // 点数不足，无法定义平面
    }
    
    // 如果只有三个点，直接使用它们定义平面
    if (dots.size() == 3) {
        std::array<CuDot, 3> results = {dots[0], dots[1], dots[2]};
        CuPlain plaine;
        plaine.define_plaine(results);
        return plaine;
    }
    
    auto height_cmp = [](const CuDot& a, const CuDot& b) { return a.z < b.z; };
    std::sort(dots.begin(), dots.end(), height_cmp);
    
    // 选择初始三个点的更好策略：
    // 1. 选择一个高点作为起点
    // 2. 选择一个与第一点距离较远的点
    // 3. 选择第三个点以最大化三角形面积
    std::array<CuDot, 3> results;
    results[0] = dots.back(); // 选择最高点
    dots.pop_back();
    
    // 选择距离最高点最远的点作为第二个点
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
    
    // 选择第三个点以最大化三角形面积
    double max_area = -1;
    size_t third_idx = 0;
    for (size_t i = 0; i < dots.size(); i++) {
        // 使用叉积计算三角形面积
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
    
    // 改进的迭代优化策略：
    // 1. 不仅检查点是否在平面上方，还要考虑点到平面的距离
    // 2. 使用更好的替换策略
    bool changed = true;
    const int max_iterations = 100; // 限制迭代次数防止无限循环
    int iterations = 0;
    
    while (changed && !dots.empty() && iterations < max_iterations) {
        changed = false;
        iterations++;
        
        // 寻找对平面影响最大的点
        double max_distance = 0;
        int best_point_idx = -1;
        CuPos best_point_pos;
        
        for (size_t i = 0; i < dots.size(); i++) {
            double distance = plaine.distance(dots[i]);
            CuPos pos = plaine.get_pos(dots[i]);
            
            // 只考虑在平面上方的点，并选择距离最大的点
            if (pos == CuPos::Above && distance > max_distance) {
                max_distance = distance;
                best_point_idx = static_cast<int>(i);
                best_point_pos = pos;
            }
        }
        
        // 如果找到合适的点，则进行替换优化
        if (best_point_idx >= 0) {
            // 尝试替换每个点，选择能使平面更好地拟合所有点的替换方案
            double best_improvement = 0;
            int best_replace_idx = -1;
            
            for (int i = 0; i < 3; i++) {
                // 临时替换一个点
                std::array<CuDot, 3> temp_results = results;
                temp_results[i] = dots[best_point_idx];
                
                // 创建临时平面
                CuPlain temp_plane;
                temp_plane.define_plaine(temp_results);
                
                // 计算替换后的总误差改善
                double original_error = 0;
                double new_error = 0;
                
                // 计算所有点到原始平面和新平面的距离
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
            
            // 如果有改善，则执行替换
            if (best_replace_idx >= 0) {
                results[best_replace_idx] = dots[best_point_idx];
                dots.erase(dots.begin() + best_point_idx);
                plaine.define_plaine(results);
                changed = true;
            } else {
                // 如果没有改善，移除该点继续寻找
                dots.erase(dots.begin() + best_point_idx);
            }
        } else {
            // 没有更多在平面上方的点
            break;
        }
    }
    
    return plaine;
}

/**
 * @brief 通过指定区域的点计算最优平面的法线
 * @param area 包含地面点坐标的向量
 * @return 表示最优平面法线的CuDot对象
 */
CuDot Ground::normal(std::vector<SqDot>& area) {
    CuPlain plaine = trip(area);
    return plaine.normal_vector();
}

/**
 * @brief 通过指定区域的点计算凸面（未实现）
 * @param area 包含地面点坐标的向量
 * @return 表示凸面的CuPlain对象
 */
CuPlain Ground::convex_trip(std::vector<SqDot>& area) { 
    // TODO: 实现convex_trip函数
    return CuPlain();
}

bool Ground::empty() const { 
    return map.empty(); 
}