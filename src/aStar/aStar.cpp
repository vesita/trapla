#include "aStar/aStar.hpp"

/**
 * @brief 使用A*算法在二维地图上搜索从起点到终点的最短路径
 * 
 * 该函数实现了经典的A*搜索算法，使用欧几里得距离作为启发式函数，
 * 曼哈顿距离和地形代价组合作为实际代价函数
 * 
 * @param graph 二维地图对象，包含地形信息
 * @param start 起点坐标
 * @param goal 终点坐标
 * @return 从起点到终点的路径点序列
 */
std::vector<Intex> a_star_search(const SqPlain& graph, const Intex& start, const Intex& goal) {

    using que_unit = std::pair<double, Intex>;
    auto cmp = [](const que_unit& a, const que_unit& b) {
        return a.first > b.first;
    };
    std::priority_queue<que_unit, std::vector<que_unit>, decltype(cmp)> frontier(cmp);
    frontier.push({0.0, start});
    std::vector<Intex> came_from = std::vector<Intex>(graph.rows() * graph.cols(), Intex(-1, -1));
    std::unordered_map<Intex, double, IntexHash> cost_so_far{{start, 0.0}};
    while (!frontier.empty()) {
        auto current = frontier.top().second;
        frontier.pop();
        if (current == goal) break;

        for (auto& next: graph.get_valid_neighbours(current)) {
            auto new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + manhattan_distance(next, goal);
                frontier.push({priority, next});
                came_from[next.x * graph.cols() + next.y] = current;
            }
        }
    }

    std::vector<Intex> path;
    Intex current = goal;
    int max_steps = graph.rows() * graph.cols();
    int steps = 0;
    
    while (current != Intex(-1, -1) && current != start && steps < max_steps) {
        path.push_back(current);

        if (current.x < 0 || current.x >= graph.rows() || current.y < 0 || current.y >= graph.cols()) {
            path.clear();
            break;
        }
        current = came_from[current.x * graph.cols() + current.y];
        steps++;
    }
    
    if (steps >= max_steps) {
        path.clear();
    } else if (!path.empty() || current == start) {
        path.push_back(start);
    }
    
    std::reverse(path.begin(), path.end());
    return std::move(path);
}

/**
 * @brief 使用缩放地图的A*算法进行路径规划
 * 
 * 该函数通过缩放地图来降低搜索空间复杂度，然后在缩放后的地图上进行A*搜索，
 * 最后将路径映射回原始地图坐标系
 * 
 * @param graph 原始二维地图对象
 * @param start 起点坐标
 * @param goal 终点坐标
 * @param stride 步长参数，用于计算缩放比例
 * @return 在原始地图上的引导点序列
 */
std::vector<Intex> scale_star(const SqPlain& graph, const Intex& start, const Intex& goal, const double& stride) {
    auto scale = 1.0/stride;

    auto ss = start.scale(scale);
    auto sg = goal.scale(scale);
    auto sr = graph.row_scale(scale);
    auto sc = graph.col_scale(scale);

    using que_unit = std::pair<double, Intex>;
    auto cmp = [](const que_unit& a, const que_unit& b) {
        return a.first > b.first;
    };

    std::priority_queue<que_unit, std::vector<que_unit>, decltype(cmp)> frontier(cmp);
    frontier.push({0.0, ss});

    std::vector<Intex> came_from = std::vector<Intex>(sr * sc, Intex(-1, -1));

    std::unordered_map<Intex, double, IntexHash> cost_so_far{{ss, 0.0}};
    while (!frontier.empty()) {
        Intex current = frontier.top().second;
        frontier.pop();
        if (current == sg) break;
        for (auto& next: current.get_neighbour(sr, sc)) {            

            auto block_pair = graph.restore(next, scale);
            auto steep = steep_extend(graph, block_pair.first, block_pair.second);
            
            if (steep < 0) {
                continue;
            }
            
            auto new_cost = cost_so_far[current] + graph.cost(current, next) + steep;
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + euclidean_distance(next, sg);
                frontier.push({priority, next});
                came_from[next.x * sc + next.y] = current;
            }
        }
    }
    

    std::vector<Intex> guides{goal};
    Intex current = came_from[sg.x * sc + sg.y];
    while (current != Intex(-1, -1) && current != ss) {
        guides.emplace_back(graph.restore_dot(current, scale));
        current = came_from[current.x * sc + current.y];
    }
    guides.emplace_back(start);
    std::reverse(guides.begin(), guides.end());
    return guides;
}

/**
 * @brief 在缩放地图上执行A*搜索并返回缩放地图上的路径
 * 
 * 与scale_star不同，该函数直接返回在缩放地图上的路径点，而不是映射回原始地图
 * 
 * @param graph 原始二维地图对象
 * @param start 起点坐标
 * @param goal 终点坐标
 * @param stride 步长参数，用于计算缩放比例
 * @return 在缩放地图上的路径点序列
 */
std::vector<Intex> scale_star_on_scaled_map(const SqPlain& graph, Intex start, Intex goal, double stride) {
    auto scale = 1.0/stride;

    auto ss = start.scale(scale);
    auto sg = goal.scale(scale);
    auto sr = graph.row_scale(scale);
    auto sc = graph.col_scale(scale);

    using que_unit = std::pair<double, Intex>;
    auto cmp = [](const que_unit& a, const que_unit& b) {
        return a.first > b.first;
    };

    std::priority_queue<que_unit, std::vector<que_unit>, decltype(cmp)> frontier(cmp);
    frontier.push({0.0, ss});

    std::vector<Intex> came_from = std::vector<Intex>(sr * sc, Intex(-1, -1));

    std::unordered_map<Intex, double, IntexHash> cost_so_far{{ss, 0.0}};
    while (!frontier.empty()) {
        Intex current = frontier.top().second;
        frontier.pop();
        if (current == sg) break;
        for (auto& next: current.get_neighbour()) {            

            auto block_pair = graph.restore(next, scale);
            auto steep = steep_extend(graph, block_pair.first, block_pair.second);
            
            if (steep < 0) {
                continue;
            }
            
            auto new_cost = cost_so_far[current] + graph.cost(current, next) + steep;
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + euclidean_distance(next, sg);
                frontier.push({priority, next});
                came_from[next.x * sc + next.y] = current;
            }
        }
    }
    

    std::vector<Intex> path;
    Intex current = sg;
    while (current != Intex(-1, -1) && current != ss) {
        path.emplace_back(current);
        current = came_from[current.x * sc + current.y];
    }
    path.push_back(ss);
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * @brief 将引导点序列转换为中心恢复点序列
 * 
 * 该函数将缩放地图上的引导点映射回原始地图的中心点
 * 
 * @param guides 缩放地图上的引导点序列
 * @param scale 缩放比例
 * @param unit_size 单元格大小
 * @return 原始地图上的中心点序列
 */
std::vector<Intex> central_restore_guide(std::vector<Intex>& guides, double scale, double unit_size) {
    std::vector<Intex> path;
    for (const auto& point : guides) {
        path.emplace_back(point.central_restore(scale));
    }
    return path;
}

/**
 * @brief 离散引导点生成函数
 * 
 * 通过缩放地图进行路径规划，然后将结果映射回原始地图
 * 
 * @param graph 原始地图
 * @param stride 步长
 * @param start 起点
 * @param goal 终点
 * @return 原始地图上的引导点序列
 */
std::vector<Intex> discrete_guide(SqPlain& graph, double stride, Intex start, Intex goal) {
    auto scale = 1.0/stride;
    auto scale_graph = graph.scale_graph(scale);
    auto scale_start = start.scale(scale);
    auto scale_goal = goal.scale(scale);

    auto guides = a_star_search(scale_graph, scale_start, scale_goal);
    return central_restore_guide(guides, scale, graph.map.size());
}

/**
 * @brief 计算两点间区域的陡峭程度
 * 
 * 该函数评估两点定义的区域的地形陡峭程度，用于路径规划中的代价计算
 * 
 * @param graph 地图对象
 * @param fi 第一个点
 * @param se 第二个点
 * @return 区域的陡峭程度评分，负值表示不可行区域
 */
double steep_extend(const SqPlain& graph, Intex& fi, Intex& se) {

    // 确保 orth_near 正确限制点的坐标在地图范围内
    Intex bounded_first = graph.orth_near(fi);
    Intex bounded_second = graph.orth_near(se);
    

    int min_x = std::min(bounded_first.x, bounded_second.x);
    int max_x = std::max(bounded_first.x, bounded_second.x);
    int min_y = std::min(bounded_first.y, bounded_second.y);
    int max_y = std::max(bounded_first.y, bounded_second.y);
    

    if (min_x >= max_x || min_y >= max_y) {
        return -1.0;
    }
    

    int obstacle_count = 0;
    int total_count = 0;
    std::vector<double> heights;
    
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {

            if (x < 0 || x >= graph.rows() || y < 0 || y >= graph.cols()) {
                continue;
            }

            total_count++;
            double height = graph[x][y];

            if (height == std::numeric_limits<double>::infinity()) {
                obstacle_count++;
            }

            else if (height >= 0) {
                heights.emplace_back(height);
            }
        }
    }
    

    if (total_count > 0 && (static_cast<double>(obstacle_count) / total_count) >= 0.5) {
        return -1.0;
    }
    

    if (heights.empty()) {
        return -1.0;
    }
    

    if (heights.size() == 1) {
        return 0.0;
    }
    

    double sum = std::accumulate(heights.begin(), heights.end(), 0.0);
    double mean = sum / heights.size();
    

    double variance = 0.0;
    for (const auto& height : heights) {
        double diff = height - mean;
        variance += diff * diff;
    }
    variance /= heights.size();
    double stddev = std::sqrt(variance);
    

    auto minmax = std::minmax_element(heights.begin(), heights.end());
    double height_diff = *minmax.second - *minmax.first;
    

    return 0.7 * stddev + 0.3 * height_diff;
}