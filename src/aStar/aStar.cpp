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
std::vector<Intex> a_star_search(const SqPlane& graph, const Intex& start, const Intex& goal) {

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
std::vector<Intex> scale_star(const SqPlane& graph, const Intex& start, const Intex& goal, const double& scale) {

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
        for (auto& next: current.get_neighbour_shuffled(sr, sc)) {            
            // 检查邻居点是否在有效范围内
            if (next.x < 0 || next.x >= sr || next.y < 0 || next.y >= sc) {
                continue;
            }

            auto block_pair = graph.restore(next, scale);
            auto steep = steep_extend(graph, block_pair.first,
                                            block_pair.second);
            
            if (steep < 0) {
                continue;
            }
            
            auto new_cost = cost_so_far[current] + graph.cost(current, next) + steep;
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + euclidean_distance(next, sg);
                frontier.push({priority, next});
                // 确保索引在有效范围内
                if (next.x >= 0 && next.x < sr && next.y >= 0 && next.y < sc) {
                    came_from[next.x * sc + next.y] = current;
                }
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

// std::vector<SqDot> scale_star(const SqPlane& graph, const SqDot& start, const SqDot& goal, const double& scale) {

//     auto ss = start.scale(scale);
//     auto sg = goal.scale(scale);
//     auto sr = graph.row_scale(scale);
//     auto sc = graph.col_scale(scale);

//     using que_unit = std::pair<double, SqDot>;
//     auto cmp = [](const que_unit& a, const que_unit& b) {
//         return a.first > b.first;
//     };

//     std::priority_queue<que_unit, std::vector<que_unit>, decltype(cmp)> frontier(cmp);
//     frontier.push({0.0, ss});

//     std::vector<SqDot> came_from = std::vector<SqDot>(sr * sc, SqDot(-1, -1));

//     std::unordered_map<SqDot, double, SqDotHash> cost_so_far{{ss, 0.0}};
//     while (!frontier.empty()) {
//         SqDot current = frontier.top().second;
//         frontier.pop();
//         if (current == sg) break;
//         for (auto& next: current.get_neighbour(sr, sc)) {            

//             auto block_pair = graph.restore(next, scale);
//             auto steep = steep_extend(graph, block_pair.first.as_index(),
//                                             block_pair.second.as_index());
            
//             if (steep < 0) {
//                 continue;
//             }
            
//             auto new_cost = cost_so_far[current] + graph.cost(current, next) + steep;
//             if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
//                 cost_so_far[next] = new_cost;
//                 double priority = new_cost + euclidean_distance(next, sg);
//                 frontier.push({priority, next});
//                 came_from[next.x_index() * sc + next.y_index()] = current;
//             }
//         }
//     }
    

//     std::vector<SqDot> guides{goal};
//     SqDot current = came_from[sg.x_index() * sc + sg.y_index()];
//     while (current != SqDot(-1, -1) && current != ss) {
//         guides.emplace_back(graph.restore_dot(current, scale));
//         current = came_from[current.x_index() * sc + current.y_index()];
//     }
//     guides.emplace_back(start);
//     std::reverse(guides.begin(), guides.end());
//     return guides;
// }


/**
 * @brief 计算两点定义区域的平均陡峭度
 * 
 * 该函数通过计算两点定义的矩形区域内所有点的平均陡峭度来评估该区域的可行走性
 * 综合考虑了障碍物密度、高度差异、标准差以及机器人足部站立角度约束等因素
 * 
 * @param graph 地图对象
 * @param fi 第一个点
 * @param se 第二个点
 * @return 区域的陡峭程度评分，负值表示不可行区域
 */
double steep_extend(const SqPlane& graph, const Intex& fi, const Intex& se) {

    // 确保 orth_near 正确限制点的坐标在地图范围内
    Intex bounded_first = graph.orth_near(fi);
    Intex bounded_second = graph.orth_near(se);
    

    int min_x = std::min(bounded_first.x, bounded_first.x);
    int max_x = std::max(bounded_first.x, bounded_second.x);
    int min_y = std::min(bounded_first.y, bounded_second.y);
    int max_y = std::max(bounded_first.y, bounded_second.y);    

    // 提前检查边界，确保不会访问无效索引
    if (min_x < 0 || max_x >= graph.rows() || min_y < 0 || max_y >= graph.cols()) {
        return -1.0;
    }

    int obstacle_count = 0;
    int total_count = 0;
    std::vector<double> heights;
    
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {

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
    
    // 如果障碍物占比超过30%，则认为该区域不可行
    if (total_count > 0 && (static_cast<double>(obstacle_count) / total_count) >= 0.3) {
        return -1.0;
    }

    if (heights.empty()) {
        return -1.0;
    }
    
    if (heights.size() == 1) {
        return 0.0;
    }
    
    // 计算平均高度
    double sum = std::accumulate(heights.begin(), heights.end(), 0.0);
    double mean = sum / heights.size();
    
    // 计算标准差
    double variance = 0.0;
    for (const auto& height : heights) {
        double diff = height - mean;
        variance += diff * diff;
    }
    variance /= heights.size();
    double stddev = std::sqrt(variance);
    
    // 计算最大高度差
    auto minmax = std::minmax_element(heights.begin(), heights.end());
    double height_diff = *minmax.second - *minmax.first;
    
    // 计算平均坡度（简化计算，实际应考虑相邻点的高度差）
    double avg_slope = 0.0;
    for (size_t i = 1; i < heights.size(); i++) {
        avg_slope += std::abs(heights[i] - heights[i-1]);
    }
    avg_slope = avg_slope / (heights.size() - 1);
    
    // 综合评分，考虑多种因素：
    // 1. 标准差（反映整体地形波动）权重0.4
    // 2. 最大高度差（反映局部极端变化）权重0.3
    // 3. 平均坡度（反映整体倾斜程度）权重0.3
    double composite_score = 0.4 * stddev + 0.3 * height_diff + 0.3 * avg_slope;
    
    // 如果综合评分超过阈值，认为地形过于陡峭，不适合机器人行走
    // 机器人足部站立角度约束通常为20度(约0.349弧度)
    if (composite_score > 10.0) {  // 阈值可根据实际测试调整
        return -1.0;
    }
    return composite_score;
}