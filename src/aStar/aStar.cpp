#include "aStar/aStar.hpp"
#include "utils/geometry.hpp"
#include "utils/fast_flatness.hpp"

/**
 * @brief 计算两点间的曼哈顿距离（L1距离）
 * 
 * 曼哈顿距离是两点在南北方向和东西方向上的距离之和
 * 公式: |x1-x2| + |y1-y2|
 * 
 * @param a 点A坐标
 * @param b 点B坐标
 * @return 曼哈顿距离值
 */
double manhattan_distance(SqDot a, SqDot b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/**
 * @brief 计算两点间的欧几里得距离（L2距离）
 * 
 * 欧几里得距离是两点之间的直线距离
 * 公式: √[(x1-x2)² + (y1-y2)²]
 * 
 * @param a 点A坐标
 * @param b 点B坐标
 * @return 欧几里得距离值
 */
double euclidean_distance(SqDot a, SqDot b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

/**
 * @brief A*寻路算法实现
 * 
 * A*算法是一种启发式搜索算法，结合了Dijkstra算法的准确性和贪心最佳优先搜索的效率。
 * 它使用函数f(n) = g(n) + h(n)来评估节点：
 * - g(n): 从起点到节点n的实际代价
 * - h(n): 从节点n到终点的启发式估计代价
 * 
 * @param graph 地图图结构，包含障碍物信息和节点连接关系
 * @param start 起始点坐标
 * @param goal  目标点坐标
 * @return 包含路径点坐标的vector<SqDot>
 *         如果未找到路径，则返回空vector
 * 
 * @details 该实现使用以下策略：
 *          1. 使用优先队列存储待访问的节点，按f(n)值排序
 *          2. 使用哈希表记录已访问的节点和到达该节点的最小代价
 *          3. 使用哈希表记录路径信息，用于重构最终路径
 *          4. 使用欧几里得距离作为启发式函数h(n)
 *          
 *          算法流程：
 *          1. 初始化起点，计算其f_score并加入开放列表
 *          2. 当开放列表不为空时，取出f_score最小的节点
 *          3. 如果当前节点是目标节点，则重构路径并返回
 *          4. 遍历当前节点的所有邻居节点
 *          5. 对每个邻居节点计算 tentative_g_score
 *          6. 如果通过当前节点到达邻居节点的路径更优，则更新邻居节点信息
 *          7. 将未访问的邻居节点加入开放列表
 *          8. 将当前节点标记为已访问
 *          9. 重复步骤2-8直到找到目标或开放列表为空
 *          
 * @note 该实现使用欧几里得距离作为启发式函数，适用于无约束移动的网格地图。
 * @note 算法复杂度取决于地图大小和障碍物分布，最坏情况下为O(|E|log|V|)。
 */
std::vector<SqDot> a_star_search(SqPlain& graph, SqDot start, SqDot goal) {
    // 自定义比较函数，用于优先队列排序
    // 按照f_score排序（小顶堆），f_score相同时按点坐标排序
    auto cmp = [](const std::pair<double, SqDot>& a, const std::pair<double, SqDot>& b) {
        if (a.first != b.first) {
            return a.first > b.first;  // f_score较小的优先
        }
        // f_score相同时，按坐标排序确保一致性
        if (a.second.x != b.second.x) {
            return a.second.x > b.second.x;
        }
        return a.second.y > b.second.y;
    };
    
    // 优先队列，用于存储待访问的节点，按照f_score排序（小顶堆）
    std::priority_queue<std::pair<double, SqDot>, 
                        std::vector<std::pair<double, SqDot>>, 
                        decltype(cmp)> open_set(cmp);
    
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
    f_score[start] = euclidean_distance(start, goal);
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
            SqDot neighbor = graph.get_neighbour(current, i);
            
            // 检查邻居是否可通行
            if (!graph.edge_allowed(neighbor)) {
                continue;
            }
            
            // 如果邻居已经访问过，跳过
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            // 计算从起点到邻居的代价
            double tentative_g_score = g_score[current] + graph.map[neighbor.x][neighbor.y];
            
            // 如果找到了更优的路径，或者首次访问该邻居
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + euclidean_distance(neighbor, goal);
                
                // 将邻居加入待访问队列
                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }
    
    // 未找到路径，返回空vector
    return std::vector<SqDot>();
}

/**
 * @brief 改进版A*寻路算法实现
 * 
 * 改进版A*算法在传统A*算法基础上增加了对区域平整度的实时评估，
 * 可以在路径规划过程中更准确地评估每个节点的实际通行代价。
 * 
 * @param graph 地图图结构，包含障碍物信息和节点连接关系
 * @param start 起始点坐标
 * @param goal  目标点坐标
 * @param foot_size 足部尺寸，用于计算区域平整度评估的范围
 * @return 包含路径点坐标的vector<SqDot>
 *         如果未找到路径，则返回空vector
 * 
 * @details 该实现使用以下策略：
 *          1. 使用优先队列存储待访问的节点，按f(n)值排序
 *          2. 使用哈希表记录已访问的节点和到达该节点的最小代价
 *          3. 使用哈希表记录路径信息，用于重构最终路径
 *          4. 使用欧几里得距离作为启发式函数h(n)
 *          5. 在计算节点代价时，结合区域平整度评估进行动态调整
 *          
 *          算法流程：
 *          1. 初始化起点，计算其f_score并加入开放列表
 *          2. 当开放列表不为空时，取出f_score最小的节点
 *          3. 如果当前节点是目标节点，则重构路径并返回
 *          4. 遍历当前节点的所有邻居节点
 *          5. 对每个邻居节点计算 tentative_g_score，包含平整度评估
 *          6. 如果通过当前节点到达邻居节点的路径更优，则更新邻居节点信息
 *          7. 将未访问的邻居节点加入开放列表
 *          8. 将当前节点标记为已访问
 *          9. 重复步骤2-8直到找到目标或开放列表为空
 */
std::vector<SqDot> a_star_search_improved(SqPlain& graph, SqDot start, SqDot goal, double foot_size) {
    // 自定义比较函数，用于优先队列排序
    // 按照f_score排序（小顶堆），f_score相同时按点坐标排序
    auto cmp = [](const std::pair<double, SqDot>& a, const std::pair<double, SqDot>& b) {
        if (a.first != b.first) {
            return a.first > b.first;  // f_score较小的优先
        }
        // f_score相同时，按坐标排序确保一致性
        if (a.second.x != b.second.x) {
            return a.second.x > b.second.x;
        }
        return a.second.y > b.second.y;
    };
    
    // 优先队列，用于存储待访问的节点，按照f_score排序（小顶堆）
    std::priority_queue<std::pair<double, SqDot>, 
                        std::vector<std::pair<double, SqDot>>, 
                        decltype(cmp)> open_set(cmp);
    
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
    f_score[start] = euclidean_distance(start, goal);
    open_set.push({f_score[start], start});
    
    // 计算用于平整度评估的区域边长（基于足部尺寸）
    int area_side_length = static_cast<int>(std::ceil(foot_size));
    
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
            SqDot neighbor = graph.get_neighbour(current, i);
            
            // 检查邻居是否可通行
            if (!graph.edge_allowed(neighbor)) {
                continue;
            }
            
            // 如果邻居已经访问过，跳过
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            // 使用快速平整度评估算法计算当前邻居节点区域的平整度
            double flatness_cost = FastFlatnessEvaluator::evaluate(graph, neighbor, area_side_length);
            
            // 计算从起点到邻居的代价（包含平整度评估）
            double tentative_g_score = g_score[current] + graph.map[neighbor.x][neighbor.y] + flatness_cost;
            
            // 如果找到了更优的路径，或者首次访问该邻居
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + euclidean_distance(neighbor, goal);
                
                // 将邻居加入待访问队列
                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }
    
    // 未找到路径，返回空vector
    return std::vector<SqDot>();
}

std::vector<SqDot> central_restore_guide(std::vector<SqDot>& guides, double scale, double unit_size) {
    std::vector<SqDot> path;
    for (const auto& point : guides) {
        path.emplace_back(point.central_restore(scale, unit_size));
    }
    return path;
}

std::vector<SqDot> discrete_guide(SqPlain& graph, double stride, SqDot start, SqDot goal) {
    auto scale = 1.0 / stride;
    auto scale_graph = graph.scale_graph(scale);
    auto scale_start = start.scale(scale);
    auto scale_goal = goal.scale(scale);

    auto guides = a_star_search(scale_graph, scale_start, scale_goal);
    return central_restore_guide(guides, scale, graph.map.size());
}

std::vector<SqDot> discrete_guide_improved(SqPlain& graph, double stride, SqDot start, SqDot goal, double foot_size) {
    auto scale = 1.0 / stride;
    auto scale_graph = graph.scale_graph(scale);
    auto scale_start = start.scale(scale);
    auto scale_goal = goal.scale(scale);

    auto guides = a_star_search_improved(scale_graph, scale_start, scale_goal, foot_size);
    return central_restore_guide(guides, scale, graph.map.size());
}