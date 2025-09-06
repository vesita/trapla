#include "aStar/aStar.hpp"
#include "utils/geometry.hpp"

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