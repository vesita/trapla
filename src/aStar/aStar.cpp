#include "aStar/aStar.hpp"

#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>
#include <algorithm>
#include <cmath>


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
double manhattan_distance(Point a, Point b) {
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
double euclidean_distance(Point a, Point b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
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
 * @return 包含路径x坐标和y坐标的数组，格式为{{x1,x2,...}, {y1,y2,...}}
 *         如果未找到路径，则返回两个空向量
 * 
 * @details 算法主要步骤：
 *          1. 初始化起点，将其加入优先队列
 *          2. 当优先队列不为空时，取出f_score最小的节点
 *          3. 如果该节点是目标节点，则构建并返回路径
 *          4. 否则，遍历该节点的所有邻居节点
 *          5. 对于每个邻居节点，计算新的g_score（从起点到该邻居的代价）
 *          6. 如果该邻居节点是首次访问或者找到了更短的路径，则更新其代价并加入优先队列
 *          7. 重复步骤2-6直到找到目标节点或队列为空
 * 
 * @note 本实现使用曼哈顿距离作为启发式函数h(n)
 * @note 算法的时间复杂度为O(|E|log|V|)，其中|E|是边数，|V|是节点数
 */
std::array<std::vector<double>, 2> a_star_search(Graph& graph,
    Point start, Point goal) {
    
    // 使用优先队列存储待访问节点，按照f_score升序排序（小顶堆）
    // pair的第一个元素是f_score，第二个元素是节点位置
    // 这里使用lambda表达式定义比较函数，实现小顶堆
    auto cmp = [](const std::pair<double, Point>& a, const std::pair<double, Point>& b) {
        return a.first > b.first; // Min-heap based on f_score
    };
    std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, 
                        decltype(cmp)> frontier(cmp);
    
    // 记录每个节点的来源节点，用于路径重构
    std::unordered_map<Point, Point, PointHash> came_from;
    // 记录从起点到每个节点的实际代价g(n)
    std::unordered_map<Point, double, PointHash> cost_so_far;

    // 初始化起点
    frontier.push({0, start});
    came_from[start] = start;
    cost_so_far[start] = 0;

    // 路径点坐标
    std::vector<double> path_x;
    std::vector<double> path_y;

    // A*主循环
    while (!frontier.empty()) {
        // 取出f_score最小的节点（根据优先队列的特性）
        auto [current_f_score, current] = frontier.top();
        frontier.pop();

        // 如果到达目标点，则构建路径
        if (current == goal) {
            // 从目标点回溯到起点构建路径
            std::vector<Point> path;
            Point current_point = goal;
            // 回溯路径直到起点
            while (!(current_point == start)) {
                path.push_back(current_point);
                current_point = came_from[current_point];
            }
            path.push_back(start);
            // 反转路径，使其从起点到终点
            std::reverse(path.begin(), path.end());

            // 将路径点的坐标分别存储到两个向量中
            for (const auto& point : path) {
                path_x.push_back(static_cast<double>(point.x));
                path_y.push_back(static_cast<double>(point.y));
            }
            break;
        }

        // 遍历当前节点的邻居节点（上下左右四个方向）
        for (int idx = 0; idx < 4; idx++) {
            Point next = graph.get_neighbour(current, idx);
            
            // 检查邻居节点是否允许通过（在地图范围内）
            if (graph.point_allowed(next)) {
                // 计算到达邻居节点的新代价
                // 新代价 = 当前节点代价 + 移动到邻居节点的代价
                // 对于不同类型的T，需要适当的转换为double类型
                double move_cost = 1.0; // 默认移动代价（可扩展为从地图数据中获取）
    
                double new_cost = cost_so_far[current] + move_cost;
                
                // 如果是第一次访问该节点，或者找到了更短的路径
                if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    // 计算启发式函数值（预计总代价 = 已花费代价 + 预计剩余代价）
                    // 这里使用曼哈顿距离作为启发式函数h(n)
                    double priority = new_cost + manhattan_distance(next, goal);
                    frontier.push({priority, next});
                    came_from[next] = current;
                }
            }
        }
    }
    
    return {path_x, path_y};
}