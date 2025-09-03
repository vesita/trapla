#include "aStar/aStar.hpp"

#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>
#include <algorithm>
#include <cmath>

// 用于在unordered_map中使用Point作为键的哈希函数
struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>{}(p.x) ^ (std::hash<int>{}(p.y) << 1);
    }
};

/**
 * 计算两点间的曼哈顿距离
 * 
 * @param a 点A坐标
 * @param b 点B坐标
 * @return 曼哈顿距离值
 */
double manhattan_distance(Point a, Point b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/**
 * 计算两点间的欧几里得距离
 * 
 * @param a 点A坐标
 * @param b 点B坐标
 * @return 欧几里得距离值
 */
double euclidean_distance(Point a, Point b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * A*寻路算法实现
 * 
 * @param graph 地图图结构，包含障碍物信息和节点连接关系
 * @param start 起始点坐标
 * @param goal  目标点坐标
 * @return 包含路径x坐标和y坐标的数组，格式为{{x1,x2,...}, {y1,y2,...}}
 */
std::array<std::vector<double>, 2> a_star_search(Graph& graph,
    Point start, Point goal) {
    
    // 使用优先队列存储待访问节点，按照f_score升序排序（小顶堆）
    // pair的第一个元素是f_score，第二个元素是节点位置
    auto cmp = [](const std::pair<double, Point>& a, const std::pair<double, Point>& b) {
        return a.first > b.first; // Min-heap based on f_score
    };
    std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, 
                        decltype(cmp)> frontier(cmp);
    
    // 记录每个节点的来源节点
    std::unordered_map<Point, Point, PointHash> came_from;
    // 记录从起点到每个节点的实际代价
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
        // 取出f_score最小的节点
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

        // 遍历当前节点的邻居节点
        for (int idx = 0; idx < 4; idx++) {
            Point next = graph.get_neighbour(current, idx);
            
            // 检查邻居节点是否允许通过（在地图范围内）
            if (graph.point_allowed(next)) {
                // 计算到达邻居节点的新代价
                // 新代价 = 当前节点代价 + 移动到邻居节点的代价
                // 对于不同类型的T，需要适当的转换为double类型
                double move_cost = 1.0; // 默认移动代价
    
                double new_cost = cost_so_far[current] + move_cost;
                
                // 如果是第一次访问该节点，或者找到了更短的路径
                if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    // 计算启发式函数值（预计总代价 = 已花费代价 + 预计剩余代价）
                    double priority = new_cost + manhattan_distance(next, goal);
                    frontier.push({priority, next});
                    came_from[next] = current;
                }
            }
        }
    }
    
    return {path_x, path_y};
}