#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <array>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>
#include <algorithm>
#include <cmath>

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
double manhattan_distance(SqDot a, SqDot b);

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
double euclidean_distance(SqDot a, SqDot b);

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
std::vector<SqDot> a_star_search(SqPlain& graph, SqDot start, SqDot goal);

#endif