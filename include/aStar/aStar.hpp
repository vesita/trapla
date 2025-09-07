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
 * 
 * @details 该实现使用以下策略：
 *          1. 使用优先队列存储待访问的节点，按f(n)值排序
 *          2. 使用哈希表记录已访问的节点和到达该节点的最小代价
 *          3. 使用哈希表记录路径信息，用于重构最终路径
 *          4. 使用曼哈顿距离作为启发式函数h(n)
 */
std::vector<SqDot> a_star_search(SqPlain& graph, SqDot start, SqDot goal);

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
 */
std::vector<SqDot> a_star_search_improved(SqPlain& graph, SqDot start, SqDot goal, double foot_size);

/**
 * @brief 将缩放后的引导点还原到原始地图坐标
 * @param guides 缩放后的引导点序列
 * @param scale 缩放比例因子
 * @param unit_size 单元格大小
 * @return 还原后的引导点序列
 * 
 * @details 该方法用于将低分辨率地图中的引导路径映射回原始高分辨率地图，
 *          生成实际的足部落足点序列。
 */
std::vector<SqDot> central_restore_guide(std::vector<SqDot>& guides, double scale, double unit_size);

/**
 * @brief 生成离散化引导路径
 * @param graph 地图图结构
 * @param stride 步长
 * @param start 起始点
 * @param goal 目标点
 * @return 离散化的引导路径点序列
 * 
 * @details 该方法使用A*算法生成从起点到终点的粗略路径，
 *          然后对该路径进行离散化处理，生成等间距的引导点序列。
 */
std::vector<SqDot> discrete_guide(SqPlain& graph, double stride, SqDot start, SqDot goal);

/**
 * @brief 生成离散化引导路径（改进版）
 * @param graph 地图图结构
 * @param stride 步长
 * @param start 起始点
 * @param goal 目标点
 * @param foot_size 足部尺寸
 * @return 离散化的引导路径点序列
 * 
 * @details 该方法使用改进版A*算法生成从起点到终点的粗略路径，
 *          然后对该路径进行离散化处理，生成等间距的引导点序列。
 *          相比于传统方法，该方法在路径规划过程中考虑了区域平整度。
 */
std::vector<SqDot> discrete_guide_improved(SqPlain& graph, double stride, SqDot start, SqDot goal, double foot_size);

#endif