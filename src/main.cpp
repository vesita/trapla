#include <iostream>
#include <vector>

#include "robot/robot.hpp"
#include "aStar/graph.hpp"
#include "aStar/aStar.hpp"

#include "prepare.hpp"

/**
 * @brief 双足机器人路径规划主程序
 * 
 * 程序演示了双足机器人在网格地图上的路径规划功能，包括：
 * 1. 创建机器人和足部对象
 * 2. 构建测试地图
 * 3. 使用A*算法规划从起点到终点的路径
 * 4. 输出路径结果
 * 
 * @return 程序执行状态码，0表示正常退出
 * 
 * @details 程序整体结构：
 *          - 初始化机器人对象
 *          - 创建测试地图（5x5的网格）
 *          - 定义起点(0,0)和终点(4,4)
 *          - 调用A*算法计算最优路径
 *          - 输出计算得到的路径点
 * 
 * @note 当前实现使用简单的测试地图，实际应用中可以从文件读取地图数据
 * @note 路径规划仅考虑了无障碍物的简单情况，实际应用中需要考虑地形复杂度和机器人足部约束
 */
int main()
{
    Robot robot;
    std::cout << "路径规划程序启动" << std::endl;
    
    // 创建一个简单的测试地图（5x5网格，所有位置都可通过）
    Graph graph ({
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1}
    });
    
    // 定义起点和终点
    Point start{0, 0};
    Point goal{4, 4};
    
    // 运行A*算法计算最优路径
    std::array<std::vector<double>, 2> path = a_star_search(graph, start, goal);
    
    // 输出路径结果
    std::cout << "找到路径:" << std::endl;
    for (size_t i = 0; i < path[0].size(); ++i) {
        std::cout << "(" << path[0][i] << ", " << path[1][i] << ")" << std::endl;
    }
    
    // 检查是否找到了路径
    if (path[0].empty()) {
        std::cout << "未找到路径!" << std::endl;
    }
    
    return 0;
}