#include <iostream>
#include <vector>
#include <cmath>

#include "aStar/aStar.hpp"
#include "utils/geometry.hpp"
#include "utils/test_framework.hpp"

/**
 * @brief 测试A*算法在简单地图上的寻路功能
 */
TEST(AStarSimplePath) {
    std::cout << "开始测试A*算法在简单地图上的寻路功能..." << std::endl;
    
    // 创建一个简单的5x5地图，其中(2,2)是障碍物
    std::vector<std::vector<double>> map_data = {
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {1.0, 1.0, std::numeric_limits<double>::infinity(), 1.0, 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0}
    };
    
    SqPlain graph(map_data);
    
    // 设置起点和终点
    SqDot start(0, 0);
    SqDot goal(4, 4);
    
    // 执行A*搜索
    std::vector<SqDot> path = a_star_search(graph, start, goal);
    
    // 验证结果
    if (path.empty()) {
        throw std::runtime_error("A*算法未能找到路径");
    }
    
    // 检查起点和终点
    if (path.front() != start) {
        throw std::runtime_error("路径起点不正确");
    }
    
    if (path.back() != goal) {
        throw std::runtime_error("路径终点不正确");
    }
    
    // 检查路径是否连续（相邻点之间的曼哈顿距离应为1）
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double distance = manhattan_distance(path[i], path[i+1]);
        if (distance != 1.0) {
            throw std::runtime_error("路径不连续");
        }
    }
    
    // 检查路径是否避开了障碍物
    SqDot obstacle(2, 2);
    for (const auto& point : path) {
        if (point == obstacle) {
            throw std::runtime_error("路径穿过了障碍物");
        }
    }
    
    std::cout << "找到路径，路径长度: " << path.size() << std::endl;
    std::cout << "路径点:" << std::endl;
    for (const auto& point : path) {
        std::cout << "  (" << point.x << ", " << point.y << ")" << std::endl;
    }
    
    std::cout << "\nA*算法测试完成。" << std::endl;
}

/**
 * @brief 测试A*算法在无法到达目标时的行为
 */
TEST(AStarNoPath) {
    std::cout << "开始测试A*算法在无法到达目标时的行为..." << std::endl;
    
    // 创建一个地图，其中目标被障碍物完全包围
    std::vector<std::vector<double>> map_data = {
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 1.0},
        {1.0, std::numeric_limits<double>::infinity(), 1.0, std::numeric_limits<double>::infinity(), 1.0},
        {1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0}
    };
    
    SqPlain graph(map_data);
    
    // 设置起点和被包围的终点
    SqDot start(0, 0);
    SqDot goal(2, 2);
    
    // 执行A*搜索
    std::vector<SqDot> path = a_star_search(graph, start, goal);
    
    // 验证结果：应该找不到路径
    if (!path.empty()) {
        throw std::runtime_error("A*算法错误地找到了被障碍物包围的目标的路径");
    }
    
    std::cout << "正确识别出无法到达目标。" << std::endl;
    std::cout << "\nA*算法无路径测试完成。" << std::endl;
}

int main() {
    // 运行所有测试
    bool passed = TestFramework::getInstance().runTests();
    return passed ? 0 : 1;
}