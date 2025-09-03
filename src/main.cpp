#include <iostream>
#include <vector>

#include "robot/foot.hpp"
#include "robot/robot.hpp"
#include "aStar/graph.hpp"
#include "aStar/aStar.hpp"

#include "prepare.hpp"

int main()
{
    Robot robot;
    std::cout << "Hello World!" << std::endl;
    
    // 创建一个简单的测试地图
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
    
    // 运行A*算法
    std::array<std::vector<double>, 2> path = a_star_search(graph, start, goal);
    
    // 输出路径结果
    std::cout << "Path found:" << std::endl;
    for (size_t i = 0; i < path[0].size(); ++i) {
        std::cout << "(" << path[0][i] << ", " << path[1][i] << ")" << std::endl;
    }
    
    if (path[0].empty()) {
        std::cout << "No path found!" << std::endl;
    }
    
    return 0;
}