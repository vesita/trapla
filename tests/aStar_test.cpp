#include "utils/test_framework.hpp"
#include "aStar/aStar.hpp"
#include "ground/ground.hpp"

TEST(a_star_search_test) {
    // 创建一个简单的测试地图
    // 0 0 0 0 0
    // 0 1 1 1 0
    // 0 0 0 0 0
    // 0 1 1 0 0
    // 0 0 0 0 0
    SqPlain graph(5, 5, 0.0);
    graph[1][1] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[1][2] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[1][3] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[3][1] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[3][2] = std::numeric_limits<double>::infinity(); // 障碍物

    Intex start(0, 0);
    Intex goal(4, 4);

    auto path = a_star_search(graph, start, goal);

    // 检查路径是否正确
    // 应该是 (0,0) -> (1,0) -> (2,0) -> (2,1) -> (2,2) -> (2,3) -> (2,4) -> (3,4) -> (4,4)
    // 或类似的绕过障碍物的路径
    auto& framework = TestFramework::getInstance();
    const std::string testName = "A*搜索测试";

    // 检查路径的基本属性
    if (path.empty()) {
        framework.addFailure(testName, {0, 0, 1, 0}); // 应该有路径但实际上没有
    } else {
        // 检查起点和终点
        if (path.front() != start) {
            framework.addFailure(testName, {0, 1, 0, 1}); // 起点不正确
        }

        if (path.back() != goal) {
            framework.addFailure(testName, {0, 2, 0, 2}); // 终点不正确
        }

        // 检查路径是否连续（相邻点之间应该是邻居关系）
        for (size_t i = 0; i < path.size() - 1; ++i) {
            bool is_neighbor = false;
            auto neighbors = path[i].get_neighbour();
            for (const auto& neighbor : neighbors) {
                if (neighbor == path[i + 1]) {
                    is_neighbor = true;
                    break;
                }
            }
            if (!is_neighbor) {
                framework.addFailure(testName, {0, 3, 0, 3}); // 路径不连续
                break;
            }
        }

        // 检查路径点是否都在地图范围内
        for (const auto& point : path) {
            if (point.x < 0 || point.x >= graph.rows() || point.y < 0 || point.y >= graph.cols()) {
                framework.addFailure(testName, {0, 4, 0, 4}); // 路径点超出地图范围
                break;
            }
        }

        // 检查路径点是否都不是障碍物
        for (const auto& point : path) {
            // 通过Ground类检查障碍物
            if (graph[point.x][point.y] == std::numeric_limits<double>::infinity()) {
                framework.addFailure(testName, {0, 5, 0, 5}); // 路径经过障碍物
                break;
            }
        }
    }

    // 定义列名
    std::vector<std::string> columnNames = {"test_case", "error_type", "expected", "actual"};

    // 写入失败数据到CSV文件
    framework.writeFailures(testName, "astar_failures.csv", columnNames);

    // 如果有失败的测试用例，则抛出异常
    framework.throwIfFailed(testName, "测试失败");

    std::cout << "a_star_search_test: 通过所有测试用例" << std::endl;
}

TEST(scale_star_test) {
    // 创建一个简单的测试地图
    // 0 0 0 0 0
    // 0 1 1 1 0
    // 0 0 0 0 0
    // 0 1 1 0 0
    // 0 0 0 0 0
    SqPlain graph(5, 5, 0.0);
    graph[1][1] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[1][2] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[1][3] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[3][1] = std::numeric_limits<double>::infinity(); // 障碍物
    graph[3][2] = std::numeric_limits<double>::infinity(); // 障碍物

    Intex start(0, 0);
    Intex goal(4, 4);
    double stride = 2.0;

    auto path = scale_star(graph, start, goal, stride);

    auto& framework = TestFramework::getInstance();
    const std::string testName = "Scale*搜索测试";

    // 检查路径的基本属性
    if (path.empty()) {
        framework.addFailure(testName, {1, 0, 1, 0}); // 应该有路径但实际上没有
    } else {
        // 检查起点和终点
        if (path.front() != start) {
            framework.addFailure(testName, {1, 1, 0, 1}); // 起点不正确
        }

        if (path.back() != goal) {
            framework.addFailure(testName, {1, 2, 0, 2}); // 终点不正确
        }

        // 检查路径点是否都在地图范围内
        for (const auto& point : path) {
            if (point.x < 0 || point.x >= graph.rows() || point.y < 0 || point.y >= graph.cols()) {
                framework.addFailure(testName, {1, 4, 0, 4}); // 路径点超出地图范围
                break;
            }
        }
    }

    // 定义列名
    std::vector<std::string> columnNames = {"test_case", "error_type", "expected", "actual"};

    // 写入失败数据到CSV文件
    framework.writeFailures(testName, "scale_star_failures.csv", columnNames);

    // 如果有失败的测试用例，则抛出异常
    framework.throwIfFailed(testName, "测试失败");

    std::cout << "scale_star_test: 通过所有测试用例" << std::endl;
}

TEST(edge_cases_test) {
    // 测试边界情况
    
    // 测试起点和终点相同的情况
    SqPlain graph2(3, 3, 0.0);
    Intex same_point(1, 1);
    auto path2 = a_star_search(graph2, same_point, same_point);
    
    auto& framework = TestFramework::getInstance();
    const std::string testName = "边界情况测试";
    
    if (path2.size() != 1 || path2[0] != same_point) {
        framework.addFailure(testName, {2, 1, 1, static_cast<double>(path2.size())}); // 应该只有一个点
    }
    
    // 定义列名
    std::vector<std::string> columnNames = {"test_case", "error_type", "expected", "actual"};
    
    // 写入失败数据到CSV文件
    framework.writeFailures(testName, "edge_cases_failures.csv", columnNames);
    
    // 如果有失败的测试用例，则抛出异常
    framework.throwIfFailed(testName, "测试失败");
    
    std::cout << "edge_cases_test: 通过所有测试用例" << std::endl;
}

int main(int argc, char* argv[]) {
    try {
        // 设置工作目录
        if (argc > 1) {
            TestFramework::getInstance().setWorkingDirectory(argv[1]);
        }
        return TestFramework::getInstance().runTests() ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "测试执行出错: " << e.what() << std::endl;
        return 1;
    }
}