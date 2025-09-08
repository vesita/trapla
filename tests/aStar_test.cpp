#include "aStar/aStar.hpp"
#include "utils/test_framework.hpp"
#include "ground/ground.hpp"
#include "utils/io.hpp"
#include <iostream>
#include <cassert>

TEST(scale_star_basic) {

    SqPlain graph(5, 5);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            graph[i][j] = 0.0;
        }
    }
    
    SqDot start(0, 0);
    SqDot goal(4, 4);
    double stride = 1.0;
    
    auto path = scale_star(graph, start, goal, stride);
    

    assert(!path.empty());
    

    assert(path.front() == start);
    assert(path.back() == goal);
    

    std::string logMessage = "scale_star_basic测试通过。路径点数: " + std::to_string(path.size()) + "，路径坐标: ";
    for (const auto& point : path) {
        logMessage += "(" + std::to_string(point.x) + "," + std::to_string(point.y) + ") ";
    }
    
    std::cout << logMessage << std::endl;
    

    auto pathFile = IOManager::getInstance().createOutputFile("log/scale_star_basic_path.txt");
    if (pathFile && pathFile->is_open()) {
        *pathFile << "scale_star_basic测试路径:\n";
        for (size_t i = 0; i < path.size(); ++i) {
            *pathFile << i << ": (" << path[i].x << ", " << path[i].y << ")\n";
        }
    }
}

TEST(scale_star_with_obstacle) {

    SqPlain graph(5, 5);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            graph[i][j] = 0.0;
        }
    }
    

    graph[2][2] = std::numeric_limits<double>::infinity();
    
    SqDot start(0, 0);
    SqDot goal(4, 4);
    double stride = 1.0;
    
    auto path = scale_star(graph, start, goal, stride);
    

    assert(!path.empty());
    

    bool passesObstacle = false;
    for (const auto& point : path) {
        if (point.x == 2 && point.y == 2) {
            passesObstacle = true;
            break;
        }
    }
    

    assert(!passesObstacle);
    

    std::string logMessage = "scale_star_with_obstacle测试通过。路径点数: " + std::to_string(path.size()) + "，路径坐标: ";
    for (const auto& point : path) {
        logMessage += "(" + std::to_string(point.x) + "," + std::to_string(point.y) + ") ";
    }
    
    std::cout << logMessage << std::endl;
    

    auto pathFile = IOManager::getInstance().createOutputFile("log/scale_star_with_obstacle_path.txt");
    if (pathFile && pathFile->is_open()) {
        *pathFile << "scale_star_with_obstacle测试路径:\n";
        for (size_t i = 0; i < path.size(); ++i) {
            *pathFile << i << ": (" << path[i].x << ", " << path[i].y << ")\n";
        }
    }
}

TEST(scale_star_action_test) {

    std::string dataPath = buildPath("data/csv/map.csv");
    Ground map = Ground(dataPath);
    auto result = scale_star(map.map, SqDot(0, 0), SqDot(300, 400), 40);
    std::cout << "scale_star_action_test: 已找到 " << result.size() << " 个路径点" << std::endl;

    auto pathFile = IOManager::getInstance().createOutputFile("log/scale_star_action_path.txt");
    if (pathFile && pathFile->is_open()) {
        *pathFile << "scale_star 路径点" << std::endl;
        for (auto& point : result) {
            *pathFile << point.x << " " << point.y << std::endl;
        }
        std::cout << "scale_star_action_test: 已写入路径文件" << std::endl;
    }
}

TEST(steep_extend_flat_terrain) {

    SqPlain graph(5, 5);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            graph[i][j] = 0.0;
        }
    }
    
    SqDot p1(1, 1);
    SqDot p2(2, 2);
    
    double steepness = steep_extend(graph, p1, p2);
    

    assert(steepness >= 0);
    assert(steepness == 0.0);
    
    std::cout << "steep_extend_flat_terrain测试通过。陡峭度: " << steepness << std::endl;
    

    std::ofstream resultFile("log/steep_extend_flat_terrain_result.txt");
    if (resultFile.is_open()) {
        resultFile << "steep_extend_flat_terrain测试结果:\n";
        resultFile << "点1: (" << p1.x << ", " << p1.y << ")\n";
        resultFile << "点2: (" << p2.x << ", " << p2.y << ")\n";
        resultFile << "陡峭度: " << steepness << "\n";
        resultFile.close();
    }
}

TEST(steep_extend_with_height_difference) {

    SqPlain graph(5, 5);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            graph[i][j] = 0.0;
        }
    }
    

    graph[1][1] = 1.0;
    graph[2][2] = 2.0;
    
    SqDot p1(1, 1);
    SqDot p2(2, 2);
    
    double steepness = steep_extend(graph, p1, p2);
    

    assert(steepness >= 0);
    
    std::cout << "steep_extend_with_height_difference测试通过。陡峭度: " << steepness << std::endl;
    

    std::ofstream resultFile("log/steep_extend_with_height_difference_result.txt");
    if (resultFile.is_open()) {
        resultFile << "steep_extend_with_height_difference测试结果:\n";
        resultFile << "点1: (" << p1.x << ", " << p1.y << "), 高度: " << graph[p1.x][p1.y] << "\n";
        resultFile << "点2: (" << p2.x << ", " << p2.y << "), 高度: " << graph[p2.x][p2.y] << "\n";
        resultFile << "陡峭度: " << steepness << "\n";
        resultFile.close();
    }
}

TEST(steep_extend_with_obstacle) {

    SqPlain graph(5, 5);
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            graph[i][j] = 0.0;
        }
    }
    

    graph[1][1] = std::numeric_limits<double>::infinity();
    graph[1][2] = std::numeric_limits<double>::infinity();
    graph[2][1] = std::numeric_limits<double>::infinity();
    graph[2][2] = std::numeric_limits<double>::infinity();
    
    SqDot p1(1, 1);
    SqDot p2(2, 2);
    
    double steepness = steep_extend(graph, p1, p2);
    

    assert(steepness < 0);
    
    std::cout << "steep_extend_with_obstacle测试通过。陡峭度: " << steepness << std::endl;
    

    std::ofstream resultFile("log/steep_extend_with_obstacle_result.txt");
    if (resultFile.is_open()) {
        resultFile << "steep_extend_with_obstacle测试结果:\n";
        resultFile << "点1: (" << p1.x << ", " << p1.y << ")\n";
        resultFile << "点2: (" << p2.x << ", " << p2.y << ")\n";
        resultFile << "区域内障碍物数量: 4\n";
        resultFile << "区域总点数: 4\n";
        resultFile << "障碍物比例: 100%\n";
        resultFile << "陡峭度: " << steepness << "\n";
        resultFile.close();
    }
}

int main(int argc, char* argv[]) {

    if (argc > 1) {
        std::string workingDir = std::string(argv[1]);
        TestFramework::getInstance().setWorkingDirectory(workingDir);
        std::cout << "工作目录设置为: " << workingDir << std::endl;
    } else {
        std::cout << "未提供工作目录参数，使用相对路径" << std::endl;
    }
    

    return TestFramework::getInstance().runTests() ? 0 : 1;
}