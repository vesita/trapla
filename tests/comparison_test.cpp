#include "aStar/aStar.hpp"
#include "utils/test_framework.hpp"
#include "utils/geometry.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <fstream>
#include <filesystem>

TEST(comparison_test) {

    SqPlain graph(10, 10);
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            graph[i][j] = 0.0;
        }
    }
    

    graph[2][2] = std::numeric_limits<double>::infinity();
    graph[2][3] = std::numeric_limits<double>::infinity();
    graph[3][2] = std::numeric_limits<double>::infinity();
    

    SqDot start(0, 0);
    SqDot goal(9, 9);
    

    auto start_time = std::chrono::high_resolution_clock::now();
    auto a_star_path = a_star_search(graph, start, goal);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto a_star_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    

    double scale = 5.0;
    SqPlain scaled_graph = graph.scale_graph(scale);
    

    SqDot scaled_start = start.scale(scale);
    SqDot scaled_goal = goal.scale(scale);
    
    start_time = std::chrono::high_resolution_clock::now();
    auto scale_star_path = scale_star(scaled_graph, scaled_start, scaled_goal, 1.0);
    end_time = std::chrono::high_resolution_clock::now();
    auto scale_star_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    

    assert(!a_star_path.empty());
    assert(!scale_star_path.empty());
    assert(a_star_path.front() == start);
    assert(a_star_path.back() == goal);

    assert(scale_star_path.front() == start);
    

    std::cout << "普通A*算法:" << std::endl;
    std::cout << "  路径点数: " << a_star_path.size() << std::endl;
    std::cout << "  执行时间: " << a_star_duration.count() << " 微秒" << std::endl;
    std::cout << "  起点: (" << a_star_path.front().x << ", " << a_star_path.front().y << ")" << std::endl;
    std::cout << "  终点: (" << a_star_path.back().x << ", " << a_star_path.back().y << ")" << std::endl;
    
    std::cout << "Scale*算法:" << std::endl;
    std::cout << "  路径点数: " << scale_star_path.size() << std::endl;
    std::cout << "  执行时间: " << scale_star_duration.count() << " 微秒" << std::endl;
    std::cout << "  起点: (" << scale_star_path.front().x << ", " << scale_star_path.front().y << ")" << std::endl;
    std::cout << "  实际终点: (" << scale_star_path.back().x << ", " << scale_star_path.back().y << ")" << std::endl;
    std::cout << "  期望终点: (" << scaled_goal.x << ", " << scaled_goal.y << ")" << std::endl;
    

    bool a_star_continuous = true;
    for (size_t i = 1; i < a_star_path.size(); ++i) {
        int dx = std::abs(a_star_path[i].x - a_star_path[i-1].x);
        int dy = std::abs(a_star_path[i].y - a_star_path[i-1].y);
        if (dx + dy != 1) {
            a_star_continuous = false;
            break;
        }
    }
    
    std::cout << "普通A*路径连续性: " << (a_star_continuous ? "是" : "否") << std::endl;
    

    std::cout << "路径长度比较:" << std::endl;
    std::cout << "  普通A*路径长度: " << a_star_path.size() << " 个点" << std::endl;
    std::cout << "  Scale*路径长度: " << scale_star_path.size() << " 个点" << std::endl;
    

    std::cout << "普通A*前10个路径点:" << std::endl;
    for (size_t i = 0; i < std::min(static_cast<size_t>(10), a_star_path.size()); ++i) {
        std::cout << "  " << i << ": (" << a_star_path[i].x << ", " << a_star_path[i].y << ")" << std::endl;
    }
    
    std::cout << "Scale*所有路径点:" << std::endl;
    for (size_t i = 0; i < scale_star_path.size(); ++i) {
        std::cout << "  " << i << ": (" << scale_star_path[i].x << ", " << scale_star_path[i].y << ")" << std::endl;
    }
    

    std::filesystem::create_directories("log");
    

    std::ofstream mapFile("log/map_data.csv");
    mapFile << "x,y,elevation\n";
    for (int i = 0; i < graph.rows(); i++) {
        for (int j = 0; j < graph.cols(); j++) {
            double elevation = std::isinf(graph[i][j]) ? -1 : graph[i][j];
            mapFile << i << "," << j << "," << elevation << "\n";
        }
    }
    mapFile.close();
    

    std::ofstream aStarPathFile("log/a_star_path.csv");
    aStarPathFile << "x,y\n";
    for (const auto& point : a_star_path) {
        aStarPathFile << point.x << "," << point.y << "\n";
    }
    aStarPathFile.close();
    

    std::ofstream scaleStarPathFile("log/scale_star_path.csv");
    scaleStarPathFile << "x,y\n";
    for (const auto& point : scale_star_path) {
        scaleStarPathFile << point.x << "," << point.y << "\n";
    }
    scaleStarPathFile.close();
    

    std::ofstream scaledMapFile("log/scaled_map_data.csv");
    scaledMapFile << "x,y,elevation\n";
    for (int i = 0; i < scaled_graph.rows(); i++) {
        for (int j = 0; j < scaled_graph.cols(); j++) {
            double elevation = std::isinf(scaled_graph[i][j]) ? -1 : scaled_graph[i][j];
            scaledMapFile << i << "," << j << "," << elevation << "\n";
        }
    }
    scaledMapFile.close();
    

    std::ofstream scaledPathFile("log/scaled_star_path.csv");
    scaledPathFile << "x,y\n";
    for (const auto& point : scale_star_path) {
        scaledPathFile << point.x << "," << point.y << "\n";
    }
    scaledPathFile.close();
    
    std::cout << "\n数据已保存到log目录下的CSV文件中:" << std::endl;
    std::cout << "  - map_data.csv: 原始地图数据" << std::endl;
    std::cout << "  - scaled_map_data.csv: 放大后的地图数据" << std::endl;
    std::cout << "  - a_star_path.csv: A*算法路径点" << std::endl;
    std::cout << "  - scale_star_path.csv: Scale*算法路径点" << std::endl;
    std::cout << "  - scaled_star_path.csv: Scale*算法在放大地图上的路径点" << std::endl;
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