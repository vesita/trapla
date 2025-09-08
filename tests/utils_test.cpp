#include "utils/geometry.hpp"
#include "utils/scale.hpp"
#include "utils/fast_flatness.hpp"
#include "utils/test_framework.hpp"
#include "utils/io.hpp"
#include <iostream>
#include <cassert>

TEST(geometry_basic_test) {

    SqDot a(0, 0);
    SqDot b(3, 4);
    double distance = a.distance(b);
    assert(std::abs(distance - 5.0) < 1e-6);
    
    std::cout << "geometry_basic_test: 通过" << std::endl;
    

    auto logFile = IOManager::getInstance().createOutputFile("log/geometry_basic_test.txt");
    if (logFile && logFile->is_open()) {
        *logFile << "geometry_basic_test: 通过" << std::endl;
    }
}

TEST(scale_basic_test) {

    SqPlain graph(4, 4);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            graph[i][j] = i * 4 + j;
        }
    }
    
    auto scaled = graph.scale_graph(0.5);
    assert(scaled.rows() == 2);
    assert(scaled.cols() == 2);
    
    std::cout << "scale_basic_test: 通过" << std::endl;
    

    auto logFile = IOManager::getInstance().createOutputFile("log/scale_basic_test.txt");
    if (logFile && logFile->is_open()) {
        *logFile << "scale_basic_test: 通过" << std::endl;
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