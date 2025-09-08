#include "ground/ground.hpp"
#include "utils/test_framework.hpp"
#include "utils/io.hpp"
#include <iostream>
#include <cassert>
#include <fstream>

TEST(ground_scale) {

    SqPlain ground(10, 10);
    

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            ground[i][j] = i + j;
        }
    }
    
    double scale = 0.5;
    auto scaled = ground.scale_graph(scale);
    

    assert(scaled.rows() == 5);
    assert(scaled.cols() == 5);
    
    std::cout << "ground_scale测试通过。原始尺寸: " << ground.rows() << " x " << ground.cols() 
              << ", 缩放后尺寸: " << scaled.rows() << " x " << scaled.cols() << std::endl;
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