#include "utils/test_framework.hpp"
#include "csv/reader.hpp"
#include "csv/writer.hpp"

#include "utils/geometry.hpp"
#include "utils/index.hpp"

#include "robot/robot.hpp"
#include "aStar/aStar.hpp"
#include "aStar/direction.hpp"

EXPLORATORY_TEST(main_test) {
    Robot robot(40.0, 75.0/180.0*M_PI, 10.0, 2.0, 5.0, 3.0);
    Ground graph("data/csv/map.csv");
    auto guides = scale_star(graph.map, Intex(0, 0), Intex(graph.cols() / 2, graph.rows() / 2), 1 / 40.0);
    std::vector<SqDot> dots;
    for (auto& dot : guides) {
        dots.emplace_back(dot.as_dot());
    }
    auto direction = direction_determine(SqDot(0, 0), dots);
}


int main(int argc, char* argv[]) {
    try {
        // 设置工作目录
        if (argc > 1) {
            TestFramework::getInstance().setWorkingDirectory(argv[1]);
        }
        
        TestFramework::getInstance().setLogFile("log/main_test.log");
        TestFramework::getInstance().info("=== 主要功能测试 ===");
        
        bool result = TestFramework::getInstance().runTests();
        TestFramework::getInstance().info("=== 测试完成 ===");
        
        return result ? 0 : 1;
    } catch (const std::exception& e) {
        TestFramework::getInstance().error("测试执行出错: " + std::string(e.what()));
        return 1;
    }
}