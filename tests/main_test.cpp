#include "utils/test_framework.hpp"
#include "csv/reader.hpp"
#include "csv/writer.hpp"

#include <iostream>

#include "utils/geometry.hpp"
#include "utils/index.hpp"

#include "robot/robot.hpp"
#include "aStar/aStar.hpp"
#include "aStar/direction.hpp"

TEST(main_test) {
    Robot robot(40.0, 75.0/180.0*M_PI, 10.0, 2.0, 5.0, 3.0);
    Ground graph("data/csv/map.csv");
    auto guides = scale_star(graph.map, Intex(0, 0), Intex(500, 500), 1 / 40.0);
    std::vector<SqDot> dots;
    std::vector<std::string> columnNames {"x", "y"};
    auto name = "guides";
    
    // 记录所有引导点
    for (size_t i = 0; i < guides.size(); ++i) {
        auto& dot = guides[i];
        TestFramework::getInstance().addDataRecord(name, columnNames,
                    std::vector<double>{static_cast<double>(dot.x), static_cast<double>(dot.y)});
        dots.emplace_back(dot.as_dot());
    }
    TestFramework::getInstance().writeDataRecords(name, "guides.csv");
    
    SqDot now(0, 0);
    TestFramework::getInstance().addDataRecord("direction", {"x", "y"}, {now.x, now.y});
    while (!dots.empty()) {
        now = direction_determine(now, dots);
        TestFramework::getInstance().addDataRecord("direction", {"x", "y"}, {now.x, now.y});
        dots = std::vector<SqDot>(dots.begin() + 1, dots.end());
    }
    TestFramework::getInstance().writeDataRecords("direction", "direction.csv");
    
    // 输出一些关键信息到日志
    TestFramework::getInstance().info("Total guide points: " + std::to_string(guides.size()));
    TestFramework::getInstance().info("Direction point: (" + std::to_string(now.x) + ", " + std::to_string(now.y) + ")");
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