#include "utils/test_framework.hpp"
#include "robot/robot.hpp"
#include <iostream>
#include <vector>
#include <tuple>

TEST(spacing_constraint_test) {
    // 测试足部间距约束检查
    Robot robot(40, M_PI * 75/180, 10, 2, 5, 3);
    robot.feet[0].position = SqDot(50, 50);
    robot.feet[1].position = SqDot(50, 45);
    robot.now_which_foot_to_move = WhichFoot::Right;
    
    std::vector<std::tuple<SqDot, bool>> test_cases = {
        {SqDot(50, 55), false},
        {SqDot(50, 58), true},
        {SqDot(50, 52), false},
        {SqDot(50, 60), true},
        {SqDot(50, 51), false},
        {SqDot(44, 40), true},
        {SqDot(65, 59), true},
        {SqDot(75, 40), true},
        {SqDot(72, 46), false},
        {SqDot(62, 50), false},
        {SqDot(58, 57), true}
    };
    
    // 使用测试框架的失败数据收集功能
    auto& framework = TestFramework::getInstance();
    const std::string testName = "足部间距约束测试";
    
    framework.info("spacing_constraint_test: 开始测试足部间距约束");
    
    for (auto& test_case : test_cases) {
        SqDot pos = std::get<0>(test_case);
        bool expected = std::get<1>(test_case);
        
        bool actual = robot.satisfy_spacing(pos);
        
        if (expected != actual) {
            // 记录失败数据
            framework.addFailure(testName, {(double)pos.x, (double)pos.y, (double)expected, (double)actual});
        }
    }
    
    // 定义列名
    std::vector<std::string> columnNames = {"position_x", "position_y", "expected", "actual"};
    
    // 写入失败数据到CSV文件
    framework.writeFailures(testName, "spacing_constraint_failures.csv", columnNames);
    
    // 如果有失败的测试用例，则抛出异常
    framework.throwIfFailed(testName, "测试失败");
    
    framework.info("spacing_constraint_test: 通过所有测试用例");
}

int main (int argc, char* argv[]) {
    try {
        // 设置工作目录
        if (argc > 1) {
            TestFramework::getInstance().setWorkingDirectory(argv[1]);
        }
        
        TestFramework::getInstance().setLogFile("log/constraints_test.log");
        TestFramework::getInstance().info("=== 约束条件测试 ===");
        
        bool result = TestFramework::getInstance().runTests();
        TestFramework::getInstance().info("=== 测试完成 ===");
        
        return result ? 0 : 1;
    } catch (const std::exception& e) {
        TestFramework::getInstance().error("测试执行出错: " + std::string(e.what()));
        return 1;
    }
}