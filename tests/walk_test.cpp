#include "utils/test_framework.hpp"
#include "robot/robot.hpp"
#include "ground/ground.hpp"
#include <iostream>
#include <vector>
#include <limits>

TEST(walk_with_guide_simple_test) {
    // 创建一个简单的测试地图
    // 0 0 0 0 0 0 0
    // 0 0 0 0 0 0 0
    // 0 0 1 1 1 0 0
    // 0 0 0 0 0 0 0
    // 0 0 0 0 0 0 0
    Ground ground(5, 7);
    ground.map[2][2] = std::numeric_limits<double>::infinity(); // 障碍物
    ground.map[2][3] = std::numeric_limits<double>::infinity(); // 障障物
    ground.map[2][4] = std::numeric_limits<double>::infinity(); // 障碍物
    
    // 创建机器人
    Robot robot(20.0, M_PI * 75.0/180.0, 10.0, 2.0, 5.0, 3.0);
    robot.feet[0].position = SqDot(1, 1);  // 左脚
    robot.feet[1].position = SqDot(1, 4);  // 右脚
    robot.now_which_foot_to_move = WhichFoot::Left; // 移动左脚
    
    // 设置引导点
    SqDot guide_point(4, 3);
    
    auto& framework = TestFramework::getInstance();
    const std::string testName = "walk_with_guide简单测试";
    
    framework.info("walk_with_guide_simple_test: 开始测试walk_with_guide函数");
    
    // 调用walk_with_guide函数
    SqDot result = robot.walk_with_guide(ground, guide_point);
    
    framework.info("walk_with_guide_simple_test: 目标点(" + std::to_string(guide_point.x) + ", " + 
                   std::to_string(guide_point.y) + "), 实际结果(" + std::to_string(result.x) + ", " + 
                   std::to_string(result.y) + ")");
    
    // 基本验证
    // 1. 检查结果点是否在地图范围内
    if (!ground.is_valid(result)) {
        framework.addFailure(testName, {0, 0, 1, 0}); // 超出地图范围
    }
    
    // 2. 检查结果点是否不是障碍物
    if (ground.obstacle(result.x, result.y)) {
        framework.addFailure(testName, {0, 1, 0, 1}); // 落在障碍物上
    }
    
    // 3. 检查结果点是否满足约束条件
    if (!robot.position_check(result)) {
        framework.addFailure(testName, {0, 2, 0, 2}); // 不满足约束条件
    }
    
    // 定义列名
    std::vector<std::string> columnNames = {"test_case", "error_type", "expected", "actual"};
    
    // 写入失败数据到CSV文件
    framework.writeFailures(testName, "walk_with_guide_failures.csv", columnNames);
    
    // 如果有失败的测试用例，则抛出异常
    framework.throwIfFailed(testName, "测试失败");
    
    framework.info("walk_with_guide_simple_test: 通过所有测试用例");
}

int main(int argc, char* argv[]) {
    try {
        // 设置工作目录
        if (argc > 1) {
            TestFramework::getInstance().setWorkingDirectory(argv[1]);
        }
        
        TestFramework::getInstance().setLogFile("log/walk_test.log");
        TestFramework::getInstance().info("=== walk_with_guide测试 ===");
        
        bool result = TestFramework::getInstance().runTests();
        TestFramework::getInstance().info("=== 测试完成 ===");
        
        return result ? 0 : 1;
    } catch (const std::exception& e) {
        TestFramework::getInstance().error("测试执行出错: " + std::string(e.what()));
        return 1;
    }
}