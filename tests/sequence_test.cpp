#include "utils/test_framework.hpp"
#include "utils/sequence.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <sstream>

TEST(geometric_sum_test) {
    auto& framework = TestFramework::getInstance();
    const std::string testName = "几何级数求和测试";
    
    // 测试用例1: 基本几何级数求和 a=1, q=2, n=4
    // 序列: 1, 2, 4, 8  总和 = 15
    double result1 = geometric_sum(1.0, 2.0, 4);
    double expected1 = 15.0;
    
    std::stringstream ss1;
    ss1 << "几何级数求和测试1: a=1, q=2, n=4" << std::endl
        << "  结果: " << result1 << ", 期望: " << expected1;
    framework.debug(ss1.str());
    
    if (result1 != expected1) {
        framework.addFailure(testName, {result1, expected1, 1, 1});
    }
    
    // 测试用例2: q=1的特殊情况 a=3, q=1, n=5
    // 序列: 3, 3, 3, 3, 3  总和 = 15
    double result2 = geometric_sum(3.0, 1.0, 5);
    double expected2 = 15.0;
    
    std::stringstream ss2;
    ss2 << "几何级数求和测试2: a=3, q=1, n=5" << std::endl
        << "  结果: " << result2 << ", 期望: " << expected2;
    framework.debug(ss2.str());
    
    if (result2 != expected2) {
        framework.addFailure(testName, {result2, expected2, 2, 2});
    }
    
    // 测试用例3: n=0的情况
    double result3 = geometric_sum(5.0, 2.0, 0);
    double expected3 = 0.0;
    
    std::stringstream ss3;
    ss3 << "几何级数求和测试3: a=5, q=2, n=0" << std::endl
        << "  结果: " << result3 << ", 期望: " << expected3;
    framework.debug(ss3.str());
    
    if (result3 != expected3) {
        framework.addFailure(testName, {result3, expected3, 3, 3});
    }
    
    // 测试用例4: 小数情况 a=0.5, q=0.5, n=3
    // 序列: 0.5, 0.25, 0.125  总和 = 0.875
    double result4 = geometric_sum(0.5, 0.5, 3);
    double expected4 = 0.875;
    
    std::stringstream ss4;
    ss4 << "几何级数求和测试4: a=0.5, q=0.5, n=3" << std::endl
        << "  结果: " << result4 << ", 期望: " << expected4;
    framework.debug(ss4.str());
    
    if (result4 != expected4) {
        framework.addFailure(testName, {result4, expected4, 4, 4});
    }
    
    framework.throwIfFailed(testName, "几何级数求和测试失败");
}

TEST(arithmetic_sum_test) {
    auto& framework = TestFramework::getInstance();
    const std::string testName = "算术级数求和测试";
    
    // 测试用例1: 基本算术级数求和 ab=1, ae=7, n=4
    // 序列: 1, 3, 5, 7  总和 = 16
    double result1 = arithmetic_sum(1.0, 7.0, 4);
    double expected1 = 16.0;
    
    std::stringstream ss1;
    ss1 << "算术级数求和测试1: ab=1, ae=7, n=4" << std::endl
        << "  结果: " << result1 << ", 期望: " << expected1;
    framework.debug(ss1.str());
    
    if (result1 != expected1) {
        framework.addFailure(testName, {result1, expected1, 1, 1});
    }
    
    // 测试用例2: n=0的情况
    double result2 = arithmetic_sum(1.0, 10.0, 0);
    double expected2 = 0.0;
    
    std::stringstream ss2;
    ss2 << "算术级数求和测试2: ab=1, ae=10, n=0" << std::endl
        << "  结果: " << result2 << ", 期望: " << expected2;
    framework.debug(ss2.str());
    
    if (result2 != expected2) {
        framework.addFailure(testName, {result2, expected2, 2, 2});
    }
    
    // 测试用例3: 小数情况 ab=0.5, ae=2.5, n=3
    // 序列: 0.5, 1.5, 2.5  总和 = 4.5
    double result3 = arithmetic_sum(0.5, 2.5, 3);
    double expected3 = 4.5;
    
    std::stringstream ss3;
    ss3 << "算术级数求和测试3: ab=0.5, ae=2.5, n=3" << std::endl
        << "  结果: " << result3 << ", 期望: " << expected3;
    framework.debug(ss3.str());
    
    if (result3 != expected3) {
        framework.addFailure(testName, {result3, expected3, 3, 3});
    }
    
    framework.throwIfFailed(testName, "算术级数求和测试失败");
}

int main(int argc, char* argv[]) {
    try {
        // 设置工作目录
        if (argc > 1) {
            TestFramework::getInstance().setWorkingDirectory(argv[1]);
        }
        
        // 设置日志文件
        TestFramework::getInstance().setLogFile("log/sequence_test.log");
        
        TestFramework::getInstance().info("=== 序列求和算法测试 ===");
        bool result = TestFramework::getInstance().runTests();
        TestFramework::getInstance().info("=== 测试完成 ===");
        
        return result ? 0 : 1;
    } catch (const std::exception& e) {
        TestFramework::getInstance().error("测试执行出错: " + std::string(e.what()));
        return 1;
    }
}