#include "utils/test_framework.hpp"
#include "aStar/direction.hpp"
#include <vector>
#include <iostream>
#include <iomanip>

// 辅助函数：打印点信息
void printDot(const std::string& name, const SqDot& dot) {
    auto& framework = TestFramework::getInstance();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) 
       << name << ": (" << dot.x << ", " << dot.y << ")";
    framework.debug(ss.str());
}

// 辅助函数：打印引导点列表
void printGuides(const std::vector<SqDot>& guides) {
    auto& framework = TestFramework::getInstance();
    framework.debug("引导点列表:");
    for (size_t i = 0; i < guides.size(); ++i) {
        std::stringstream ss;
        ss << "  Guide[" << i << "]: (" << guides[i].x << ", " << guides[i].y << ")";
        framework.debug(ss.str());
    }
}

TEST(direction_determine_basic_test) {
    // 基本测试：简单的引导点序列
    SqDot at(0, 0);
    std::vector<SqDot> guides = {
        SqDot(10, 0),   // 第一个引导点
        SqDot(20, 0),   // 第二个引导点
        SqDot(30, 0)    // 第三个引导点
    };

    printDot("当前位置", at);
    printGuides(guides);

    SqDot result = direction_determine(at, guides);
    printDot("计算出的指向点", result);

    // 验证结果是否合理
    auto& framework = TestFramework::getInstance();
    const std::string testName = "基本方向点测试";

    // 检查结果是否在合理范围内
    // 由于使用了几何衰减，结果应该偏向第一个引导点，但应该在引导点范围内
    if (result.x < 0 || result.x > guides.back().x || result.y != 0) {
        framework.addFailure(testName, {result.x, result.y, 0, guides.back().x});
    }

    framework.throwIfFailed(testName, "基本方向点测试失败");
}

TEST(direction_determine_weight_test) {
    // 权重测试：验证不同距离引导点的权重分配
    SqDot at(0, 0);
    std::vector<SqDot> guides = {
        SqDot(10, 0),   // 近距离引导点
        SqDot(50, 0)    // 远距离引导点
    };

    printDot("当前位置", at);
    printGuides(guides);

    SqDot result = direction_determine(at, guides);
    printDot("计算出的指向点", result);

    // 验证结果在合理范围内
    auto& framework = TestFramework::getInstance();
    const std::string testName = "权重分配测试";

    // 结果应该在引导点范围内（允许一定误差）
    const double epsilon = 1e-6;
    if (result.x < -epsilon || result.x > 50 + epsilon || std::abs(result.y) > epsilon) {
        framework.addFailure(testName, {result.x, result.y, 0, 50});
    }

    framework.throwIfFailed(testName, "权重分配测试失败");
}

TEST(direction_determine_complex_test) {
    // 复杂测试：多个引导点和对角线移动
    SqDot at(0, 0);
    std::vector<SqDot> guides = {
        SqDot(10, 10),   // 对角线方向
        SqDot(20, 15),   // 稍微偏向Y轴
        SqDot(30, 20)    // 继续对角线方向
    };

    printDot("当前位置", at);
    printGuides(guides);

    SqDot result = direction_determine(at, guides);
    printDot("计算出的指向点", result);

    auto& framework = TestFramework::getInstance();
    const std::string testName = "复杂方向点测试";

    // 验证结果在合理范围内
    if (result.x < 0 || result.x > 30 || 
        result.y < 0 || result.y > 20) {
        framework.addFailure(testName, {result.x, result.y, 30, 20});
    }

    framework.throwIfFailed(testName, "复杂方向点测试失败");
}

TEST(geometric_decay_test) {
    // 测试几何衰减系数计算
    double alpha = 3.0;
    int n = 5;
    
    auto& framework = TestFramework::getInstance();
    std::stringstream ss;
    ss << "测试几何衰减系数计算 (alpha=" << alpha << ", n=" << n << ")";
    framework.info(ss.str());
    
    auto coefficients = geometric_decay(alpha, n);
    
    std::stringstream ss2;
    ss2 << "系数列表: ";
    double sum = 0.0;
    for (size_t i = 0; i < coefficients.size(); ++i) {
        ss2 << std::fixed << std::setprecision(4) << coefficients[i] << " ";
        sum += coefficients[i];
    }
    framework.debug(ss2.str());
    
    std::stringstream ss3;
    ss3 << std::fixed << std::setprecision(4) << "系数总和: " << sum;
    framework.debug(ss3.str());

    const std::string testName = "几何衰减系数测试";

    // 验证系数总和接近1（允许一定误差）
    if (std::abs(sum - 1.0) > 1e-3) {
        framework.addFailure(testName, {sum, 1.0, 0, 0});
    }

    // 验证系数数量正确
    if (coefficients.size() != n) {
        framework.addFailure(testName, {(double)coefficients.size(), (double)n, 0, 0});
    }

    // 验证系数递减特性（第一个系数最大）
    for (size_t i = 1; i < coefficients.size(); ++i) {
        if (coefficients[i] > coefficients[i-1]) {
            framework.addFailure(testName, {(double)i, coefficients[i], coefficients[i-1], 0});
        }
    }

    framework.throwIfFailed(testName, "几何衰减系数测试失败");
}

int main(int argc, char* argv[]) {
    try {
        // 设置工作目录
        if (argc > 1) {
            TestFramework::getInstance().setWorkingDirectory(argv[1]);
        }
        
        // 设置日志文件
        TestFramework::getInstance().setLogFile("log/direction_test.log");
        
        TestFramework::getInstance().info("=== 指向点算法测试 ===");
        bool result = TestFramework::getInstance().runTests();
        TestFramework::getInstance().info("=== 测试完成 ===");
        
        return result ? 0 : 1;
    } catch (const std::exception& e) {
        TestFramework::getInstance().error("测试执行出错: " + std::string(e.what()));
        return 1;
    }
}