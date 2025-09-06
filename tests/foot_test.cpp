#include <iostream>
#include <vector>
#include <cmath>

#include "robot/foot.hpp"
#include "utils/geometry.hpp"
#include "utils/test_framework.hpp"
#include "ground/ground.hpp"

/**
 * @brief 测试FootShape的slide方法
 */
TEST(FootShapeSlide) {
    std::cout << "开始测试FootShape的slide方法..." << std::endl;
    
    // 定义中心点坐标
    int center_x = 11;
    int center_y = 11;
    
    // 创建一个测试区域（包含几个点）
    std::vector<SqDot> testArea = {
        SqDot(10, 10),
        SqDot(12, 10),
        SqDot(10, 12),
        SqDot(12, 12),
        SqDot(center_x - 1, center_y - 1),
        SqDot(center_x + 1, center_y - 1),
        SqDot(center_x - 1, center_y + 1),
        SqDot(center_x + 1, center_y + 1)
    };
    
    // 显示原始区域
    std::cout << "\n原始区域点集:" << std::endl;
    for (const auto& point : testArea) {
        std::cout << "  (" << point.x << ", " << point.y << ")" << std::endl;
    }
    
    // 创建FootShape对象
    FootShape footShape(3.0, 5.0); // 3cm x 5cm的足部
    
    // 创建Ground对象用于测试
    // 创建一个简单的测试地图数据
    std::vector<std::vector<double>> map_data(20, std::vector<double>(20, 0.0));
    // 添加一些高度变化以使测试更有意义
    for (int i = 9; i <= 13; i++) {
        for (int j = 9; j <= 13; j++) {
            map_data[i][j] = (i - 11) * 0.1 + (j - 11) * 0.1;
        }
    }
    
    Ground ground("");
    ground.map = map_data;
    
    // 调用slide方法
    SlideResult result = footShape.slide(testArea, ground);
    
    // 显示slide结果
    std::cout << "\nSlide操作结果: ";
    switch (result) {
        case SlideResult::NoModification:
            std::cout << "未发生修改" << std::endl;
            break;
        case SlideResult::Modified:
            std::cout << "发生了修改" << std::endl;
            break;
        case SlideResult::NotApplicable:
            std::cout << "不适用" << std::endl;
            break;
    }
    
    // 显示调整后的区域
    std::cout << "调整后区域点集:" << std::endl;
    for (const auto& point : testArea) {
        std::cout << "  (" << point.x << ", " << point.y << ")" << std::endl;
    }
    
    std::cout << "\nFootShape的slide方法测试完成。" << std::endl;
}

int main() {
    // 运行所有测试
    bool passed = TestFramework::getInstance().runTests();
    return passed ? 0 : 1;
}