#include <iostream>
#include <vector>
#include <cmath>

#include "robot/foot.hpp"
#include "ground/ground.hpp"
#include "utils/geometry.hpp"
#include "utils/test_framework.hpp"

/**
 * @brief 测试FootShape的slide方法
 */
TEST(FootShapeSlide) {
    std::cout << "开始测试FootShape的slide方法..." << std::endl;
    
    // 加载实际的地图数据，使用相对于可执行文件的路径
    Ground ground("../../data/csv/map.csv");
    
    // 检查地图是否成功加载
    auto shape = ground.shape();
    std::cout << "地图尺寸: " << shape[0] << " x " << shape[1] << std::endl;
    
    if (shape[0] <= 0 || shape[1] <= 0) {
        std::cout << "错误: 地图数据加载失败!" << std::endl;
        throw std::runtime_error("地图数据加载失败");
    }
    
    // 选择地图中心附近的一个区域进行测试
    int center_x = shape[0] / 2;
    int center_y = shape[1] / 2;
    
    // 创建一个测试区域（包含几个点）
    std::vector<Point> testArea = {
        Point(center_x - 1, center_y - 1),
        Point(center_x + 1, center_y - 1),
        Point(center_x - 1, center_y + 1),
        Point(center_x + 1, center_y + 1)
    };
    
    // 显示原始区域
    std::cout << "\n原始区域点集:" << std::endl;
    for (const auto& point : testArea) {
        std::cout << "  (" << point.x << ", " << point.y << ")" << std::endl;
    }
    
    // 测试原始区域的法向量和倾角
    CuDot original_normal = ground.normal(testArea);
    double original_angle = ground.stand_angle(testArea);
    std::cout << "原始区域法向量: (" << original_normal.x << ", " << original_normal.y << ", " << original_normal.z << ")" << std::endl;
    std::cout << "原始区域倾角: " << original_angle << " 弧度 (" << original_angle * 180.0 / M_PI << " 度)" << std::endl;
    
    // 创建FootShape对象
    FootShape footShape(3.0, 5.0); // 3cm x 5cm的足部
    
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
    
    // 测试调整后区域的法向量和倾角
    CuDot adjusted_normal = ground.normal(testArea);
    double adjusted_angle = ground.stand_angle(testArea);
    std::cout << "调整后区域法向量: (" << adjusted_normal.x << ", " << adjusted_normal.y << ", " << adjusted_normal.z << ")" << std::endl;
    std::cout << "调整后区域倾角: " << adjusted_angle << " 弧度 (" << adjusted_angle * 180.0 / M_PI << " 度)" << std::endl;
    
    // 比较调整前后的倾角
    if (result == SlideResult::Modified) {
        double angle_diff = original_angle - adjusted_angle;
        std::cout << "\n倾角改善: " << angle_diff << " 弧度 (" << angle_diff * 180.0 / M_PI << " 度)" << std::endl;
        if (angle_diff > 0) {
            std::cout << "成功改善了区域的平坦度!" << std::endl;
        } else {
            std::cout << "虽然发生了修改，但平坦度没有改善。" << std::endl;
            throw std::runtime_error("平坦度没有改善");
        }
    }
    
    std::cout << "\nFootShape的slide方法测试完成。" << std::endl;
}

int main() {
    bool passed = TestFramework::getInstance().runTests();
    return passed ? 0 : 1;
}