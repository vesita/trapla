#include <iostream>
#include <vector>
#include <cmath>

#include "ground/ground.hpp"
#include "utils/geometry.hpp"
#include "utils/test_framework.hpp"
#include "robot/foot.hpp"

/**
 * @brief 测试地面类的trip方法
 */
TEST(GroundTrip) {
    std::cout << "开始测试地面类的trip方法..." << std::endl;
    
    // 创建一个简单的测试地图数据
    std::vector<std::vector<double>> map_data = {
        {0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 1.0, 1.0, 0.0},
        {0.0, 1.0, 2.0, 1.0, 0.0},
        {0.0, 1.0, 1.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0}
    };
    
    // 由于Ground类需要从文件读取数据，我们创建一个临时文件
    // 这里我们直接测试trip方法，模拟Ground对象
    
    // 创建测试区域点集
    std::vector<SqDot> area = {
        SqDot(1, 1),
        SqDot(1, 2),
        SqDot(1, 3),
        SqDot(2, 1),
        SqDot(2, 2),
        SqDot(2, 3),
        SqDot(3, 1),
        SqDot(3, 2),
        SqDot(3, 3)
    };
    
    // 创建Ground对象的替代方法：直接使用map数据构造
    Ground ground("");
    ground.map = map_data;
    
    // 测试trip方法
    CuPlain plane = ground.trip(area);
    
    // 验证平面法向量不为空
    CuDot normal = plane.normal_vector();
    double normal_magnitude = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    
    if (normal_magnitude == 0) {
        throw std::runtime_error("计算出的平面法向量为空");
    }
    
    std::cout << "平面法向量: (" << normal.x << ", " << normal.y << ", " << normal.z << ")" << std::endl;
    
    // 测试stand_angle方法
    double angle = ground.stand_angle(area);
    std::cout << "站立角度: " << angle << " 弧度 (" << angle * 180.0 / M_PI << " 度)" << std::endl;
    
    // 验证角度在合理范围内 (角度可能大于π/2，因为这取决于法向量方向)
    // 我们只验证角度在[0, π]范围内
    if (angle < 0 || angle > M_PI) {
        throw std::runtime_error("计算出的站立角度超出合理范围");
    }
    
    std::cout << "\n地面trip方法测试完成。" << std::endl;
}

/**
 * @brief 测试FootShape的cover方法
 */
TEST(FootShapeCover) {
    std::cout << "开始测试FootShape的cover方法..." << std::endl;
    
    // 创建FootShape对象
    FootShape footShape(2.0, 1.0); // 2cm x 1cm的足部
    
    // 测试不同角度下的覆盖区域
    double angle = 0.0; // 无旋转
    std::vector<SqDot> coverage = footShape.cover(angle);
    
    std::cout << "足部覆盖点数 (角度=" << angle << "): " << coverage.size() << std::endl;
    
    if (coverage.empty()) {
        throw std::runtime_error("足部覆盖区域为空");
    }
    
    // 显示部分覆盖点
    std::cout << "前几个覆盖点:" << std::endl;
    for (size_t i = 0; i < std::min(coverage.size(), static_cast<size_t>(5)); ++i) {
        std::cout << "  (" << coverage[i].x << ", " << coverage[i].y << ")" << std::endl;
    }
    
    // 测试不同角度
    angle = M_PI / 4; // 45度旋转
    std::vector<SqDot> coverage_rotated = footShape.cover(angle);
    
    std::cout << "足部覆盖点数 (角度=" << angle << "): " << coverage_rotated.size() << std::endl;
    
    std::cout << "\nFootShape cover方法测试完成。" << std::endl;
}

int main() {
    // 运行所有测试
    bool passed = TestFramework::getInstance().runTests();
    return passed ? 0 : 1;
}