#include "utils/order.hpp"

/**
 * 将点集按逆时针方向排序
 * 
 * 该函数通过计算所有点的中心点，然后根据每个点相对于中心点的角度进行排序，
 * 实现点集的逆时针排序。适用于需要按角度顺序处理点集的场景，如凸包算法等。
 * 
 * 算法步骤：
 * 1. 计算所有点的中心点坐标
 * 2. 使用atan2函数计算每个点相对于中心点的角度
 * 3. 按角度从大到小排序（在标准坐标系中为逆时针方向）
 * 
 * 时间复杂度: O(n log n)，主要由排序操作决定
 * 空间复杂度: O(1)，只使用了常数额外空间
 * 
 * @param points 点集的引用，函数将直接修改此点集的顺序
 */
void co_clowise(std::vector<Point>& points) {
    // 少于3个点无需排序
    if (points.size() < 3) return;

    // 计算所有点的中心点坐标
    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto& p : points) {
        center_x += p.x;
        center_y += p.y;
    }
    center_x /= points.size();
    center_y /= points.size();

    // 按照相对于中心点的角度进行排序（逆时针）
    std::sort(points.begin(), points.end(), [center_x, center_y](const Point& a, const Point& b) {
        // 使用atan2计算角度，避免除法运算和无穷大问题
        // atan2返回值范围为[-π, π]，满足角度比较需求
        double angle_a = std::atan2(a.y - center_y, a.x - center_x);
        double angle_b = std::atan2(b.y - center_y, b.x - center_x);
        // 按角度从大到小排序实现逆时针排序
        return angle_a > angle_b;
    });
}