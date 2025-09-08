#include "utils/order.hpp"

void co_clowise(std::vector<SqDot>& points) {

    if (points.size() < 3) return;

    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto& p : points) {
        center_x += p.x;
        center_y += p.y;
    }
    center_x /= points.size();
    center_y /= points.size();

    std::sort(points.begin(), points.end(), [center_x, center_y](const SqDot& a, const SqDot& b) {

        double angle_a = std::atan2(a.y - center_y, a.x - center_x);
        double angle_b = std::atan2(b.y - center_y, b.x - center_x);

        return angle_a > angle_b;
    });
}