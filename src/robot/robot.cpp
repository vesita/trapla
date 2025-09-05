#include "robot/robot.hpp"

Robot::Robot(double max_stride, double max_turn, double max_foot_separation, double min_foot_separation):
max_stride(max_stride),
max_turn(max_turn),
max_foot_separation(max_foot_separation),
min_foot_separation(min_foot_separation), 
now_which_foot_to_move(WhichFoot::Left) {}

void Robot::walk_update() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        now_which_foot_to_move = WhichFoot::Right;
    } else {
        now_which_foot_to_move = WhichFoot::Left;
    }
}

std::vector<Point> Robot::ideal_walk(const Ground& ground) {
    std::unordered_set<Point, PointHash> area_set{};
    // 获取摆动足和支撑足
    auto& swing_foot = getSwingFoot();
    auto& support_foot = getSupportFoot();
    
    double step = 0.5;
    
    // 第一步：基于无旋转约束确定初步移动范围
    // 在支撑脚朝向方向上，考虑最大步长和双足间距约束
    double cos_rz = cos(support_foot.rz);
    double sin_rz = sin(support_foot.rz);
    // 使用三角函数公式简化：cos(θ+π/2) = -sin(θ), sin(θ+π/2) = cos(θ)
    double cos_rz_perp = -sin_rz;  // 垂直于朝向的方向
    double sin_rz_perp = cos_rz;
    
    if (now_which_foot_to_move == WhichFoot::Left) {
        // 左脚为摆动脚
        // 计算左脚相对于右脚的合理位置范围
        for (double stride = 0; stride <= max_stride; stride += step) {
            for (double separation = min_foot_separation; separation <= max_foot_separation; separation += step) {
                // 基本前进步伐位置
                double x_base = support_foot.x + stride * cos_rz;
                double y_base = support_foot.y + stride * sin_rz;
                
                // 考虑双足间距，在垂直方向上偏移
                double left_x = x_base - separation * cos_rz_perp;
                double left_y = y_base - separation * sin_rz_perp;
                
                int x_offset = static_cast<int>(round(left_x - swing_foot.x));
                int y_offset = static_cast<int>(round(left_y - swing_foot.y));
                area_set.insert(Point(x_offset, y_offset));
            }
        }
    } else {
        // 右脚为摆动脚
        // 计算右脚相对于左脚的合理位置范围
        for (double stride = 0; stride <= max_stride; stride += step) {
            for (double separation = min_foot_separation; separation <= max_foot_separation; separation += step) {
                // 基本前进步伐位置
                double x_base = support_foot.x + stride * cos_rz;
                double y_base = support_foot.y + stride * sin_rz;
                
                // 考虑双足间距，在垂直方向上偏移
                double right_x = x_base + separation * cos_rz_perp;
                double right_y = y_base + separation * sin_rz_perp;
                
                int x_offset = static_cast<int>(round(right_x - swing_foot.x));
                int y_offset = static_cast<int>(round(right_y - swing_foot.y));
                area_set.insert(Point(x_offset, y_offset));
            }
        }
    }

    std::vector<Point> area;
    
    auto shape = std::move(ground.shape());
    for (auto& point : area_set) {
        if (point.x + swing_foot.x >= 0 && point.x + swing_foot.x < shape[0] && point.y + swing_foot.y >= 0 && point.y + swing_foot.y < shape[1]) {
            area.emplace_back(point);
        }
    }
    return area;
}
std::vector<Point> Robot::about_area(Point& target) {}

Foot& Robot::getSwingFoot() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        return feet[0];  // 左脚为摆动脚
    } else {
        return feet[1];  // 右脚为摆动脚
    }
}

Foot& Robot::getSupportFoot() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        return feet[1];  // 右脚为支撑脚
    } else {
        return feet[0];  // 左脚为支撑脚
    }
}