#include "robot/robot.hpp"

Robot::Robot(double max_stride, double max_turn, double max_foot_separation, double min_foot_separation,
    double foot_length, double foot_width) : 
max_stride(max_stride),
max_turn(max_turn),
max_foot_separation(max_foot_separation),
min_foot_separation(min_foot_separation), 
now_which_foot_to_move(WhichFoot::Left),
foot_shape(foot_length, foot_width) {}

void Robot::walk_update() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        now_which_foot_to_move = WhichFoot::Right;
    } else {
        now_which_foot_to_move = WhichFoot::Left;
    }
}

/**
 * @brief 生成理想行走路径
 * @param ground 地面对象
 * @return 包含路径点的向量
 * 
 * @details 该方法生成从机器人当前位置到目标位置的理想行走路径，
 *          路径点表示足部的理想落足位置。
 *          
 *          算法流程：
 *          1. 获取当前摆动足和支撑足
 *          2. 基于无旋转约束确定初步移动范围
 *          3. 在支撑足朝向方向上，考虑最大步长和双足间距约束
 *          4. 遍历所有满足约束的候选位置点
 *          5. 对每个候选点检查是否满足足部限制条件
 *          6. 将满足条件的点添加到结果集合中
 *          7. 返回最终的候选位置点集合
 *          
 *          约束条件：
 *          - 最大步长约束：步长不能超过max_stride
 *          - 双足间距约束：双足间距必须在[min_foot_separation, max_foot_separation]范围内
 *          - 转向角约束：转向角度不能超过max_turn
 *          
 * @note 该方法仅生成候选位置点，不考虑路径规划和避障
 * @note 结果点是相对于摆动足当前位置的偏移量
 */
std::vector<SqDot> Robot::ideal_walk(const Ground& ground) {
    std::unordered_set<SqDot, SqDotHash> area_set{};
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
                area_set.insert(SqDot(x_offset, y_offset));
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
                area_set.insert(SqDot(x_offset, y_offset));
            }
        }
    }

    std::vector<SqDot> area;
    
    auto shape = std::move(ground.shape());
    for (auto& point : area_set) {
        if (point.x + swing_foot.x >= 0 && point.x + swing_foot.x < shape[0] && point.y + swing_foot.y >= 0 && point.y + swing_foot.y < shape[1]) {
            area.emplace_back(point);
        }
    }
    return area;
}

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

bool Robot::satisfy_foot_limits(const SqDot& new_pos) { 
    // 获取摆动足和支撑足
    auto& swing_foot = getSwingFoot();
    auto& support_foot = getSupportFoot();
    
    // 计算摆动足实际的新位置坐标
    double new_x = swing_foot.x + new_pos.x;
    double new_y = swing_foot.y + new_pos.y;
    
    // 计算从支撑足到新位置的步长
    double dx = new_x - support_foot.x;
    double dy = new_y - support_foot.y;
    double stride = sqrt(dx * dx + dy * dy);
    
    // 检查步长是否超过最大步长限制
    if (stride > max_stride) {
        return false;
    }
    
    // 计算支撑足朝向的垂直方向（根据右手定则）
    double cos_rz = cos(support_foot.rz);
    double sin_rz = sin(support_foot.rz);
    // 垂直于朝向的方向: cos(θ+π/2) = -sin(θ), sin(θ+π/2) = cos(θ)
    double cos_rz_perp = -sin_rz;
    double sin_rz_perp = cos_rz;
    
    // 计算从支撑足朝向垂直方向到新位置的预期偏移
    double to_new_x = new_x - support_foot.x;
    double to_new_y = new_y - support_foot.y;
    
    // 计算在垂直方向上的投影（即双足间距）
    // 投影公式: |a| * cos(theta) = a · b / |b|，这里b是单位向量，所以|b|=1
    // 由于我们只需要距离，所以直接计算点积即可
    double separation;
    if (now_which_foot_to_move == WhichFoot::Left) {
        // 如果左脚是摆动足，它应该在支撑足朝向的右侧（从机器人前进方向看）
        // 所以我们计算新位置在支撑足朝向垂直方向上的投影
        separation = fabs(to_new_x * cos_rz_perp + to_new_y * sin_rz_perp);
    } else {
        // 如果右脚是摆动足，它应该在支撑足朝向的左侧（从机器人前进方向看）
        // 但由于我们计算的是距离，所以同样使用fabs
        separation = fabs(to_new_x * cos_rz_perp + to_new_y * sin_rz_perp);
    }
    
    // 检查双足间距是否在允许范围内
    if (separation > max_foot_separation || separation < min_foot_separation) {
        return false;
    }
    
    return true;
}

SlideResult Robot::slide(std::vector<SqDot>& area, Ground& ground) { 
    return foot_shape.slide(area, ground);
}

SqDot Robot::walk_with_guide(const Ground& ground, const SqDot& goal) { 
    
}

SqDot Robot::fit_target(const Ground& ground, const SqDot& goal) { 

}

SqDot Robot::direct_target(const Ground& ground, const SqDot& goal) { 
    // 获取支撑足（当前支撑身体的脚）
    auto& support_foot = getSupportFoot();
    
    // 计算从支撑足到目标点的距离
    double dx = goal.x - support_foot.x;
    double dy = goal.y - support_foot.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    SqDot target_point;
    
    // 如果目标点在最大步长范围内，直接返回目标点
    if (distance <= max_stride) {
        target_point = goal;
    } else {
        // 目标点在可移动距离之外，需要选择连线上的最近点
        // 计算方向向量并归一化
        double dir_x = dx / distance;
        double dir_y = dy / distance;
        
        // 在支撑足朝向和到目标点方向之间找到一个平衡点
        // 将目标点设置在最大步长距离上
        double target_x = support_foot.x + max_stride * dir_x;
        double target_y = support_foot.y + max_stride * dir_y;
        
        // 返回计算得到的目标点
        target_point = SqDot(static_cast<int>(round(target_x)), static_cast<int>(round(target_y)));
    }
    
    // 使用fit_target方法修正落足点
    return fit_target(ground, target_point);
}

std::vector<SqDot> Robot::find_path(const Ground& ground, const SqDot& goal) {

}