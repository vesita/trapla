#include "robot/robot.hpp"

/**
 * @brief 构造函数，初始化机器人参数
 * 
 * @param max_stride 最大步长
 * @param max_turn 最大转向角（弧度）
 * @param max_foot_separation 最大足部间距
 * @param min_foot_separation 最小足部间距
 * @param foot_length 足部长度
 * @param foot_width 足部宽度
 */
Robot::Robot(double max_stride, double max_turn, double max_foot_separation, double min_foot_separation,
    double foot_length, double foot_width) : 
max_stride(max_stride),
max_turn(max_turn),
max_foot_separation(max_foot_separation),
min_foot_separation(min_foot_separation), 
now_which_foot_to_move(WhichFoot::Left),
foot_shape(foot_length, foot_width) {}

/**
 * @brief 更新行走状态，切换支撑脚和摆动脚
 */
void Robot::walk_update() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        now_which_foot_to_move = WhichFoot::Right;
    } else {
        now_which_foot_to_move = WhichFoot::Left;
    }
}

/**
 * @brief 计算理想行走区域
 * 
 * 根据当前支撑脚的位置和朝向，计算摆动脚可能的落足点区域
 * 
 * @param ground 地形对象
 * @return 可能的落足点区域
 */
std::vector<SqDot> Robot::ideal_walk(const Ground& ground) {
    std::unordered_set<SqDot, SqDotHash> area_set{};

    auto& swing_foot = getSwingFoot();
    auto& support_foot = getSupportFoot();
    
    double step = 0.5;
    

    double cos_rz = cos(support_foot.rz);
    double sin_rz = sin(support_foot.rz);

    double cos_rz_perp = -sin_rz;
    double sin_rz_perp = cos_rz;
    
    if (now_which_foot_to_move == WhichFoot::Left) {

        for (double stride = 0; stride <= max_stride; stride += step) {
            for (double separation = min_foot_separation; separation <= max_foot_separation; separation += step) {

                double x_base = support_foot.x + stride * cos_rz;
                double y_base = support_foot.y + stride * sin_rz;
                

                double left_x = x_base - separation * cos_rz_perp;
                double left_y = y_base - separation * sin_rz_perp;
                
                int x_offset = static_cast<int>(round(left_x - swing_foot.x));
                int y_offset = static_cast<int>(round(left_y - swing_foot.y));
                area_set.insert(SqDot(x_offset, y_offset));
            }
        }
    } else {

        for (double stride = 0; stride <= max_stride; stride += step) {
            for (double separation = min_foot_separation; separation <= max_foot_separation; separation += step) {

                double x_base = support_foot.x + stride * cos_rz;
                double y_base = support_foot.y + stride * sin_rz;
                

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

/**
 * @brief 获取当前摆动脚的引用
 * 
 * @return 当前摆动脚的引用
 */
Foot& Robot::getSwingFoot() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        return feet[0];
    } else {
        return feet[1];
    }
}

/**
 * @brief 获取当前支撑脚的引用
 * 
 * @return 当前支撑脚的引用
 */
Foot& Robot::getSupportFoot() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        return feet[1];
    } else {
        return feet[0];
    }
}

/**
 * @brief 检查新位置是否满足足部限制条件
 * 
 * 检查新位置是否满足步长、足部间距等物理约束条件
 * 
 * @param new_pos 新位置
 * @return 如果满足限制条件返回true，否则返回false
 */
bool Robot::satisfy_foot_limits(const SqDot& new_pos) { 

    auto& swing_foot = getSwingFoot();
    auto& support_foot = getSupportFoot();
    

    double new_x = swing_foot.x + new_pos.x;
    double new_y = swing_foot.y + new_pos.y;
    

    double dx = new_x - support_foot.x;
    double dy = new_y - support_foot.y;
    double stride = sqrt(dx * dx + dy * dy);
    

    if (stride > max_stride) {
        return false;
    }
    

    double cos_rz = cos(support_foot.rz);
    double sin_rz = sin(support_foot.rz);

    double cos_rz_perp = -sin_rz;
    double sin_rz_perp = cos_rz;
    

    double to_new_x = new_x - support_foot.x;
    double to_new_y = new_y - support_foot.y;
    

    double separation;
    if (now_which_foot_to_move == WhichFoot::Left) {

        separation = fabs(to_new_x * cos_rz_perp + to_new_y * sin_rz_perp);
    } else {

        separation = fabs(to_new_x * cos_rz_perp + to_new_y * sin_rz_perp);
    }
    

    if (separation > max_foot_separation || separation < min_foot_separation) {
        return false;
    }
    
    return true;
}

/**
 * @brief 滑动调整足部落足区域
 * 
 * 根据地面法向量调整足部的落足区域，以获得更好的稳定性
 * 
 * @param area 足部落足区域
 * @param ground 地形对象
 * @return 滑动调整结果
 */
SlideResult Robot::slide(std::vector<SqDot>& area, Ground& ground) { 
    return foot_shape.slide(area, ground);
}

/**
 * @brief 根据引导点行走
 * 
 * @param ground 地形对象
 * @param goal 引导点
 * @return 目标落足点
 */
SqDot Robot::walk_with_guide(const Ground& ground, const SqDot& goal) { 
    
}

/**
 * @brief 调整目标点以适应地形约束
 * 
 * @param ground 地形对象
 * @param goal 原始目标点
 * @return 调整后的目标点
 */
SqDot Robot::fit_target(const Ground& ground, const SqDot& goal) { 

}

/**
 * @brief 计算直接目标点
 * 
 * 根据当前位置和目标点计算下一步的直接目标点
 * 
 * @param ground 地形对象
 * @param goal 最终目标点
 * @return 直接目标点
 */
SqDot Robot::direct_target(const Ground& ground, const SqDot& goal) { 

    auto& support_foot = getSupportFoot();
    

    double dx = goal.x - support_foot.x;
    double dy = goal.y - support_foot.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    SqDot target_point;
    

    if (distance <= max_stride) {
        target_point = goal;
    } else {

        double dir_x = dx / distance;
        double dir_y = dy / distance;
        

        double target_x = support_foot.x + max_stride * dir_x;
        double target_y = support_foot.y + max_stride * dir_y;
        

        target_point = SqDot(static_cast<int>(round(target_x)), static_cast<int>(round(target_y)));
    }
    

    return fit_target(ground, target_point);
}

/**
 * @brief 查找从当前位置到目标点的路径
 * 
 * @param ground 地形对象
 * @param goal 目标点
 * @return 路径点序列
 */
std::vector<SqDot> Robot::find_path(const Ground& ground, const SqDot& goal) {

}