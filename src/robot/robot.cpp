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
now_which_foot_to_move(WhichFoot::Left) {
    // 初始化足部，将足部形状信息传递给每个足部
    feet[0] = Foot(SqDot(0.0, 0.0), 0.0, foot_length, foot_width);  // 左脚
    feet[1] = Foot(SqDot(0.0, 0.0), 0.0, foot_length, foot_width);  // 右脚
}

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

void Robot::walk_update(const SqDot& new_pos) { 
    Foot& sw = get_swing_foot();
    sw = sw.next(new_pos);
    walk_update();
}

double& Robot::sw_x() {
    return get_swing_foot().position.x;
}

double& Robot::sw_y() { 
    return get_swing_foot().position.y;
}

double& Robot::sw_rz() { 
    return get_swing_foot().rz;
}

double& Robot::sp_x() { 
    return get_support_foot().position.x;
}

double& Robot::sp_y() { 
    return get_support_foot().position.y;
}

double& Robot::sp_rz() { 
    return get_support_foot().rz;
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

    auto& swing_foot = get_swing_foot();
    auto& support_foot = get_support_foot();
    
    double step = 0.5;
    

    double cos_rz = cos(support_foot.rz);
    double sin_rz = sin(support_foot.rz);

    double cos_rz_perp = -sin_rz;
    double sin_rz_perp = cos_rz;
    
    if (now_which_foot_to_move == WhichFoot::Left) {

        for (double stride = 0.0; stride <= max_stride; stride += step) {
            for (double separation = min_foot_separation; separation <= max_foot_separation; separation += step) {

                double x_base = support_foot.position.x + stride * cos_rz;
                double y_base = support_foot.position.y + stride * sin_rz;
                

                double left_x = x_base - separation * cos_rz_perp;
                double left_y = y_base - separation * sin_rz_perp;
                
                double x_offset = left_x - swing_foot.position.x;
                double y_offset = left_y - swing_foot.position.y;
                area_set.insert(SqDot(round(x_offset), round(y_offset)));
            }
        }
    } else {

        for (double stride = 0.0; stride <= max_stride; stride += step) {
            for (double separation = min_foot_separation; separation <= max_foot_separation; separation += step) {

                double x_base = support_foot.position.x + stride * cos_rz;
                double y_base = support_foot.position.y + stride * sin_rz;
                

                double right_x = x_base + separation * cos_rz_perp;
                double right_y = y_base + separation * sin_rz_perp;
                
                double x_offset = right_x - swing_foot.position.x;
                double y_offset = right_y - swing_foot.position.y;
                area_set.insert(SqDot(round(x_offset), round(y_offset)));
            }
        }
    }

    std::vector<SqDot> area;
    
    auto shape = std::move(ground.shape());
    for (auto& point : area_set) {
        if (point.x + swing_foot.position.x >= 0 && point.x + swing_foot.position.x < shape[0] && point.y + swing_foot.position.y >= 0 && point.y + swing_foot.position.y < shape[1]) {
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
Foot& Robot::get_swing_foot() {
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
Foot& Robot::get_support_foot() {
    if (now_which_foot_to_move == WhichFoot::Left) {
        return feet[1];
    } else {
        return feet[0];
    }
}


double Robot::distance(const SqDot& new_pos) {
    return get_swing_foot().position.distance(new_pos);
}

/**
 * @brief 检查新位置是否满足足部限制条件
 * 
 * 检查新位置是否满足步长、足部间距等物理约束条件
 * 
 * @param new_pos 新位置
 * * @return 如果满足限制条件返回true，否则返回false
 */
bool Robot::satisfy_spacing(const SqDot& new_pos) { 
    auto swing_foot = get_swing_foot();
    auto support_foot = get_support_foot();
    auto new_foot = swing_foot.next(new_pos);
    // 获取新位置覆盖区域的四角
    std::vector<SqDot> points = new_foot.corner();

    // 获取支撑脚的两个长边中点
    double half_width = support_foot.shape.width / 2.0;
    
    // 正向中心线+半边长模拟近侧外形线
    SqLine as_near_side_line(support_foot.position, support_foot.rz);

    double spacing = INFINITY;

    // 获取四角中离近侧形线最近的点（哪怕这里不是近侧外形线，最近点是相同的）
    for (const auto& point : points) {
        spacing = std::min(spacing, as_near_side_line.distance(point));
    }
    
    // 模拟近侧外形线
    spacing = std::max(0.0, spacing - half_width);

    // 使用epsilon处理浮点数比较问题
    const double epsilon = 1e-3;
    double min_allowed = min_foot_separation - epsilon;
    double max_allowed = max_foot_separation + epsilon;
    
    return spacing >= min_allowed && spacing <= max_allowed;

    // 需求判断逻辑：
    // 近侧外形线：支撑脚靠近摆动脚的长边所处于的线
    // 计算摆动脚覆盖区域中到近侧外形线最近的点的距离需要满足max与min约束
}

bool Robot::satisfy_stride(const SqDot& new_pos) {
    return distance(new_pos) < max_stride;
}

bool Robot::satisfy_turn(const SqDot& new_pos) {
    auto angle = get_swing_foot().position.angle(new_pos) - get_support_foot().rz;
    return angle < max_turn && angle > - max_turn;
}

bool Robot::position_check(const SqDot& new_pos) {
    return satisfy_spacing(new_pos) && satisfy_stride(new_pos) && satisfy_turn(new_pos);
}

SqDot Robot::position() {
    auto sw = get_swing_foot();
    auto su = get_support_foot();
    return sw.position.center(su.position);
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
    return get_swing_foot().shape.slide(area, ground);
}

/**
 * @brief 根据引导点行走
 * 
 * @param ground 地形对象
 * @param goal 引导点
 * @return 目标落足点
 */
SqDot Robot::walk_with_guide(const Ground& ground, const SqDot& guide_point) { 
    auto target = get_target(ground, guide_point);
    return bfs(ground, target);
}

SqDot Robot::get_target(const Ground& ground, const SqDot& guide_point) {
    auto result = direct_target(guide_point);
    auto sw = get_swing_foot();
    if (result.distance(sw.position) < 0.3 * sw.about_R()) {
        return little_step();
    }
    return result;
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
SqDot Robot::direct_target(const SqDot& guide_point) { 

    auto& support_foot = get_support_foot();
    auto& sw = get_swing_foot();

    double dx = guide_point.x - sw.position.x;
    double dy = guide_point.y - sw.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // 如果目标点本身满足约束条件，直接返回
    if (position_check(guide_point)) {
        return guide_point;
    }
    
    // 在从当前位置到目标点的连线上搜索满足约束条件的最近点
    const double step_size = 0.1; // 搜索步长
    for (double ratio = step_size; ratio < 1.0; ratio += step_size) {
        SqDot test_point(sw.position.x + dx * ratio, 
                         sw.position.y + dy * ratio);
        
        if (position_check(test_point)) {
            return test_point;
        }
    }
    
    // 如果连线上没有满足约束的点，则返回当前位置
    return sw.position;
}

SqDot Robot::little_step() {
    auto& sw = get_swing_foot();
    auto dist = max_stride * cos(sw.rz) * 0.5;
    auto disn = max_stride * sin(sw.rz) * 0.5;
    return SqDot(sw.position.x + dist, sw.position.y + disn);
}

/**
 * @brief 查找从当前位置到目标点的路径
 * 
 * @param ground 地形对象
 * @param goal 目标点
 * @return 路径点序列
 */
std::vector<SqDot> Robot::find_path(const Ground& ground, const SqDot& goal) {
    std::vector<SqDot> path;
    auto guides = scale_star(ground.map, position().as_index(), goal.as_index(), 1.0/max_stride);
    std::reverse(guides.begin(), guides.end());
    int tick = 2 * guides.size() + 127;
    while (!guides.empty()) {
        auto guide = guides.back();
        path.emplace_back(walk_with_guide(ground, guide.as_dot()));
        if (path.back().distance(guide.as_dot()) < max_stride * 0.6) {
            guides.pop_back();
        }
        walk_update(path.back());
        if (--tick < 0) break;
    }
    return path;
}

std::vector<SqDot> Robot::rot_neighbour(const SqDot& dot, double rot, double R) { 
    auto dist = R * cos(rot);
    auto disn = R * sin(rot);
    return std::vector<SqDot> {SqDot{dot.x + dist, dot.y + disn}, SqDot{dot.x - dist, dot.y - disn},
    SqDot{dot.x + disn, dot.y - dist}, SqDot{dot.x - disn, dot.y + dist}};
}

SqDot Robot::bfs(const Ground& ground, const SqDot& target) { 
    auto sw = get_swing_foot();
    // 检查目标点是否在地图范围内
    if (!ground.is_valid(target)) {
        return sw.position; // 返回当前摆动脚位置
    }
    int tick = 100;
    std::queue<SqDot> pends;
    std::unordered_set<SqDot, SqDotHash> visited;
    pends.push(target);
    while (!pends.empty() && --tick > 0) { 
        auto now = pends.front();
        pends.pop();
        if (visited.count(now)) continue;
        visited.insert(now);
        
        // 检查当前点是否在地图范围内
        if (!ground.is_valid(now)) continue;
        
        auto new_pos = sw.next(now);        
        if (!position_check(now)) continue;
        if (sw.standable(ground)) return new_pos.position;
        
        // 在添加邻居点到队列前检查它们是否在地图范围内
        for (auto& point : rot_neighbour(now, sw.rz, sw.about_R())) { 
            if (ground.is_valid(point)) {
                pends.push(point);
            }
        }
    }
    return sw.position;
}