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
    double foot_length, double foot_width): 
max_stride(max_stride),
max_turn(max_turn),
max_foot_separation(max_foot_separation),
min_foot_separation(min_foot_separation),
active_distance(max_stride * 0.4),
now_which_foot_to_move(WhichFoot::Left) {
    // 初始化足部，将足部形状信息传递给每个足部
    feet[0] = Foot(SqDot(foot_width / 2 + 1, foot_length / 2 + 1), 0.0, foot_length, foot_width);  // 左脚
    feet[1] = Foot(SqDot(foot_width / 2 + 1, foot_length / 2 + 1 + 0.6 * min_foot_separation + 0.4 * max_foot_separation),
        0.0, foot_length, foot_width);  // 右脚
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
    sw = next(new_pos);
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
 * @brief 获取当前摆动脚的const引用
 * 
 * @return 当前摆动脚的const引用
 */
const Foot& Robot::get_swing_foot() const {
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
    auto new_foot = next(new_pos);
    // 获取新位置覆盖区域的四角
    std::vector<SqDot> points = new_foot.corner();

    // 获取支撑脚的两个长边中点
    double half_width = support_foot.shape.width / 2.0;
    
    // 正向中心线+半边长模拟近侧外形线
    SqLine as_near_side_line(support_foot.position, support_foot.rz);

    double spacing = std::numeric_limits<double>::infinity();

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
    auto target = get_target(guide_point);
    auto next = bfs(ground, target);
    walk_update(next);
    return next;    
}

SqDot Robot::get_target(const SqDot& guide_point) {
    auto result = find_target(guide_point);
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

    auto& sp = get_support_foot();
    auto& sw = get_swing_foot();

    double dx = guide_point.x - sw.position.x;
    double dy = guide_point.y - sw.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // 如果目标点本身满足约束条件，直接返回
    if (position_check(guide_point)) {
        return guide_point;
    }
    
    // 从引导点向当前位置反向搜索满足约束条件的最近点
    const double step_size = 0.1; // 使用较小步长以提高精度
    for (double ratio = 1.0 - step_size; ratio > 0; ratio -= step_size) {
        SqDot test_point(sw.position.x + dx * ratio, 
                         sw.position.y + dy * ratio);
        
        if (position_check(test_point)) {
            return test_point;
        }
    }
    
    // 如果连线上没有满足约束的点，则返回当前位置
    return sw.position;
}

SqDot Robot::find_target(const SqDot& guide_point) {
    auto candidates = walk_candidates();
    
    // // 如果没有找到候选点，使用direct_target作为备选方案
    // if (candidates.empty()) {
    //     return direct_target(guide_point);
    // }
    
    auto cmp = [&](const SqDot& a, const SqDot& b) {
        return a.distance(guide_point) < b.distance(guide_point);
    };
    std::sort(candidates.begin(), candidates.end(), cmp);
    return candidates[0];
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
    // 搜索初始化
    std::vector<SqDot> path;
    auto temp = position();
    auto start = temp.as_index();
    auto target = goal.as_index();
    auto guides = scale_star(ground.map, start, target, 1.0/max_stride);
    std::reverse(guides.begin(), guides.end());
    int tick = 2 * guides.size() + 127;

    // 路径搜索
    while (!guides.empty() && tick > 0 && !reach_target(goal)) {
        while(reach_target(guides.back().as_dot())) {
            guides.pop_back();
            if (guides.empty()) return path;
        }
        auto guide = guides.back();
        SqDot next_step = walk_with_guide(ground, guide.as_dot());
        path.emplace_back(next_step);
        tick--;
    }
    return path;
}

std::vector<SqDot> Robot::rot_neighbour(const SqDot& dot, double rot, double R) { 
    auto dist = R * cos(rot);
    auto disn = R * sin(rot);
    return std::vector<SqDot> {SqDot{dot.x + dist, dot.y + disn}, SqDot{dot.x - dist, dot.y - disn},
    SqDot{dot.x + disn, dot.y - dist}, SqDot{dot.x - disn, dot.y + disn}};
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
        
        // 根据当前位置计算新的足部状态
        auto new_foot = next(now);        
        // 检查新位置是否满足约束条件
        if (!position_check(now)) continue;
        // 检查新位置是否可以稳定站立
        if (new_foot.standable(ground)) return new_foot.position;
        
        // 在添加邻居点到队列前检查它们是否在地图范围内
        for (auto& point : rot_neighbour(now, sw.rz, sw.about_R())) { 
            if (ground.is_valid(point)) {
                pends.push(point);
            }
        }
    }
    return sw.position;
}

bool Robot::reach_target(const SqDot& target) {
    return position().distance(target) < active_distance;
}

/**
 * @brief 根据新位置计算足部的下一步状态
 * 
 * @param new_pos 新位置
 * @return 计算出的足部下一步状态
 */
Foot Robot::next(const SqDot& new_pos) const {
    // 使用机器人的active_distance作为激活距离阈值
    auto& swing_foot = get_swing_foot();
    
    // 计算步长
    double stride = swing_foot.position.distance(new_pos);
    
    // 如果步长超过激活距离，则计算新的朝向角，否则保持原朝向
    double new_rz = (stride >= active_distance) ? swing_foot.position.angle(new_pos) : swing_foot.rz;
    
    return Foot(new_pos, new_rz, swing_foot.shape.length, swing_foot.shape.width);
}

std::vector<SqDot> Robot::walk_candidates() {
    std::vector<SqDot> candidates;
    
    // 获取当前摆动脚
    const Foot& swing_foot = get_swing_foot();
    const Foot& support_foot = get_support_foot();
    
    // 设置搜索参数
    const double resolution = 2.0; // 分辨率，控制搜索密度以保证性能
    const double angle_resolution = 3.14159265358979323846 / 12; // 角度分辨率
    
    // 在激活距离到最大步长范围内搜索
    for (double distance = active_distance; distance <= max_stride; distance += resolution) {
        for (double angle_offset = -max_turn; angle_offset <= max_turn; angle_offset += angle_resolution) {
            // 计算相对于支撑脚的新位置
            double angle = support_foot.rz + angle_offset;
            
            // 计算新位置坐标
            SqDot new_pos(swing_foot.position.x + distance * cos(angle),
                          swing_foot.position.y + distance * sin(angle));
            
            // 检查是否满足所有约束条件
            if (position_check(new_pos)) {
                candidates.push_back(new_pos);
            }
        }
    }
    return candidates;
}
