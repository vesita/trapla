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
    feet[0] = Foot(SqDot(foot_width / 2 + 1,
        foot_length / 2 + 1 - 0.6 * min_foot_separation - 0.4 * max_foot_separation),
        0.0, foot_length, foot_width);  // 左脚
    feet[1] = Foot(SqDot(foot_width / 2 + 1, foot_length / 2 + 1),
        0.0, foot_length, foot_width);  // 右脚
}

void Robot::walk_update(const SqDot& new_pos) { 
    Foot& sw = get_swing_foot();
    sw = next(new_pos);
    change_feet();
}

void Robot::last_walk_update(const SqDot& new_pos) { 
    Foot& sp = get_support_foot();
    sp = last_next(new_pos);
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

const Foot& Robot::get_support_foot() const { 
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
SqDot Robot::walk_to(const Ground& ground, const SqDot& guide_point) { 
    auto target = get_target(guide_point);
    auto next = bfs(ground, target);
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
    
    auto cmp = [&](const SqDot& a, const SqDot& b) {
        auto support_foot = get_support_foot();
        auto sp = support_foot.position;
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
 * 使用引导点策略进行路径规划，通过缩放A*算法生成引导点序列，
 * 然后逐步向引导点移动，直到到达目标位置。
 * 
 * @param ground 地形对象
 * @param goal 目标点
 * @return 路径点序列
 */
std::vector<SqDot> Robot::find_path(const Ground& ground, const SqDot& goal) {
    std::vector<SqDot> path;
    
    // 获取起始位置和目标位置的索引
    Intex start = position().as_index();
    Intex target = goal.as_index();
    
    // 使用缩放A*算法生成引导点序列
    auto guides = scale_star(ground.map, start, target, 1.0/max_stride);
    
    // 反转引导点序列，使其从当前位置开始
    std::reverse(guides.begin(), guides.end());
    
    // 设置最大迭代次数，防止无限循环
    int tick = 2 * guides.size() + 127;

    // 获取初始引导点
    auto guide = direction_determine(position(), guides, true);
    select_swing_foot(guide);
    
    std::vector<SqDot> pend_dots{};
    pend_dots.reserve(2);

    int cursor = 0;
    bool waiting = false;
    // 路径搜索主循环
    while (!guides.empty() && tick > 0 && !reach_target(goal)) {
        // 检查是否已到达当前引导点，如果是则更新引导点
        while(!guides.empty() && reach_target(guide)) {
            guides.pop_back();
            guide = direction_determine(position(), guides, true);
            if (guides.empty()) return path;
        }
        
        // 根据当前引导点行走一步
        SqDot next_step = walk_to(ground, guide);

        pend_dots.push_back(next_step);
        cursor = (cursor + 1) % 2;
        waiting = cursor == 0;
        if (waiting && pend_dots.size() >= 2) {
            // 当收集到两个待定点时，检查是否会发生脚步交叉
            if (will_feet_cross(pend_dots)) {
                // 如果会发生交叉，则避免交叉
                avoid_feet_cross(ground, pend_dots);
                // 将两个新位置添加到路径中
                path.emplace_back(get_swing_foot().position);
                path.emplace_back(get_support_foot().position);
            } else {
                // 如果不会交叉，则正常更新脚步
                auto target = pend_dots.front();
                walk_update(target);
                path.emplace_back(target);
                pend_dots.clear();
            }
        }
        change_feet();
        tick--;
    }
    
    // 处理剩余的pend_dots
    for (const auto& dot : pend_dots) {
        walk_update(dot);
        path.emplace_back(dot);
    }
    
    return path;
}

std::vector<SqDot> Robot::rot_neighbour(const SqDot& dot, double rot, double R) { 
    auto dist = R * cos(rot);
    auto disn = R * sin(rot);
    return std::vector<SqDot> {
        SqDot{dot.x + dist, dot.y + disn},     // 第一个方向
        SqDot{dot.x - dist, dot.y - disn},     // 相反方向
        SqDot{dot.x + disn, dot.y - dist},     // 第二个方向
        SqDot{dot.x - disn, dot.y + dist}      // 相反方向
    };
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

double Robot::central_angle(const SqDot& dot) {
    auto angle = position().angle(dot) - ((get_support_foot().rz + get_swing_foot().rz) / 2);
    return angle;
}

SqDot Robot::guide_bias(const SqDot& dot) {
    auto angle = central_angle(dot);
    if (now_which_foot_to_move == WhichFoot::Left) {
        return dot.bias(angle + M_PI / 2, active_distance);
    } else {
        return dot.bias(angle - M_PI / 2, active_distance);
    }
}

SqDot Robot::arc_guide(const SqDot& dot) { 
    if (fit_foot(dot)) {
        return dot;
    }
    
    // 使用切线策略来寻找替代引导点
    // 以支撑脚为中心，两脚间距为半径画圆
    // 然后以摆动脚与引导点连线做圆切线，以切点为重设引导点
    
    auto swing_foot = get_swing_foot();
    auto support_foot = get_support_foot();
    
    // 计算两脚之间的距离
    double foot_separation = swing_foot.position.distance(support_foot.position);
    
    // 计算从支撑脚到引导点的角度
    double angle_to_target = support_foot.position.angle(dot);
    
    // 计算从支撑脚到摆动脚的角度
    double angle_to_swing = support_foot.position.angle(swing_foot.position);
    
    // 计算摆动脚与引导点连线和支撑脚位置的夹角
    double angle_swing_target = swing_foot.position.angle(dot);
    
    // 计算从摆动脚到引导点的距离
    double distance_swing_to_target = swing_foot.position.distance(dot);
    
    // 避免除零情况或距离过近的情况
    if (distance_swing_to_target <= foot_separation || distance_swing_to_target < 1e-6) {
        // 如果距离太近，使用简单的偏移策略
        double turn_direction = (now_which_foot_to_move == WhichFoot::Left) ? 1.0 : -1.0;
        return support_foot.position.bias(angle_to_target + turn_direction * M_PI/4, foot_separation);
    }
    
    // 计算切线角度偏移
    double angle_offset = asin(foot_separation / distance_swing_to_target);
    
    // 确定切线方向（左脚或右脚）
    double turn_direction = (now_which_foot_to_move == WhichFoot::Left) ? -1.0 : 1.0;
    
    // 计算切点位置
    double tangent_angle = angle_swing_target + turn_direction * angle_offset;
    SqDot tangent_point = swing_foot.position.bias(tangent_angle, foot_separation);
    
    // 检查切点是否合适
    if (fit_foot(tangent_point)) {
        return tangent_point;
    }
    
    // 如果第一个切点不合适，尝试另一个方向的切线
    double other_tangent_angle = angle_swing_target - turn_direction * angle_offset;
    SqDot other_tangent_point = swing_foot.position.bias(other_tangent_angle, foot_separation);
    
    if (fit_foot(other_tangent_point)) {
        return other_tangent_point;
    }
    
    // 如果两个切点都不合适，使用三等分圆弧策略作为备选
    auto current_pos = position();
    double distance = current_pos.distance(dot);
    double turn_direction2 = (now_which_foot_to_move == WhichFoot::Left) ? 1.0 : -1.0;
    
    // 生成三个候选点（三等分圆弧）
    // 第一个点：沿圆弧30度方向
    SqDot candidate1 = current_pos.bias(angle_to_target + turn_direction2 * M_PI/6, distance * 0.8);
    if (fit_foot(candidate1)) {
        return candidate1;
    }
    
    // 第二个点：沿圆弧60度方向
    SqDot candidate2 = current_pos.bias(angle_to_target + turn_direction2 * M_PI/3, distance * 0.6);
    if (fit_foot(candidate2)) {
        return candidate2;
    }
    
    // 第三个点：沿圆弧90度方向
    SqDot candidate3 = current_pos.bias(angle_to_target + turn_direction2 * M_PI/2, distance * 0.5);
    if (fit_foot(candidate3)) {
        return candidate3;
    }
    
    // 如果所有策略都不行，则返回原点
    return dot;
}

bool Robot::fit_foot(const SqDot& dot) {
    auto angle = central_angle(dot);
    if (now_which_foot_to_move == WhichFoot::Left) {
        return angle >= 0;
    } else {
        return angle <= 0;
    }
}

bool Robot::will_feet_cross(std::vector<SqDot>& pends) {
    // 确保pends至少有两个点
    if (pends.size() < 2) {
        return false;
    }
    
    // 获取当前摆动脚和支撑脚的位置
    auto sw = get_swing_foot();
    auto sp = get_support_foot();
    
    // 检查线段是否相交：从当前摆动脚到第一步目标位置的线段，
    // 与从当前支撑脚到第二步目标位置的线段是否相交
    return segments_intersect(sw.position, pends[0], sp.position, pends[1]);
}

void Robot::avoid_feet_cross(const Ground& ground, std::vector<SqDot>& pends) {
    // 确保pends至少有两个点
    if (pends.size() < 2) {
        return;
    }
    
    // 先执行第二步（当前支撑脚的移动）
    auto sp_target = walk_to(ground, pends.back());
    walk_update(sp_target);
    pends.pop_back();
    
    // 再执行第一步（当前摆动脚的移动）
    auto sw_target = walk_to(ground, pends.back());
    walk_update(sw_target);
    pends.pop_back();
}

/**
 * @brief 根据引导点选择合适的摆动脚
 * 
 * @param first_guide 第一个引导点
 */
void Robot::select_swing_foot(const SqDot& first_guide) {
    // 计算左脚到第一个引导点的距离
    double left_foot_distance = feet[0].position.distance(first_guide);
    // 计算右脚到第一个引导点的距离
    double right_foot_distance = feet[1].position.distance(first_guide);
    
    // 选择距离更近的脚作为摆动脚
    if (left_foot_distance <= right_foot_distance) {
        now_which_foot_to_move = WhichFoot::Left;
    } else {
        now_which_foot_to_move = WhichFoot::Right;
    }
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

Foot Robot::last_next(const SqDot& new_pos) const {
    // 使用机器人的active_distance作为激活距离阈值
    auto& support_foot = get_support_foot();
    
    // 计算步长
    double stride = support_foot.position.distance(new_pos);
    
    // 如果步长超过激活距离，则计算新的朝向角，否则保持原朝向
    double new_rz = (stride >= active_distance) ? support_foot.position.angle(new_pos) : support_foot.rz;
    
    return Foot(new_pos, new_rz, support_foot.shape.length, support_foot.shape.width);
}

std::vector<SqDot> Robot::walk_candidates() {
    std::vector<SqDot> candidates;
    
    // 获取当前摆动脚
    const Foot& swing_foot = get_swing_foot();
    const Foot& support_foot = get_support_foot();
    
    // 设置搜索参数
    const double resolution = 2.0; // 分辨率，控制搜索密度以保证性能
    const double angle_resolution = M_PI / 12; // 角度分辨率
    
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

void Robot::change_feet() {
    now_which_foot_to_move = (now_which_foot_to_move == WhichFoot::Left) ? WhichFoot::Right : WhichFoot::Left;
}

/**
 * @brief 根据左右脚倾向生成候选点
 * 左脚限制角度范围为-75-10度，右脚限制角度范围为-10-75度
 * 
 * @return 满足约束条件的候选点列表
 */
std::vector<SqDot> Robot::biased_walk_candidates() {
    std::vector<SqDot> candidates;
    
    // 获取当前摆动脚
    const Foot& swing_foot = get_swing_foot();
    const Foot& support_foot = get_support_foot();
    
    // 设置搜索参数
    const double resolution = 2.0; // 分辨率，控制搜索密度以保证性能
    const double angle_resolution = M_PI / 12; // 角度分辨率
    
    // 定义左右脚的角度范围（相对于支撑脚朝向）
    double min_angle_offset, max_angle_offset;
    
    // 根据当前摆动脚确定搜索角度范围
    if (now_which_foot_to_move == WhichFoot::Left) {
        min_angle_offset = -max_turn;
        max_angle_offset = 10 * M_PI / 180.0;
    } else {
        min_angle_offset = -10 * M_PI / 180.0;
        max_angle_offset = max_turn;
    }
    
    // 在激活距离到最大步长范围内搜索
    for (double distance = active_distance; distance <= max_stride; distance += resolution) {
        for (double angle_offset = min_angle_offset; angle_offset <= max_angle_offset; angle_offset += angle_resolution) {
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
