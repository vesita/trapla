#include "robot/foot.hpp"

/**
 * @brief 默认构造函数，创建一个空的足部形状对象
 */
FootShape::FootShape(): length(0), width(0) {
    return;
}

/**
 * @brief 构造函数，创建指定尺寸的足部形状对象
 * 
 * @param length 足部长度
 * @param width 足部宽度
 */
FootShape::FootShape(double length, double width): length(length), width(width) {
    return;
}

/**
 * @brief 检查指定点是否在足部范围内
 * 
 * @param l_side 长度方向坐标
 * @param w_side 宽度方向坐标
 * @return 如果点在足部范围内返回true，否则返回false
 */
bool FootShape::inside(double l_side, double w_side) {

    if (-length / 2.0 <= l_side && l_side <= length / 2.0 &&
        -width / 2.0 <= w_side && w_side <= width / 2.0) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief 计算足部在特定朝向下的覆盖区域
 * 
 * @param rz_of_foot 足部朝向角（弧度）
 * @return 足部覆盖的点集合
 */
std::vector<SqDot> FootShape::cover(double& rz_of_foot) {

    std::unordered_set<SqDot, SqDotHash> point_set{};
    
    auto half_length = length / 2.0;
    auto half_width = width / 2.0;

    double scan_step = 0.5; 
    

    for (double l = -half_length; l <= half_length; l += scan_step) {
        for (double w = -half_width; w <= half_width; w += scan_step) {

            double world_l = l * cos(rz_of_foot) - w * sin(rz_of_foot);
            double world_w = l * sin(rz_of_foot) + w * cos(rz_of_foot);

            point_set.insert(SqDot(static_cast<int>(round(world_l)), static_cast<int>(round(world_w))));
        }
    }
    

    return std::vector<SqDot>(point_set.begin(), point_set.end());
}

/**
 * @brief 滑动调整足部落足区域以提高稳定性
 * 
 * 根据地面法向量调整足部的落足区域，以获得更好的稳定性
 * 
 * @param area 足部落足区域
 * @param ground 地形对象
 * @return 滑动调整结果
 */
SlideResult FootShape::slide(std::vector<SqDot>& area, Ground& ground) {

    auto shape = ground.shape();
    int rows = shape[0];
    int cols = shape[1];
    

    if (rows <= 0 || cols <= 0) {
        return SlideResult::NotApplicable;
    }
    

    auto normal = ground.normal(area);
    

    SqDot slide_vector = normal.slide();
    

    if (slide_vector.x == 0 && slide_vector.y == 0) {
        return SlideResult::NoModification;
    }
    

    double original_angle = ground.stand_angle(area);
    

    SqDot best_slide_vector = slide_vector;
    double best_angle = original_angle;
    std::vector<SqDot> best_area = area;
    bool found_better = false;
    

    const int max_iterations = 3;
    for (int i = 1; i <= max_iterations; i++) {
        std::vector<SqDot> new_area;
        bool all_points_valid = true;
        
        for (const auto& point : area) {
            int new_x = point.x + slide_vector.x * i;
            int new_y = point.y + slide_vector.y * i;
            
            if (new_x < 0 || new_x >= rows || new_y < 0 || new_y >= cols) {
                all_points_valid = false;
                break;
            }
            
            new_area.emplace_back(SqDot(new_x, new_y));
        }
        
        if (all_points_valid) {
            double new_angle = ground.stand_angle(new_area);
            
            if (new_angle < best_angle) {
                best_angle = new_angle;
                best_area = std::move(new_area);
                found_better = true;
            }
        }
    }
    

    if (found_better) {
        area = std::move(best_area);
        return SlideResult::Modified;
    }
    

    SqDot reverse_slide_vector(-slide_vector.x, -slide_vector.y);
    for (int i = 1; i <= max_iterations; i++) {
        std::vector<SqDot> new_area;
        bool all_points_valid = true;
        
        for (const auto& point : area) {
            int new_x = point.x + reverse_slide_vector.x * i;
            int new_y = point.y + reverse_slide_vector.y * i;
            
            if (new_x < 0 || new_x >= rows || new_y < 0 || new_y >= cols) {
                all_points_valid = false;
                break;
            }
            
            new_area.emplace_back(SqDot(new_x, new_y));
        }
        
        if (all_points_valid) {
            double new_angle = ground.stand_angle(new_area);
            
            if (new_angle < best_angle) {
                best_angle = new_angle;
                best_area = std::move(new_area);
                found_better = true;
            }
        }
    }
    

    if (found_better) {
        area = std::move(best_area);
        return SlideResult::Modified;
    }
    
    return SlideResult::NoModification;
}

/**
 * @brief 默认构造函数，创建一个位于原点的足部对象
 */
Foot::Foot(): x(0), y(0), rz(0) {
    return;
}

/**
 * @brief 设置足部位置和朝向
 * 
 * @param x x坐标
 * @param y y坐标
 * @param rz 朝向角（弧度）
 */
void Foot::set(double x, double y, double rz) {
    this->x = x;
    this->y = y;
    this->rz = rz;
}

/**
 * @brief 让足部走向指定位置
 * 
 * @param ground 地形对象
 * @param shape 足部形状
 * @return 如果成功走向指定位置返回true，否则返回false
 */
bool Foot::walkto(Ground& ground, FootShape& shape) { 

    if (ground.empty()) {
        return false;
    }
    

    int rows = ground.map.rows();
    int cols = ground.map.cols();
    

    if (x < 0 || x >= rows || y < 0 || y >= cols) {
        return false;
    }

    return true;
}