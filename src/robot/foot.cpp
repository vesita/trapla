#include "robot/foot.hpp"
#include "ground/ground.hpp"
#include "aStar/graph.hpp"
#include <iostream>



/**
 * @brief 默认构造函数，初始化足部形状参数为0
 */
FootShape::FootShape(): length(0), width(0) {
    return;
}

/**
 * @brief 构造函数，使用指定的长度和宽度初始化足部形状
 * @param length 足部长度
 * @param width 足部宽度
 */
FootShape::FootShape(double length, double width): length(length), width(width) {
    return;
}

/**
 * @brief 判断指定坐标点是否在足部形状内部
 * @param l_side 相对于足部中心的纵向坐标
 * @param w_side 相对于足部中心的横向坐标
 * @return 如果点在足部范围内返回true，否则返回false
 */
bool FootShape::inside(double l_side, double w_side) {
    // 假设l_side和w_side是脚部长宽
    // 检查脚部是否在给定范围内
    if (-length / 2.0 <= l_side && l_side <= length / 2.0 &&
        -width / 2.0 <= w_side && w_side <= width / 2.0) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief 计算足部覆盖的地面点集合
 * @param rz_of_foot 足部绕垂直轴的旋转角度(弧度)
 * @return 包含所有被足部覆盖的地面点坐标的向量
 * 
 * @details 该函数通过遍历足部矩形区域内的点，并将这些点从足部本地坐标系
 *          转换到世界坐标系来计算足部的覆盖区域。为了提高精度，使用0.5的步长
 *          进行采样，确保能够捕捉到足部边界上的点。使用std::unordered_set自动
 *          去除重复点，最后将结果转换为std::vector返回。
 * 
 * @note 步长选择为0.5是为了在精度和计算效率之间取得平衡。较小的步长会提高精度
 *       但会增加计算量，较大的步长会降低精度但提高计算效率。
 * 
 * @note 坐标变换使用标准的二维旋转变换公式：
 *       x' = x*cos(θ) - y*sin(θ)
 *       y' = x*sin(θ) + y*cos(θ)
 *       其中θ为足部的旋转角度rz_of_foot
 */
std::vector<Point> FootShape::cover(double& rz_of_foot) {
    // 创建一个点集合，用于存储覆盖区域中的点并自动去重
    std::unordered_set<Point, PointHash> point_set{};
    
    auto half_length = length / 2.0;
    auto half_width = width / 2.0;

    // 使用较小的步长以提高精度，确保能够捕捉到足部边界上的点
    // 步长选择0.5是为了在精度和计算效率之间取得平衡
    double scan_step = 0.5; 
    
    // 遍历足部周围区域的所有点
    for (double l = -half_length; l <= half_length; l += scan_step) {
        for (double w = -half_width; w <= half_width; w += scan_step) {
            // 将点从足部本地坐标系转换到世界坐标系（考虑足部的旋转角度rz）
            // 使用标准的二维旋转变换公式
            double world_l = l * cos(rz_of_foot) - w * sin(rz_of_foot);
            double world_w = l * sin(rz_of_foot) + w * cos(rz_of_foot);

            // 添加世界坐标系下的点到集合中，自动去重
            point_set.insert(Point(static_cast<int>(round(world_l)), static_cast<int>(round(world_w))));
        }
    }
    
    // 将集合转换为向量返回
    return std::vector<Point>(point_set.begin(), point_set.end());
}

SlideResult FootShape::slide(std::vector<Point>& area, Ground& ground) {
    // 获取地面规模信息
    auto shape = ground.shape();
    int rows = shape[0];
    int cols = shape[1];
    
    // 如果地面数据为空，返回NotApplicable
    if (rows <= 0 || cols <= 0) {
        return SlideResult::NotApplicable;
    }
    
    // 计算区域的法线
    auto normal = ground.normal(area);
    
    // 获取滑动向量
    Point slide_vector = normal.slide();
    
    // 如果滑动向量为零，说明表面水平，无需滑动
    if (slide_vector.x == 0 && slide_vector.y == 0) {
        return SlideResult::NoModification;
    }
    
    // 获取原始区域的倾角
    double original_angle = ground.stand_angle(area);
    
    // 初始化最佳选择
    Point best_slide_vector = slide_vector;
    double best_angle = original_angle;
    std::vector<Point> best_area = area;
    bool found_better = false;
    
    // 尝试不同步长的正向滑动
    const int max_iterations = 3;
    for (int i = 1; i <= max_iterations; i++) {
        std::vector<Point> new_area;
        bool all_points_valid = true;
        
        for (const auto& point : area) {
            int new_x = point.x + slide_vector.x * i;
            int new_y = point.y + slide_vector.y * i;
            
            if (new_x < 0 || new_x >= rows || new_y < 0 || new_y >= cols) {
                all_points_valid = false;
                break;
            }
            
            new_area.emplace_back(Point(new_x, new_y));
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
    
    // 如果找到了更平坦的区域，则更新
    if (found_better) {
        area = std::move(best_area);
        return SlideResult::Modified;
    }
    
    // 尝试不同步长的反向滑动
    Point reverse_slide_vector(-slide_vector.x, -slide_vector.y);
    for (int i = 1; i <= max_iterations; i++) {
        std::vector<Point> new_area;
        bool all_points_valid = true;
        
        for (const auto& point : area) {
            int new_x = point.x + reverse_slide_vector.x * i;
            int new_y = point.y + reverse_slide_vector.y * i;
            
            if (new_x < 0 || new_x >= rows || new_y < 0 || new_y >= cols) {
                all_points_valid = false;
                break;
            }
            
            new_area.emplace_back(Point(new_x, new_y));
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
    
    // 如果找到了更平坦的区域，则更新
    if (found_better) {
        area = std::move(best_area);
        return SlideResult::Modified;
    }
    
    return SlideResult::NoModification;
}


/**
 * @brief 默认构造函数，初始化足部位置和姿态为(0,0,0)
 */
Foot::Foot(): x(0), y(0), rz(0) {
    return;
}

/**
 * @brief 设置足部的位置和姿态
 * @param x 足部在世界坐标系中的x坐标
 * @param y 足部在世界坐标系中的y坐标
 * @param rz 足部绕垂直轴的旋转角度(弧度)
 */
void Foot::set(double x, double y, double rz) {
    this->x = x;
    this->y = y;
    this->rz = rz;
}

/**
 * @brief 模拟足部行走动作，检查足部是否能稳定地放置在指定地面位置
 * @param ground 地面对象，包含地形高度信息
 * @param shape 足部形状参数
 * @return 如果足部可以稳定放置返回true，否则返回false
 */
bool Foot::walkto(Ground& ground, FootShape& shape) { 
    // 检查地面数据是否为空
    if (ground.map.empty() || ground.map[0].empty()) {
        return false;
    }
    
    // 获取地面尺寸
    int rows = ground.map.size();
    int cols = ground.map[0].size();
    
    // 确保目标位置在地面范围内
    if (x < 0 || x >= rows || y < 0 || y >= cols) {
        return false;
    }
    
    // 在实际应用中，我们可能需要根据地面高度数据调整足部位置
    // 并检查足部与地面的接触角度是否符合约束条件（小于±20°）
    
    // 计算足部在地面上的姿态
    // 这里我们假设x,y是足部中心在地面坐标系中的位置
    // rz是足部绕垂直轴的旋转角度
    
    // 可以进一步实现：
    // 1. 检查足部放置区域的地形坡度
    // 2. 验证足部是否与地面稳定接触
    // 3. 确保足部角度符合机器人约束
    
    // 目前简单实现，总是返回成功
    return true;
}