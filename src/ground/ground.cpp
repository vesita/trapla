#include "ground/ground.hpp"

/**
 * @brief 地面类构造函数，从指定文件读取地形数据
 * @param filename 包含地形高度数据的CSV文件路径
 */
Ground::Ground(std::string filename) {
    Reader reader;
    try {
        if (!reader.readFromFile(filename)) {
            std::cout << "Error: Could not read file " << filename << std::endl;
            return;
        } else {
            map = reader.getData();
        }
    } catch (std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}

/**
 * @brief 计算指定区域的站立角度
 * @param area 包含地面点坐标的向量
 * @return 区域的站立角度（弧度）
 */
double Ground::stand_angle(std::vector<Point>& area) {
    CuPlain plaine = trip(area);
    return plaine.normal_angle();
}

std::array<int, 2> Ground::shape() const {
    if (map.empty() || map[0].empty()) {
        return {0, 0};
    }
    return {static_cast<int>(map.size()), static_cast<int>(map[0].size())};
}

/**
 * @brief 通过指定区域的点计算最优平面
 * @param area 包含地面点坐标的向量
 * @return 表示最优平面的CuPlain对象
 * 
 * @details 该函数通过以下步骤计算最优平面：
 *          1. 将输入的点转换为包含高度信息的三维点
 *          2. 按照z坐标（高度）对点进行排序
 *          3. 选择最高的三个点作为初始平面
 *          4. 遍历剩余的点，如果发现有更高的点在当前平面的上方，
 *             则用该点替换当前平面的一个点并重新计算平面
 *          5. 返回最终的最优平面
 * 
 * @note 该算法的目标是找到一个尽可能贴近最高点的平面，用于评估足部放置的稳定性
 */
CuPlain Ground::trip(std::vector<Point>& area) { 
    std::vector<CuDot> dots;
    for (const auto& point : area) {
        if (point.x < 0 || point.x >= map.size() || point.y < 0 || point.y >= map[0].size()) {
            return CuPlain(); // 点超出地图范围
        }
        dots.emplace_back(CuDot{static_cast<double>(point.x), static_cast<double>(point.y), map[point.x][point.y]});
    }
    auto height_cmp = [](const CuDot& a, const CuDot& b) { return a.z < b.z; };
    std::sort(dots.begin(), dots.end(), height_cmp);
    
    // 修复: 从排序后的vector中获取前三个元素(最高的三个点)
    std::array<CuDot, 3> results = {
        dots[dots.size() - 1], 
        dots[dots.size() - 2], 
        dots[dots.size() - 3]
    };
    for (auto count = 0; count < 3; count++) {
        dots.pop_back();
    }
    CuPlain plaine;
    plaine.define_plaine(results);
    while (!dots.empty()) {
        CuPos pos = plaine.get_pos(dots.back());
        if (pos == CuPos::Above) {
            results[0] = dots.back();
            dots.pop_back();
            plaine.define_plaine(results);
        } else {
            dots.pop_back();
        }
    }
    return plaine;
}

/**
 * @brief 通过指定区域的点计算最优平面的法线
 * @param area 包含地面点坐标的向量
 * @return 表示最优平面法线的CuDot对象
 */
CuDot Ground::normal(std::vector<Point>& area) {
    CuPlain plaine = trip(area);
    return plaine.normal_vector();
}

/**
 * @brief 通过指定区域的点计算凸面（未实现）
 * @param area 包含地面点坐标的向量
 * @return 表示凸面的CuPlain对象
 */
CuPlain Ground::convex_trip(std::vector<Point>& area) { 
    // TODO: 实现convex_trip函数
    return CuPlain();
}