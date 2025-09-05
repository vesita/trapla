#include "aStar/graph.hpp"

/**
 * @brief 点类构造函数
 * @param x 点的x坐标
 * @param y 点的y坐标
 */
Point::Point(int x, int y) : x(x), y(y) {}

/**
 * @brief 重载相等运算符，判断两个点是否相同
 * @param other 另一个点
 * @return 如果两点坐标相同返回true，否则返回false
 */
bool Point::operator==(const Point& other) const {
    return x == other.x && y == other.y;
}

/**
 * @brief 图类构造函数
 * @param map 二维向量表示的地图数据，其中每个元素代表对应位置的地形代价
 */
Graph::Graph(std::vector<std::vector<double>> map) : map(map) {}


/**
 * @brief 检查指定点是否在地图范围内
 * @param point 待检查的点
 * @return 如果点在地图范围内返回true，否则返回false
 */
bool Graph::point_allowed(Point point) {
    if (point.x < 0 || point.x >= map.size() || point.y < 0 || point.y >= map[0].size())
        return false;
    else
        return true;
}

/**
 * @brief 获取指定点的邻居节点
 * @param point 当前点
 * @param idx 邻居节点索引(0-3)，分别代表上、下、左、右四个方向
 * @return 邻居点的坐标
 */
Point Graph::get_neighbour(Point point, int idx) { 
    switch (idx) {
    case 0:
        return Point{ point.x, point.y - 1 };  // 上方邻居
    case 1:
        return Point{ point.x, point.y + 1 };  // 下方邻居
    case 2:
        return Point{ point.x - 1, point.y };  // 左侧邻居
    case 3:
        return Point{ point.x + 1, point.y };  // 右侧邻居
    default:
        return Point{ -1, -1 };  // 无效索引
    }
}