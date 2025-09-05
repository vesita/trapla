#ifndef GRAPH_H
#define GRAPH_H

class Point;
struct PointHash;
class Graph;

#include <vector>
#include <cmath>

/**
 * @brief 表示二维网格中的点
 * 点由整数坐标(x, y)定义
 */
class Point {
public:
    int x;  ///< 点的x坐标
    int y;  ///< 点的y坐标

    /**
     * @brief 构造函数，创建一个点
     * @param x 点的x坐标，默认为0
     * @param y 点的y坐标，默认为0
     */
    Point(int x=0, int y=0);
    
    /**
     * @brief 重载相等运算符，判断两个点是否相同
     * @param other 另一个点
     * @return 如果两点坐标相同返回true，否则返回false
     */
    bool operator==(const Point& other) const;
};


/**
 * @brief Point的哈希函数结构体
 * 用于在unordered_map和unordered_set中作为Point的哈希函数
 */
struct PointHash {
    /**
     * @brief 计算Point对象的哈希值
     * @param p 待计算哈希值的点
     * @return 点的哈希值
     */
    std::size_t operator()(const Point& p) const {
        return std::hash<int>{}(p.x) ^ (std::hash<int>{}(p.y) << 1);
    }
};


/**
 * @brief 图结构类，表示用于路径规划的网格地图
 * 地图由二维向量表示，每个元素代表对应位置的地形代价
 */
class Graph {
public:
    std::vector<std::vector<double>> map;  ///< 地图数据，二维向量

    /**
     * @brief 构造函数，创建一个图结构
     * @param map 二维向量表示的地图数据
     */
    Graph(std::vector<std::vector<double>> map);

    /**
     * @brief 检查指定点是否在地图范围内
     * @param point 待检查的点
     * @return 如果点在地图范围内返回true，否则返回false
     */
    bool point_allowed(Point point);

    /**
     * @brief 获取指定点的邻居节点
     * @param point 当前点
     * @param idx 邻居节点索引(0-3)，分别代表上、下、左、右四个方向
     * @return 邻居点的坐标
     */
    Point get_neighbour(Point point, int idx);
};

#endif