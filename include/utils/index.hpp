#ifndef INDEX_HPP
#define INDEX_HPP 

struct IntexHash;
class Intex;

#include <vector>
#include "utils/scale.hpp"

struct IntexHash {
    std::size_t operator()(const Intex& dot) const;
};

class Intex {
public:
    int x;
    int y;
    Intex(int x, int y);

    int x_index() const;
    
    int y_index() const;

    SqDot as_dot() const;
    
    /**
     * @brief 计算x坐标在缩放后的索引
     * 
     * @param scale 缩放比例
     * @return 缩放后的索引
     */
    int sx(double scale) const;
    
    
    /**
     * @brief 计算y坐标在缩放后的索引
     * 
     * @param scale 缩放比例
     * @return 缩放后的索引
     */
    int sy(double scale) const;
    
    
    /**
     * @brief 按比例缩放坐标点
     * 
     * @param scale 缩放比例
     * @return 缩放后的坐标点
     */
    Intex scale(double scale) const;

    /**
     * @brief 获取指定方向的邻居点
     * 
     * @param index 方向索引(0-3)
     * @return 对应方向的邻居点
     */
    Intex get_neighbour(int index) const;

    
    /**
     * @brief 获取所有四个方向的邻居点
     * 
     * @return 四个方向邻居点的向量
     */
    std::vector<Intex> get_neighbour() const;

    
    /**
     * @brief 获取指定范围内的邻居点
     * 
     * @param x_ceil x方向上限
     * @param y_ceil y方向上限
     * @return 范围内的邻居点向量
     */
    std::vector<Intex> get_neighbour(int x_ceil, int y_ceil) const;

    Intex central_restore(const double& scale) const;

    Intex center(const Intex& other);

    bool operator==(const Intex& other) const;

    bool operator!=(const Intex& other) const;

    bool operator<(const Intex& other) const;

    Intex operator+(const Intex& other) const;

    Intex operator-(const Intex& other) const;
};

#endif