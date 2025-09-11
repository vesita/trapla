#include "utils/index.hpp"
#include <cmath>
#include <algorithm>

std::size_t IntexHash::operator()(const Intex& index) const { 
    return std::hash<int>()(index.x) ^ std::hash<int>()(index.y);
}

Intex::Intex(int x, int y):x(x),y(y){}

int Intex::x_index() const { return x; }

int Intex::y_index() const { return y; }

SqDot Intex::as_dot() const { 
    return SqDot(x, y);
}

/**
 * @brief 计算x坐标在缩放后的索引
 * 
 * @param scale 缩放比例
 * @return 缩放后的索引
 */
int Intex::sx(double scale) const { 
    return index_scale(x, scale);
}

/**
 * @brief 计算y坐标在缩放后的索引
 * 
 * @param scale 缩放比例
 * @return 缩放后的索引
 */
int Intex::sy(double scale) const { 
    return index_scale(y, scale);
}

/**
 * @brief 按比例缩放坐标点
 * 
 * @param scale 缩放比例
 * @return 缩放后的坐标点
 */
Intex Intex::scale(double scale) const {
    return Intex(sx(scale), sy(scale));
}

/**
 * @brief 获取指定方向的邻居点
 * 
 * @param index 方向索引(0-3)
 * @return 对应方向的邻居点
 */
Intex Intex::get_neighbour(int index) const { 
    static const std::array<int, 4> dx = {-1, 1, 0, 0};
    static const std::array<int, 4> dy = {0, 0, -1, 1};
    return Intex(x + dx[index], y + dy[index]);
}

/**
 * @brief 获取所有四个方向的邻居点
 * 
 * @return 四个方向邻居点的向量
 */
std::vector<Intex> Intex::get_neighbour() const { 
    std::vector<Intex> neighbours;
    for (int idx = 0; idx < 4; idx++) {
        neighbours.emplace_back(get_neighbour(idx));
    }
    return neighbours;
}

/**
 * @brief 获取指定范围内的邻居点
 * 
 * @param x_ceil x方向上限
 * @param y_ceil y方向上限
 * @return 范围内的邻居点向量
 */
std::vector<Intex> Intex::get_neighbour(int x_ceil, int y_ceil) const { 
    std::vector<Intex> neighbours;
    for (int idx = 0; idx < 4; idx++) {
        Intex neighbour = get_neighbour(idx);

        if (neighbour.x >= 0 && neighbour.x < x_ceil && 
            neighbour.y >= 0 && neighbour.y < y_ceil) {
            neighbours.push_back(neighbour);
        }
    }
    return neighbours;
}

Intex Intex::central_restore(const double& scale) const { 
    auto offset = 0.5 / scale;
    return Intex(x / scale + offset, y / scale + offset);
}

Intex Intex::center(const Intex& other) { 
    return Intex((x + other.x) / 2, (y + other.y) / 2);
}

bool Intex::operator==(const Intex& other) const { 
    return x == other.x && y == other.y;
}

bool Intex::operator!=(const Intex& other) const { 
    return !(*this == other);
}

bool Intex::operator<(const Intex& other) const { 
    return x < other.x || (x == other.x && y < other.y);
}

Intex Intex::operator+(const Intex& other) const { 
    return Intex(x + other.x, y + other.y);
}

Intex Intex::operator-(const Intex& other) const { 
    return Intex(x - other.x, y - other.y);
}