#include "utils/scale.hpp"

/**
 * @brief 计算索引在缩放后的值
 * 
 * 该函数用于计算给定索引在特定缩放比例下的新索引值
 * 
 * @param index 原始索引值
 * @param scale 缩放比例
 * @return 缩放后的索引值
 */
int index_scale(double index, double scale) {
    return static_cast<int>(std::ceil(index * scale));
}

/**
 * @brief 以origin为原点对input点进行缩放
 * 
 * @param origin 原点
 * @param input 输入点
 * @param scale 缩放因子
 * @return 缩放后的点
 */
SqDot scale_dot(const SqDot& origin, const SqDot& input, int scale) {
    int new_x = origin.x + (input.x - origin.x) * scale;
    int new_y = origin.y + (input.y - origin.y) * scale;
    return SqDot(new_x, new_y);
}