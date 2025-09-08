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