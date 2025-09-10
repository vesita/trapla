#ifndef SCALE_HPP
#define SCALE_HPP 

#include <cmath>
#include "utils/geometry.hpp"

int index_scale(double index, double scale);

/**
 * @brief 以origin为原点对input点进行缩放
 * 
 * @param origin 原点
 * @param input 输入点
 * @param scale 缩放因子
 * @return 缩放后的点
 */
SqDot scale_dot(const SqDot& origin, const SqDot& input, int scale);

#endif