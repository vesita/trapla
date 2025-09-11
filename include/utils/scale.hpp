#ifndef SCALE_HPP
#define SCALE_HPP 

#include <cmath>
#include "utils/geometry.hpp"
#include "utils/index.hpp"

int index_scale(double index, double scale);

int index_scale(int index, double scale);

double scale(double index, double scale);
/**
 * @brief 以origin为原点对input点进行缩放
 * 
 * @param origin 原点
 * @param input 输入点
 * @param scale 缩放因子
 * @return 缩放后的点
 */
Intex scale_dot(const Intex& origin, const Intex& input, int scale);

#endif