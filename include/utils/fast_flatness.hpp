#ifndef FAST_FLATNESS_HPP
#define FAST_FLATNESS_HPP

#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>
#include "utils/geometry.hpp"

class FastFlatnessEvaluator {
public:
    
    static double evaluate(const SqPlain& map, const SqDot& center, int side_length);
    
    
    static double evaluate(const std::vector<double>& heights);
};

#endif