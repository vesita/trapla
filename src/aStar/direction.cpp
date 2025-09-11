#include "aStar/direction.hpp"

SqDot direction_determine(const SqDot& at, const std::vector<SqDot>& guides) { 
    SqDot result(0.0, 0.0);
    if (guides.empty()) {
        return result;
    }
    
    auto coefficient = geometric_decay(3, guides.size());
    for (int order = 0; order < guides.size(); ++order) {
        auto& shift = coefficient[order];
        result += guides[order] * shift;
    }
    return result;
}

std::vector<double> geometric_decay(double alpha, int n) {
    auto total = geometric_sum(1.0, alpha, n);
    std::vector<double> coefficient;
    coefficient.reserve(n);
    for (int order = 1; order <= n; ++order) {
        coefficient.emplace_back(pow(alpha, n - order)/ total);
    }
    
    return coefficient;
}