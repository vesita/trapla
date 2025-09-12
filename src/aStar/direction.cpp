#include "aStar/direction.hpp"

SqDot direction_determine(const SqDot& at, const std::vector<SqDot>& guides) { 
    SqDot result(0.0, 0.0);
    if (guides.empty()) {
        return result;
    }
    // 在这里更改路径权重
    // 目前在简单测试中,根号pi和根号e的平滑表现比较好
    auto coefficient = geometric_decay(sqrt(M_PI), guides.size());
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
        coefficient.emplace_back(pow(alpha, order - 1) / total);
    }
    
    std::reverse(coefficient.begin(), coefficient.end());
    return coefficient;
}

std::vector<double> geometric_decay(double start, double alpha, int n) {
    auto total = geometric_sum(start, alpha, n);
    std::vector<double> coefficient;
    coefficient.reserve(n);
    for (int order = 1; order <= n; ++order) {
        coefficient.emplace_back(start * pow(alpha, order - 1) / total);
    }
    std::reverse(coefficient.begin(), coefficient.end());
    return coefficient;
}

std::vector<double> square_decay(int n) {
    auto total = n * (n + 1) * (2 * n + 1) / 6.0;
    std::vector<double> coefficient;
    coefficient.reserve(n);
    for (int order = 1; order <= n; ++order) {
        coefficient.emplace_back(order * order / total);
    }
    std::reverse(coefficient.begin(), coefficient.end());
    return coefficient;
}

std::vector<double> square_decay(double start, int n) { 
    auto coefficient = square_decay(n + start);
    return std::vector<double>(coefficient.begin(), coefficient.begin() + start);
}