#include "utils/fast_flatness.hpp"
#include <numeric>
#include <limits>

double FastFlatnessEvaluator::evaluate(const SqPlain& map, const SqDot& center, int side_length) {
    // 计算区域边界
    int half_side = side_length / 2;
    int start_row = std::max(0, center.x - half_side);
    int end_row = std::min(map.rows() - 1, center.x + half_side);
    int start_col = std::max(0, center.y - half_side);
    int end_col = std::min(map.cols() - 1, center.y + half_side);
    
    // 收集区域内的高度值
    std::vector<double> heights;
    heights.reserve((end_row - start_row + 1) * (end_col - start_col + 1));
    
    for (int i = start_row; i <= end_row; i++) {
        for (int j = start_col; j <= end_col; j++) {
            // 只考虑有效的单元格（非障碍物）
            if (map[i][j] >= 0) {
                heights.emplace_back(map[i][j]);
            }
        }
    }
    
    return evaluate(heights);
}

double FastFlatnessEvaluator::evaluate(const std::vector<double>& heights) {
    if (heights.empty()) {
        return std::numeric_limits<double>::max(); // 空区域认为完全不平整
    }
    
    if (heights.size() == 1) {
        return 0.0; // 单点区域完全平整
    }
    
    // 计算平均高度
    double sum = std::accumulate(heights.begin(), heights.end(), 0.0);
    double mean = sum / heights.size();
    
    // 计算标准差
    double variance = 0.0;
    for (const auto& height : heights) {
        double diff = height - mean;
        variance += diff * diff;
    }
    variance /= heights.size();
    double stddev = std::sqrt(variance);
    
    // 计算最大高度差
    auto minmax = std::minmax_element(heights.begin(), heights.end());
    double height_diff = *minmax.second - *minmax.first;
    
    // 综合评估值：结合标准差和最大高度差
    // 标准差反映整体波动情况，最大高度差反映局部极端情况
    return 0.7 * stddev + 0.3 * height_diff;
}