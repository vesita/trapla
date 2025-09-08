#include "utils/fast_flatness.hpp"

/**
 * @brief 评估指定地图区域的平整度
 * 
 * 该函数计算指定中心点和边长定义的区域的平整度，通过计算区域内高度值的
 * 标准差和高度差来综合评估地形的平整程度
 * 
 * @param map 地图对象
 * @param center 区域中心点
 * @param side_length 区域边长
 * @return 平整度评分，值越小表示越平整
 */
double FastFlatnessEvaluator::evaluate(const SqPlain& map, const SqDot& center, int side_length) {

    int half_side = side_length / 2;
    int start_row = std::max(0, center.x - half_side);
    int end_row = std::min(map.rows() - 1, center.x + half_side);
    int start_col = std::max(0, center.y - half_side);
    int end_col = std::min(map.cols() - 1, center.y + half_side);
    

    std::vector<double> heights;
    heights.reserve((end_row - start_row + 1) * (end_col - start_col + 1));
    
    for (int i = start_row; i <= end_row; i++) {
        for (int j = start_col; j <= end_col; j++) {

            if (map[i][j] >= 0) {
                heights.emplace_back(map[i][j]);
            }
        }
    }
    
    return evaluate(heights);
}

/**
 * @brief 评估高度值集合的平整度
 * 
 * 通过计算高度值的标准差和最大最小值差来评估平整度，标准差占70%权重，
 * 高度差占30%权重
 * 
 * @param heights 高度值集合
 * @return 平整度评分，值越小表示越平整
 */
double FastFlatnessEvaluator::evaluate(const std::vector<double>& heights) {
    if (heights.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    if (heights.size() == 1) {
        return 0.0;
    }
    

    double sum = std::accumulate(heights.begin(), heights.end(), 0.0);
    double mean = sum / heights.size();
    

    double variance = 0.0;
    for (const auto& height : heights) {
        double diff = height - mean;
        variance += diff * diff;
    }
    variance /= heights.size();
    double stddev = std::sqrt(variance);
    

    auto minmax = std::minmax_element(heights.begin(), heights.end());
    double height_diff = *minmax.second - *minmax.first;
    

    return 0.7 * stddev + 0.3 * height_diff;
}