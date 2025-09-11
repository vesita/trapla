#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <array>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "utils/io.hpp"
#include "utils/geometry.hpp"
#include "utils/scale.hpp"

std::vector<Intex> a_star_search(const SqPlain& graph, const Intex& start, const Intex& goal);

std::vector<Intex> scale_star(const SqPlain& graph, const Intex& start, const Intex& goal, const double& scale);

// std::vector<SqDot> scale_star(const SqPlain& graph, const SqDot& start, const SqDot& goal, const double& scale);

// 地面采集算法，待完成
double steep_extend(const SqPlain& graph, const Intex& fi, const Intex& se);


#endif