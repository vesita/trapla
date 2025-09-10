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

std::vector<SqDot> a_star_search(const SqPlain& graph, const SqDot& start, const SqDot& goal);

std::vector<SqDot> scale_star(const SqPlain& graph, const SqDot& start, const SqDot& goal, const double& stride);

std::vector<SqDot> central_restore_guide(std::vector<SqDot>& guides, double scale, double unit_size);

std::vector<SqDot> scale_star_on_scaled_map(const SqPlain& graph, SqDot start, SqDot goal, double stride);

std::vector<SqDot> discrete_guide(SqPlain& graph, double stride, SqDot start, SqDot goal);

double steep_extend(const SqPlain& graph, SqDot& fi, SqDot& se);

#endif