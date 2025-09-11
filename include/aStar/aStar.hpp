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

std::vector<Intex> scale_star(const SqPlain& graph, const Intex& start, const Intex& goal, const double& stride);

std::vector<Intex> central_restore_guide(std::vector<Intex>& guides, double scale, double unit_size);

std::vector<Intex> scale_star_on_scaled_map(const SqPlain& graph, Intex start, Intex goal, double stride);

std::vector<Intex> discrete_guide(SqPlain& graph, double stride, Intex start, Intex goal);

double steep_extend(const SqPlain& graph, Intex& fi, Intex& se);

#endif