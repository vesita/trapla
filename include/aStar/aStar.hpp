#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <array>

#include "aStar/graph.hpp"

double manhattan_distance(Point a, Point b);
double euclidean_distance(Point a, Point b);

std::array<std::vector<double>, 2> a_star_search(Graph& graph,
    Point start, Point goal);

#endif