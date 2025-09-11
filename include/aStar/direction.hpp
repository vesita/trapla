#ifndef DIRECTION_HPP
#define DIRECTION_HPP 

#include "utils/geometry.hpp"
#include "utils/sequence.hpp"


SqDot direction_determine(const SqDot& at, const std::vector<SqDot>& guides);

std::vector<double> geometric_decay (double alpha, int n);

#endif