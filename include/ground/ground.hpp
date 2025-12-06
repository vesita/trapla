#ifndef GROUND_HPP
#define GROUND_HPP 

class Ground;

#include <vector>
#include <iostream>
#include <array>
#include <algorithm>

#include "csv/reader.hpp"
#include "robot/foot.hpp"
#include "utils/geometry.hpp"

class Ground {
public:
    
    Ground(std::string filename);

    Ground(int rows, int cols);

    SqPlane map;
    
    CuPlane trip(std::vector<SqDot> area) const;
    
    CuPlane convex(std::vector<SqDot> area) const;

    CuDot normal(std::vector<SqDot> area) const;
    
    double stand_angle(std::vector<SqDot> area) const;

    std::array<int, 2> shape() const;

    bool empty() const;

    bool is_valid(const SqDot& point) const;

    bool is_valid(const int& x, const int& y) const;

    bool obstacle(const int& x, const int& y) const;

    bool set_unit(const int& x, const int& y, bool is_obstacle);

    int rows() const;

    int cols() const;

};

#endif