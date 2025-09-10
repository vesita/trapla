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

    SqPlain map;
    
    CuPlain trip(std::vector<SqDot>& area);
    
    
    CuDot normal(std::vector<SqDot>& area);
    
    
    CuPlain convex_trip(std::vector<SqDot>& area);
    
    
    double stand_angle(std::vector<SqDot>& area);

    
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