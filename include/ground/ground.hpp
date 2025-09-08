#ifndef GROUND_HPP
#define GROUND_HPP 

class Ground;

#include <vector>
#include <iostream>
#include <array>
#include <algorithm>

#include "csvReader/reader.hpp"
#include "robot/foot.hpp"
#include "utils/geometry.hpp"

class Ground {
public:
    
    Ground(std::string filename);

    SqPlain map;
    
    CuPlain trip(std::vector<SqDot>& area);
    
    
    CuDot normal(std::vector<SqDot>& area);
    
    
    CuPlain convex_trip(std::vector<SqDot>& area);
    
    
    double stand_angle(std::vector<SqDot>& area);

    
    std::array<int, 2> shape() const;

    
    bool empty() const;
};

#endif