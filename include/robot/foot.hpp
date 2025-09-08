#ifndef FOOT_HPP
#define FOOT_HPP

class Foot;
class FootShape;

#include <iostream>
#include <cmath>
#include <unordered_set>

#include "ground/ground.hpp"
#include "utils/geometry.hpp"

enum class SlideResult {
    NoModification,
    Modified,
    NotApplicable
};

class FootShape {
public:
    double length{};
    double width{};

    
    FootShape();
    
    
    FootShape(double length, double width);
    
    
    bool inside(double l_side, double w_side);
    
    
    std::vector<SqDot> cover(double& rz_of_foot);

    
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);
};

class Foot {
public:
    double x{};
    double y{};
    double rz{};

    
    Foot();

    
    void set(double x, double y, double rz);

    
    bool walkto(Ground& ground, FootShape& shape);
};

#endif