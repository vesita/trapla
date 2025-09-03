#ifndef FOOT_HPP
#define FOOT_HPP

#include "ground/ground.hpp"

class FootShape {
public:
    double length{};
    double width{};

    FootShape();
    FootShape(double length, double width);
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