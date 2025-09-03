#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <array>

#include "robot/foot.hpp"

class Robot {
public:
    std::array<Foot, 2> feet{};


    Robot();
};

#endif