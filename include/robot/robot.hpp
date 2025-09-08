#ifndef ROBOT_HPP
#define ROBOT_HPP

enum class WhichFoot;

#include <array>
#include <cmath>
#include <random>
#include <unordered_set>

#include "robot/foot.hpp"
#include "utils/geometry.hpp"
#include "aStar/aStar.hpp"

enum class WhichFoot { Left, Right };

class Robot {
public:
    std::array<Foot, 2> feet;
    double max_stride;
    double max_turn;
    double max_foot_separation;
    double min_foot_separation;
    FootShape foot_shape;

    WhichFoot now_which_foot_to_move;

    
    
    Robot(double max_stride=40, double max_turn= M_PI * 75/180, double max_foot_separation=10, double min_foot_separation=2,
        double foot_length=5, double foot_width=3);

    
    void walk_update();

    
    std::vector<SqDot> ideal_walk(const Ground& ground);

    
    bool satisfy_foot_limits(const SqDot& new_pos);

    
    Foot& getSwingFoot();

    
    Foot& getSupportFoot();

    
    SlideResult slide(std::vector<SqDot>& area, Ground& ground);

    
    SqDot walk_with_guide(const Ground& ground, const SqDot& goal);

    
    SqDot fit_target(const Ground& ground, const SqDot& goal);

    
    SqDot direct_target(const Ground& ground, const SqDot& goal);

    
    std::vector<SqDot> find_path(const Ground& ground, const SqDot& goal);
};

#endif