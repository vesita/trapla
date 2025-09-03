#include "robot/foot.hpp"

Foot::Foot(): x(0), y(0), rz(0) {
    return;
}

void Foot::set(double x, double y, double rz) {
    this->x = x;
    this->y = y;
    this->rz = rz;
}