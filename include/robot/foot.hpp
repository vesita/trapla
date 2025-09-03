#ifndef FOOT_HPP
#define FOOT_HPP


class Foot {
public:
    double x{};
    double y{};
    double rz{};

    Foot();
    void set(double x, double y, double rz);
};

#endif