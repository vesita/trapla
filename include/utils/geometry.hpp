#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

enum class Cupos;
struct SqDotHash;
class SqDot;
class SqPlain;
class CuDot;
class CuLine;
class CuPlain;

#include <vector>
#include <array>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include "utils/fast_flatness.hpp"
#include "utils/scale.hpp"

double manhattan_distance(SqDot a, SqDot b);

double euclidean_distance(SqDot a, SqDot b);

enum class CuPos { Above, Below, Inside };

struct SqDotHash { 
    
    
    std::size_t operator()(const SqDot& p) const;
};

class SqDot { 
public:
    int x;
    int y;

    
    SqDot(int x=0, int y=0);

    
    SqDot scale(double scale) const;
    
    
    SqDot central_restore(const double& scale) const;
    
    
    bool operator==(const SqDot& other) const;
    
    
    bool operator!=(const SqDot& other) const;

    
    bool operator<(const SqDot& other) const;

    
    int sx(double& scale) const;
    
    
    int sy(double& scale) const;
    
    
    SqDot scale(double& scale);

    
    SqDot center(const SqDot& other);

    
    SqDot get_neighbour(int index) const;

    
    std::vector<SqDot> get_neighbour() const;

    
    std::vector<SqDot> get_neighbour(int x_ceil, int y_ceil) const;

    
    double distance(const SqDot& other) const;
};

class SqPlain { 
public:
    std::vector<std::vector<double>> map;

    
    SqPlain(std::vector<std::vector<double>> map);

    
    SqPlain(int rows, int cols, double value=0.0);

    
    SqPlain();

    
    bool edge_allowed(SqDot point);

    
    SqDot get_neighbour(SqDot point, int idx);

    
    SqDot orth_near(const SqDot& point) const;

    
    SqDot local_center(SqDot& first, SqDot& second);

    
    std::vector<SqDot> get_neighbour(SqDot point);
    
    
    std::vector<SqDot> get_valid_neighbours(SqDot point);
    
    
    std::vector<SqDot> find_path(SqDot start, SqDot goal);
    
    
    SqPlain scale_graph(double scale);

    
    SqPlain scale_graph_variance(double scale);
    
    
    double summary(SqDot& center, int side_length);

    
    bool empty() const;
    
    
    int rows() const;

    
    int row_scale(double& scale) const;

    
    int cols() const;

    
    int col_scale(double& scale) const;

    
    std::vector<double>& operator[](int index);
    
    
    const std::vector<double>& operator[](int index) const;

    
    double cost(SqDot& at, SqDot& to);

    
    SqDot restore_dot(SqDot& dot, double scale) const;

    
    std::pair<SqDot, SqDot> restore(const SqDot& block, double scale) const;

    
    bool in_same_block(const SqDot& a, const SqDot& b, double scale) const;
};

class CuDot {
public:
    double x;
    double y;
    double z;

    
    CuDot(double x=0.0, double y=0.0, double z=0.0);
    
    
    CuDot operator+(const CuDot& dot) const;

    
    SqDot slide();
};

class CuLine {
public: 
    CuDot point;

    
    CuLine(const CuDot& point=CuDot());

    
    CuDot offset(const CuDot& dot);

    
    CuDot get();

    
    CuLine normal_vector(const CuLine& other) const;
};

class CuPlain {
public:
    double A;
    double B;
    double C;
    double D;

    int define_extend;

    
    CuPlain(double A=0.0, double B=0.0, double C=0.0, double D=0.0);
    
    
    bool define_plaine(const std::array<CuDot, 3>& dot);
    
    
    CuPos get_pos(const CuDot& dot) const;
    
    
    double distance(const CuDot& dot) const;
    
    
    CuDot normal_vector() const;
    
    
    double normal_angle() const;
};

#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif