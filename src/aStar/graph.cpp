#include "aStar/graph.hpp"

bool Point::operator==(const Point& other) const {
    return x == other.x && y == other.y;
}

Graph::Graph(std::vector<std::vector<double>> map) : map(map) {}


bool Graph::point_allowed(Point point) {
    if (point.x < 0 || point.x >= map.size() || point.y < 0 || point.y >= map[0].size())
        return false;
    else
        return true;
}

Point Graph::get_neighbour(Point point, int idx) { 
    switch (idx) {
    case 0:
        return Point{ point.x, point.y - 1 };
    case 1:
        return Point{ point.x, point.y + 1 };
    case 2:
        return Point{ point.x - 1, point.y };
    case 3:
        return Point{ point.x + 1, point.y };
    default:
        return Point{ -1, -1 };
    }
}