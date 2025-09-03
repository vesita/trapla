#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <cmath>

class Point {
public:
    int x;
    int y;

    bool operator==(const Point& other) const;
};


class Graph {
public:
    std::vector<std::vector<double>> map;

    Graph(std::vector<std::vector<double>> map);

    bool point_allowed(Point point);

    Point get_neighbour(Point point, int idx);
};

#endif