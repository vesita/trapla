#ifndef GROUND_HPP
#define GROUND_HPP 

#include <vector>
#include <iostream>

#include "csvReader/reader.hpp"
#include "aStar/graph.hpp"



class Ground {
public:
    Ground(std::string filename);

    std::vector<std::vector<double>> map;
    std::vector<std::vector<double>> costs;
};

#endif