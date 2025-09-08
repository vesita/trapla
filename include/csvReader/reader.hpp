#ifndef CSVREADER_READER_HPP
#define CSVREADER_READER_HPP

#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <fstream>

class Reader { 
public:
    
    Reader();
    
    
    ~Reader();
    
    
    bool readFromFile(const std::string& filename);
    
    
    const std::vector<std::vector<double>>& getData() const;
    
    
    size_t getRows() const;
    
    
    size_t getCols() const;

private:
    std::vector<std::vector<double>> data;
    size_t rows;
    size_t cols;
};

#endif