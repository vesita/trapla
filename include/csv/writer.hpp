#ifndef CSVWRITER_WRITER_HPP
#define CSVWRITER_WRITER_HPP

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <fstream>

class CSVWriter { 
public:
    
    CSVWriter();
    
    
    ~CSVWriter();
    
    
    bool writeToFile(const std::string& filename, const std::vector<std::vector<double>>& data);
    
    
    bool writeToFile(const std::string& filename, const std::vector<std::vector<double>>& data, 
                    const std::vector<std::string>& columnNames);
    
    
    bool writeToFile(const std::string& filename, const std::vector<std::vector<double>>& data,
                    const std::vector<std::string>& columnNames, bool includeHeader);

private:
    std::vector<std::vector<double>> data;
    std::vector<std::string> columnNames;
    size_t rows;
    size_t cols;
};

#endif