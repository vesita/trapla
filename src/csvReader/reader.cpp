#include "csvReader/reader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

Reader::Reader() : rows(0), cols(0) {
}

Reader::~Reader() {
}

bool Reader::readFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }
    
    data.clear();
    std::string line;
    rows = 0;
    
    // 读取CSV文件的每一行
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream lineStream(line);
        std::string cell;
        size_t colCount = 0;
        
        // 分割每一行的单元格
        while (std::getline(lineStream, cell, ',')) {
            int value = std::stoi(cell);
            row.push_back(value);
            colCount++;
        }
        
        // 设置列数（仅在第一行）
        if (rows == 0) {
            cols = colCount;
        }
        // 检查每一行是否具有相同的列数
        else if (colCount != cols) {
            std::cerr   << "Warning: Row " << rows << " has " << colCount 
                        << " columns, expected " << cols << std::endl;
        }
        
        data.push_back(row);
        rows++;
    }
    
    file.close();
    return true;
}

const std::vector<std::vector<double>>& Reader::getData() const {
    return data;
}

size_t Reader::getRows() const {
    return rows;
}

size_t Reader::getCols() const {
    return cols;
}