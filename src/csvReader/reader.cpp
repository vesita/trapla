#include "csvReader/reader.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

/**
 * @brief CSV读取器构造函数
 * 初始化行数和列数为0
 */
Reader::Reader() : rows(0), cols(0) {
}

/**
 * @brief CSV读取器析构函数
 */
Reader::~Reader() {
}

/**
 * @brief 从CSV文件中读取数据
 * @param filename 要读取的CSV文件名
 * @return 如果成功读取文件返回true，否则返回false
 */
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

/**
 * @brief 获取读取到的数据
 * @return 包含CSV数据的二维向量的常量引用
 */
const std::vector<std::vector<double>>& Reader::getData() const {
    return data;
}

/**
 * @brief 获取数据的行数
 * @return 行数
 */
size_t Reader::getRows() const {
    return rows;
}

/**
 * @brief 获取数据的列数
 * @return 列数
 */
size_t Reader::getCols() const {
    return cols;
}