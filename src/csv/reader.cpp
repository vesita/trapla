#include "csv/reader.hpp"

/**
 * @brief 构造函数，初始化CSV读取器
 */
CSVReader::CSVReader() : rows(0), cols(0) {
}

/**
 * @brief 析构函数
 */
CSVReader::~CSVReader() {
}

/**
 * @brief 从文件读取CSV数据
 * 
 * @param filename 文件名
 * @return 如果读取成功返回true，否则返回false
 */
bool CSVReader::readFromFile(const std::string& filename) {

    std::filesystem::path filePath(filename);
    

    if (filePath.is_relative()) {
        filePath = std::filesystem::absolute(filePath);
    }
    std::cout << "尝试读取文件: " << filePath.string() << std::endl;

    if (!std::filesystem::exists(filePath)) {
        std::cerr << "错误: 文件不存在 " << filename << std::endl;
        return false;
    }
    
    std::ifstream file(filePath.string());
    if (!file.is_open()) {
        std::cerr << "错误: 无法打开文件 " << filePath.string() << std::endl;
        return false;
    }
    
    data.clear();
    std::string line;
    rows = 0;
    

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream lineStream(line);
        std::string cell;
        size_t colCount = 0;
        

        while (std::getline(lineStream, cell, ',')) {
            int value = std::stoi(cell);
            row.push_back(value);
            colCount++;
        }
        

        if (rows == 0) {
            cols = colCount;
        }

        else if (colCount != cols) {
            std::cerr   << "警告: 第 " << rows << " 行有 " << colCount 
                        << " 列，但预期为 " << cols << " 列" << std::endl;
        }
        
        data.push_back(row);
        rows++;
    }
    
    file.close();
    return true;
}

/**
 * @brief 获取读取的CSV数据
 * 
 * @return CSV数据的常量引用
 */
const std::vector<std::vector<double>>& CSVReader::getData() const {
    return data;
}

/**
 * @brief 获取数据行数
 * 
 * @return 数据行数
 */
size_t CSVReader::getRows() const {
    return rows;
}

/**
 * @brief 获取数据列数
 * 
 * @return 数据列数
 */
size_t CSVReader::getCols() const {
    return cols;
}