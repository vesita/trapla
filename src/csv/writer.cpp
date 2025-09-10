#include "csv/writer.hpp"

/**
 * @brief 构造函数，初始化CSV写入器
 */
CSVWriter::CSVWriter() : rows(0), cols(0) {
}

/**
 * @brief 析构函数
 */
CSVWriter::~CSVWriter() {
}

/**
 * @brief 将数据写入CSV文件
 * 
 * @param filename 文件名
 * @param data 要写入的数据
 * @return 如果写入成功返回true，否则返回false
 */
bool CSVWriter::writeToFile(const std::string& filename, const std::vector<std::vector<double>>& data) {
    return writeToFile(filename, data, {}, false);
}

/**
 * @brief 将数据和列名写入CSV文件
 * 
 * @param filename 文件名
 * @param data 要写入的数据
 * @param columnNames 列名
 * @return 如果写入成功返回true，否则返回false
 */
bool CSVWriter::writeToFile(const std::string& filename, const std::vector<std::vector<double>>& data, 
                            const std::vector<std::string>& columnNames) {
    return writeToFile(filename, data, columnNames, true);
}

/**
 * @brief 将数据写入CSV文件的核心实现
 * 
 * @param filename 文件名
 * @param data 要写入的数据
 * @param columnNames 列名
 * @param includeHeader 是否包含列名头
 * @return 如果写入成功返回true，否则返回false
 */
bool CSVWriter::writeToFile(const std::string& filename, const std::vector<std::vector<double>>& data,
                            const std::vector<std::string>& columnNames, bool includeHeader) {
    std::filesystem::path filePath(filename);
    
    if (filePath.is_relative()) {
        filePath = std::filesystem::absolute(filePath);
    }
    
    std::cout << "尝试写入文件: " << filePath.string() << std::endl;
    
    // 确保目录存在
    std::filesystem::create_directories(filePath.parent_path());
    
    std::ofstream file(filePath.string());
    if (!file.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filePath.string() << std::endl;
        return false;
    }
    
    // 写入列名头
    if (includeHeader && !columnNames.empty()) {
        for (size_t i = 0; i < columnNames.size(); ++i) {
            file << columnNames[i];
            if (i < columnNames.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }
    
    // 写入数据
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }
    
    file.close();
    return true;
}