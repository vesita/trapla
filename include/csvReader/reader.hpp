#ifndef CSVREADER_READER_HPP
#define CSVREADER_READER_HPP

#include <string>
#include <vector>

/**
 * @brief CSV文件读取器类
 * 用于从CSV文件中读取二维数值数据
 */
class Reader { 
public:
    /**
     * @brief 构造函数，初始化读取器
     */
    Reader();
    
    /**
     * @brief 析构函数
     */
    ~Reader();
    
    /**
     * @brief 从CSV文件中读取数据
     * @param filename 要读取的CSV文件名
     * @return 如果成功读取文件返回true，否则返回false
     */
    bool readFromFile(const std::string& filename);
    
    /**
     * @brief 获取读取到的数据
     * @return 包含CSV数据的二维向量的常量引用
     */
    const std::vector<std::vector<double>>& getData() const;
    
    /**
     * @brief 获取数据的行数
     * @return 行数
     */
    size_t getRows() const;
    
    /**
     * @brief 获取数据的列数
     * @return 列数
     */
    size_t getCols() const;

private:
    std::vector<std::vector<double>> data;  ///< 存储读取的CSV数据
    size_t rows;  ///< 数据行数
    size_t cols;  ///< 数据列数
};

#endif //CSVREADER_READER_HPP