#ifndef CSVREADER_READER_HPP
#define CSVREADER_READER_HPP

#include <string>
#include <vector>

class Reader { 
public:
    Reader();
    ~Reader();
    
    // 从CSV文件读取数据
    bool readFromFile(const std::string& filename);
    
    // 获取读取到的数据
    const std::vector<std::vector<double>>& getData() const;
    
    // 获取数据的行数和列数
    size_t getRows() const;
    size_t getCols() const;

private:
    std::vector<std::vector<double>> data;
    size_t rows;
    size_t cols;
};

#endif //CSVREADER_READER_HPP