#ifndef IO_HPP
#define IO_HPP

#include <string>
#include <filesystem>
#include <memory>
#include <fstream>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
// 解决Windows.h中ERROR宏与测试框架中LogLevel::ERROR枚举值的冲突
#undef ERROR
#endif

class IOManager {
public:
    static IOManager& get_instance() {
        static IOManager instance;
        return instance;
    }
    

    void set_work_path(const std::string& workingDir) {
        workingDirectory = workingDir;
    }
    

    std::string get_work_path() const {
        return workingDirectory;
    }
    

    std::string build_path(const std::string& relativePath) const {
        // 如果设置了工作目录，则使用工作目录
        if (!workingDirectory.empty()) {
            std::filesystem::path workPath(workingDirectory);
            std::filesystem::path fullPath = workPath / relativePath;
            return fullPath.string();
        }
        
        // 否则返回相对路径
        return relativePath;
    }
    

    bool createDirectories(const std::string& path) const {
        try {
            std::filesystem::create_directories(std::filesystem::path(path).parent_path());
            return true;
        } catch (...) {
            return false;
        }
    }
    

    std::unique_ptr<std::ofstream> createOutputFile(const std::string& relativePath) const {
        std::string fullPath = build_path(relativePath);
        createDirectories(fullPath);
        
        auto file = std::make_unique<std::ofstream>(fullPath);
        if (!file->is_open()) {
            std::cerr << "无法创建文件: " << fullPath << std::endl;
            return nullptr;
        }
        
        return file;
    }
    

    std::unique_ptr<std::ifstream> createInputFile(const std::string& relativePath) const {
        std::string fullPath = build_path(relativePath);
        
        auto file = std::make_unique<std::ifstream>(fullPath);
        if (!file->is_open()) {
            std::cerr << "无法打开文件: " << fullPath << std::endl;
            return nullptr;
        }
        
        return file;
    }

private:
    std::string workingDirectory;
    
    IOManager() = default;
};

inline std::string buildPath(const std::string& relativePath) {
    return IOManager::get_instance().build_path(relativePath);
}

#endif