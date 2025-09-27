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
        // 首先检查是否设置了工作目录
        if (!workingDirectory.empty()) {
            std::filesystem::path workPath(workingDirectory);
            std::filesystem::path fullPath = workPath / relativePath;
            if (std::filesystem::exists(fullPath)) {
                return fullPath.string();
            }
        }
        
        // 检查相对于可执行文件的路径 (安装后结构)
        std::filesystem::path exePath;
        #ifdef _WIN32
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        exePath = std::filesystem::path(buffer);
        #else
        exePath = std::filesystem::canonical("/proc/self/exe");
        #endif
        std::filesystem::path installDataPath = exePath.parent_path().parent_path() / "data" / relativePath;
        if (std::filesystem::exists(installDataPath)) {
            return installDataPath.string();
        }
        
        // 检查开发环境中的相对路径
        std::filesystem::path devDataPath = std::filesystem::path("data") / relativePath;
        if (std::filesystem::exists(devDataPath)) {
            return devDataPath.string();
        }
        
        // 如果所有路径都不存在，返回原始相对路径
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