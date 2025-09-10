#ifndef IO_HPP
#define IO_HPP

#include <string>
#include <filesystem>
#include <memory>
#include <fstream>
#include <iostream>

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
        if (workingDirectory.empty()) {
            return relativePath;
        }
        return workingDirectory + "/" + relativePath;
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