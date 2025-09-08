#ifndef TEST_FRAMEWORK_HPP
#define TEST_FRAMEWORK_HPP

#include <iostream>
#include <vector>
#include <functional>
#include <string>
#include <fstream>
#include <memory>
#include "utils/io.hpp"

class TestFramework {
public:
    using TestFunction = std::function<void()>;
    
    
    static TestFramework& getInstance() {
        static TestFramework instance;
        return instance;
    }
    
    
    void setWorkingDirectory(const std::string& workingDir) {
        IOManager::getInstance().setWorkingDirectory(workingDir);
    }
    
    
    void setLogFile(const std::string& filename) {
        std::string logPath = IOManager::getInstance().buildPath(filename);
        IOManager::getInstance().createDirectories(logPath);
        logFile = std::make_unique<std::ofstream>(logPath);
    }
    
    
    void addTest(const std::string& name, TestFunction test) {
        tests.push_back({name, test});
    }
    
    
    void log(const std::string& message) {
        std::cout << message << std::endl;
        if (logFile && *logFile) {
            *logFile << message << std::endl;
            logFile->flush();
        }
    }
    
    
    bool runTests() {
        log("正在运行 " + std::to_string(tests.size()) + " 个测试...");
        bool allPassed = true;
        
        for (const auto& test : tests) {
            log("正在运行测试: " + test.name);
            try {
                test.func();
                log("  结果: 通过");
            } catch (const std::exception& e) {
                log("  结果: 失败 - " + std::string(e.what()));
                allPassed = false;
            } catch (...) {
                log("  结果: 失败 - 未知错误");
                allPassed = false;
            }
        }
        
        log("测试套件执行完成。" + std::string(allPassed ? "所有测试通过。" : "部分测试失败。"));
        return allPassed;
    }

private:
    
    struct Test {
        std::string name;
        TestFunction func;
    };
    
    std::vector<Test> tests;
    std::unique_ptr<std::ofstream> logFile;
    
    TestFramework() = default;
};

#define TEST(name) \
    static void test_##name(); \
    static bool registered_##name = []() { \
        TestFramework::getInstance().addTest(#name, test_##name); \
        return true; \
    }(); \
    static void test_##name()

#endif