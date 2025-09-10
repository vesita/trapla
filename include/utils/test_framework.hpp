#ifndef TEST_FRAMEWORK_HPP
#define TEST_FRAMEWORK_HPP

#include <iostream>
#include <vector>
#include <functional>
#include <string>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <regex>
#include "utils/io.hpp"
#include "csv/writer.hpp"

class TestFramework {
public:
    using TestFunction = std::function<void()>;
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    
    struct TestResult {
        std::string name;
        std::string group; // 测试组
        bool passed;
        std::string errorMessage;
        double duration; // 测试持续时间(毫秒)
        size_t failureCount;
    };
    
    static TestFramework& getInstance() {
        static TestFramework instance;
        return instance;
    }
    
    void setWorkingDirectory(const std::string& workingDir) {
        IOManager::get_instance().set_work_path(workingDir);
    }
    
    void setLogFile(const std::string& filename) {
        std::string logPath = IOManager::get_instance().build_path(filename);
        IOManager::get_instance().createDirectories(logPath);
        logFile = std::make_unique<std::ofstream>(logPath);
    }
    
    // 添加测试（支持分组）
    void addTest(const std::string& name, TestFunction test, const std::string& group = "default") {
        tests.push_back({name, test, group});
    }
    
    void log(const std::string& message) {
        auto timestamp = getCurrentTimestamp();
        std::cout << "[" << timestamp << "] " << message << std::endl;
        if (logFile && *logFile) {
            *logFile << "[" << timestamp << "] " << message << std::endl;
            logFile->flush();
        }
    }
    
    // 设置测试过滤器
    void setTestFilter(const std::string& filter) {
        testFilter = filter;
    }
    
    // 添加失败数据方法
    void addFailure(const std::string& testName, const std::vector<double>& data) {
        failedData[testName].push_back(data);
    }
    
    // 检查是否有失败数据
    bool hasFailures(const std::string& testName) const {
        auto it = failedData.find(testName);
        return it != failedData.end() && !it->second.empty();
    }
    
    // 获取失败数据数量
    size_t failureCount(const std::string& testName) const {
        auto it = failedData.find(testName);
        return it != failedData.end() ? it->second.size() : 0;
    }
    
    // 写入失败数据到CSV文件
    void writeFailures(const std::string& testName, const std::string& csvFilename, 
                        const std::vector<std::string>& columnNames) {
        auto it = failedData.find(testName);
        if (it != failedData.end() && !it->second.empty()) {
            CSVWriter writer;
            std::string logPath = IOManager::get_instance().build_path("log/" + csvFilename);
            IOManager::get_instance().createDirectories(logPath);
            writer.writeToFile(logPath, it->second, columnNames);
        }
    }
    
    // 如果有失败数据则抛出异常
    void throwIfFailed(const std::string& testName, const std::string& message) {
        auto it = failedData.find(testName);
        if (it != failedData.end() && !it->second.empty()) {
            throw std::runtime_error(testName + " " + message + "，共 " + std::to_string(it->second.size()) + " 个测试用例未通过");
        }
    }
    
    // 清除指定测试的失败数据
    void clearFailures(const std::string& testName) {
        failedData.erase(testName);
    }
    
    bool runTests() {
        // 过滤测试
        std::vector<Test> filteredTests;
        if (!testFilter.empty()) {
            std::regex filterRegex(testFilter);
            for (const auto& test : tests) {
                if (std::regex_search(test.name, filterRegex) || 
                    std::regex_search(test.group, filterRegex)) {
                    filteredTests.push_back(test);
                }
            }
        } else {
            filteredTests = tests;
        }
        
        log("总共找到 " + std::to_string(tests.size()) + " 个测试，运行 " + std::to_string(filteredTests.size()) + " 个测试...");
        bool allPassed = true;
        testResults.clear();
        testResults.reserve(filteredTests.size());
        
        for (const auto& test : filteredTests) {
            // 清除之前的失败数据
            clearFailures(test.name);
            
            log("正在运行测试: " + test.name + " [" + test.group + "]");
            TestResult result;
            result.name = test.name;
            result.group = test.group;
            result.passed = true;
            result.failureCount = 0;
            
            TimePoint start = std::chrono::high_resolution_clock::now();
            try {
                test.func();
                result.errorMessage = "";
            } catch (const std::exception& e) {
                result.passed = false;
                result.errorMessage = e.what();
                allPassed = false;
            } catch (...) {
                result.passed = false;
                result.errorMessage = "未知错误";
                allPassed = false;
            }
            TimePoint end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            result.duration = duration.count() / 1000.0; // 转换为毫秒
            result.failureCount = failureCount(test.name);
            
            testResults.push_back(result);
            
            if (result.passed) {
                log("  结果: 通过 (耗时: " + std::to_string(result.duration) + " ms)");
            } else {
                log("  结果: 失败 - " + result.errorMessage + " (耗时: " + std::to_string(result.duration) + " ms)");
            }
        }
        
        // 输出测试摘要
        printTestSummary();
        
        log("测试套件执行完成。" + std::string(allPassed ? "所有测试通过。" : "部分测试失败。"));
        return allPassed;
    }

private:
    struct Test {
        std::string name;
        TestFunction func;
        std::string group;
    };
    
    std::vector<Test> tests;
    std::unique_ptr<std::ofstream> logFile;
    std::map<std::string, std::vector<std::vector<double>>> failedData; // 存储失败数据
    std::vector<TestResult> testResults; // 存储测试结果
    std::string testFilter; // 测试过滤器
    
    TestFramework() = default;
    
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }
    
    void printTestSummary() {
        log("");
        log("测试摘要:");
        log("--------------------------------------------------------------------------------");
        log("测试名称                组别        状态     失败数   耗时(ms)");
        log("--------------------------------------------------------------------------------");
        
        size_t totalFailures = 0;
        for (const auto& result : testResults) {
            std::string status = result.passed ? "通过" : "失败";
            
            // 使用iomanip格式化功能实现更好的对齐
            std::stringstream lineStream;
            lineStream << std::left << std::setw(24) << result.name
                        << std::left << std::setw(12) << result.group
                        << std::left << std::setw(9) << status
                        << std::right << std::setw(5) << result.failureCount
                        << std::right << std::setw(11);
            
            // 格式化耗时，保留2位小数
            std::stringstream durationStream;
            durationStream.precision(2);
            durationStream << std::fixed << result.duration;
            lineStream << durationStream.str();
            
            log(lineStream.str());
            totalFailures += result.failureCount;
        }
        
        log("--------------------------------------------------------------------------------");
        log("总计失败用例数: " + std::to_string(totalFailures));
        log("");
    }
};

// 定义用于测试失败数据收集的宏
#define TEST(name) \
    static void test_##name(); \
    static bool registered_##name = []() { \
        TestFramework::getInstance().addTest(#name, test_##name); \
        return true; \
    }(); \
    static void test_##name()

// 定义带分组的测试宏
#define TEST_GROUP(name, group) \
    static void test_##name(); \
    static bool registered_##name = []() { \
        TestFramework::getInstance().addTest(#name, test_##name, #group); \
        return true; \
    }(); \
    static void test_##name()

#endif