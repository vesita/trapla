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
    
    enum class LogLevel {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };
    
    enum class TestType {
        VALIDATION,  // 验证式测试（默认）- 需要通过/失败判断
        EXPLORATORY  // 非验证式测试 - 仅运行，不强制验证结果，但会记录数据
    };
    
    struct TestResult {
        std::string name;
        std::string group; // 测试组
        bool passed;
        std::string errorMessage;
        double duration; // 测试持续时间(毫秒)
        size_t failureCount;
        TestType type; // 测试类型
    };
    
    // 数据记录结构
    struct DataRecord {
        std::string testName;
        std::vector<std::string> columnNames;
        std::vector<std::vector<double>> dataRows;
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
    
    // 设置最小日志级别
    void setLogLevel(LogLevel level) {
        minLogLevel = level;
    }
    
    // 添加测试（支持分组和测试类型）
    void addTest(const std::string& name, TestFunction test, const std::string& group = "default", TestType type = TestType::VALIDATION) {
        tests.push_back({name, test, group, type});
    }
    
    // 标准化日志输出方法
    void log(const std::string& message, LogLevel level = LogLevel::INFO) {
        if (level < minLogLevel) {
            return;
        }
        
        auto timestamp = getCurrentTimestamp();
        std::string levelStr = logLevelToString(level);
        
        std::stringstream logStream;
        logStream << "[" << timestamp << "] [" << levelStr << "] " << message;
        std::string formattedMessage = logStream.str();
        
        // 根据日志级别决定输出到标准输出还是错误流
        if (level == LogLevel::ERROR || level == LogLevel::WARN) {
            std::cerr << formattedMessage << std::endl;
        } else {
            std::cout << formattedMessage << std::endl;
        }
        
        if (logFile && *logFile) {
            *logFile << formattedMessage << std::endl;
            logFile->flush();
        }
    }
    
    // 便捷的日志方法
    void debug(const std::string& message) { log(message, LogLevel::DEBUG); }
    void info(const std::string& message) { log(message, LogLevel::INFO); }
    void warn(const std::string& message) { log(message, LogLevel::WARN); }
    void error(const std::string& message) { log(message, LogLevel::ERROR); }
    
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
    
    // 如果有失败数据则抛出异常（仅对验证式测试有效）
    void throwIfFailed(const std::string& testName, const std::string& message) {
        auto it = failedData.find(testName);
        // 检查测试是否为验证式测试
        bool isValidationTest = true;
        for (const auto& test : tests) {
            if (test.name == testName && test.type == TestType::EXPLORATORY) {
                isValidationTest = false;
                break;
            }
        }
        
        // 只有验证式测试才会因为失败而抛出异常
        if (isValidationTest && it != failedData.end() && !it->second.empty()) {
            throw std::runtime_error(testName + " " + message + "，共 " + std::to_string(it->second.size()) + " 个测试用例未通过");
        }
    }
    
    // 清除指定测试的失败数据
    void clearFailures(const std::string& testName) {
        failedData.erase(testName);
    }
    
    // 设置是否运行非验证式测试
    void setRunExploratoryTests(bool run) {
        runExploratoryTests = run;
    }
    
    // 添加数据记录（用于非验证式测试的数据分析）
    void addDataRecord(const std::string& testName, 
                       const std::vector<std::string>& columnNames,
                       const std::vector<double>& dataRow) {
        // 查找是否已存在该测试的数据记录
        auto it = dataRecords.find(testName);
        if (it == dataRecords.end()) {
            // 创建新的数据记录
            DataRecord record;
            record.testName = testName;
            record.columnNames = columnNames;
            record.dataRows.push_back(dataRow);
            dataRecords[testName] = record;
        } else {
            // 添加到现有记录
            it->second.dataRows.push_back(dataRow);
        }
    }
    
    // 写入数据记录到CSV文件
    void writeDataRecords(const std::string& testName, const std::string& csvFilename) {
        auto it = dataRecords.find(testName);
        if (it != dataRecords.end() && !it->second.dataRows.empty()) {
            CSVWriter writer;
            std::string logPath = IOManager::get_instance().build_path("log/" + csvFilename);
            IOManager::get_instance().createDirectories(logPath);
            writer.writeToFile(logPath, it->second.dataRows, it->second.columnNames);
        }
    }
    
    // 清除指定测试的数据记录
    void clearDataRecords(const std::string& testName) {
        dataRecords.erase(testName);
    }
    
    bool runTests() {
        // 过滤测试
        std::vector<Test> filteredTests;
        if (!testFilter.empty()) {
            std::regex filterRegex(testFilter);
            for (const auto& test : tests) {
                // 如果不运行非验证式测试，则跳过
                if (!runExploratoryTests && test.type == TestType::EXPLORATORY) {
                    continue;
                }
                
                if (std::regex_search(test.name, filterRegex) || 
                    std::regex_search(test.group, filterRegex)) {
                    filteredTests.push_back(test);
                }
            }
        } else {
            // 如果没有过滤器，根据设置决定是否包含非验证式测试
            for (const auto& test : tests) {
                if (runExploratoryTests || test.type == TestType::VALIDATION) {
                    filteredTests.push_back(test);
                }
            }
        }
        
        info("总共找到 " + std::to_string(tests.size()) + " 个测试，运行 " + std::to_string(filteredTests.size()) + " 个测试...");
        bool allPassed = true;
        testResults.clear();
        testResults.reserve(filteredTests.size());
        
        for (const auto& test : filteredTests) {
            // 清除之前的失败数据
            clearFailures(test.name);
            clearDataRecords(test.name);
            
            std::string testTypeStr = (test.type == TestType::EXPLORATORY) ? " [探索性]" : "";
            info("正在运行测试: " + test.name + " [" + test.group + "]" + testTypeStr);
            TestResult result;
            result.name = test.name;
            result.group = test.group;
            result.passed = true;
            result.failureCount = 0;
            result.type = test.type;
            
            TimePoint start = std::chrono::high_resolution_clock::now();
            try {
                test.func();
                result.errorMessage = "";
            } catch (const std::exception& e) {
                result.passed = false;
                result.errorMessage = e.what();
                // 非验证式测试的异常不计入整体失败
                if (test.type == TestType::VALIDATION) {
                    allPassed = false;
                }
                error("测试异常: " + test.name + " - " + e.what());
            } catch (...) {
                result.passed = false;
                result.errorMessage = "未知错误";
                // 非验证式测试的异常不计入整体失败
                if (test.type == TestType::VALIDATION) {
                    allPassed = false;
                }
                error("测试异常: " + test.name + " - 未知错误");
            }
            TimePoint end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            result.duration = duration.count() / 1000.0; // 转换为毫秒
            result.failureCount = failureCount(test.name);
            
            testResults.push_back(result);
            
            // 对于非验证式测试，写入数据记录到CSV文件
            if (test.type == TestType::EXPLORATORY) {
                std::string csvFilename = test.name + "_data.csv";
                writeDataRecords(test.name, csvFilename);
                info("  数据已保存到: " + csvFilename);
            }
            
            if (result.passed) {
                info("  结果: 通过 (耗时: " + std::to_string(result.duration) + " ms)");
            } else {
                // 非验证式测试即使失败也显示为"完成"
                if (test.type == TestType::EXPLORATORY) {
                    info("  结果: 完成 (耗时: " + std::to_string(result.duration) + " ms)");
                } else {
                    error("  结果: 失败 - " + result.errorMessage + " (耗时: " + std::to_string(result.duration) + " ms)");
                }
            }
        }
        
        // 输出测试摘要
        printTestSummary();
        
        info("测试套件执行完成。" + std::string(allPassed ? "所有验证式测试通过。" : "部分验证式测试失败。"));
        return allPassed;
    }

private:
    struct Test {
        std::string name;
        TestFunction func;
        std::string group;
        TestType type;
    };
    
    std::vector<Test> tests;
    std::unique_ptr<std::ofstream> logFile;
    std::map<std::string, std::vector<std::vector<double>>> failedData; // 存储失败数据
    std::map<std::string, DataRecord> dataRecords; // 存储非验证式测试的数据记录
    std::vector<TestResult> testResults; // 存储测试结果
    std::string testFilter; // 测试过滤器
    LogLevel minLogLevel = LogLevel::INFO; // 默认最小日志级别为INFO
    bool runExploratoryTests = false; // 是否运行非验证式测试
    
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
    
    std::string logLevelToString(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO";
            case LogLevel::WARN:  return "WARN";
            case LogLevel::ERROR: return "ERROR";
            default:              return "UNKNOWN";
        }
    }
    
    void printTestSummary() {
        info("");
        info("测试摘要:");
        info("--------------------------------------------------------------------------------");
        info("测试名称                组别        类型       状态     失败数   耗时(ms)");
        info("--------------------------------------------------------------------------------");
        
        size_t totalFailures = 0;
        size_t exploratoryTests = 0;
        size_t validationTests = 0;
        
        for (const auto& result : testResults) {
            std::string status = result.passed ? "通过" : "失败";
            // 非验证式测试显示为"完成"而不是"失败"
            if (!result.passed && result.type == TestType::EXPLORATORY) {
                status = "完成";
            }
            
            std::string typeStr = (result.type == TestType::EXPLORATORY) ? "探索" : "验证";
            
            // 使用iomanip格式化功能实现更好的对齐
            std::stringstream lineStream;
            lineStream << std::left << std::setw(24) << result.name
                        << std::left << std::setw(12) << result.group
                        << std::left << std::setw(11) << typeStr
                        << std::left << std::setw(9) << status
                        << std::right << std::setw(5) << result.failureCount
                        << std::right << std::setw(11);
            
            // 格式化耗时，保留2位小数
            std::stringstream durationStream;
            durationStream.precision(2);
            durationStream << std::fixed << result.duration;
            lineStream << durationStream.str();
            
            info(lineStream.str());
            totalFailures += (result.type == TestType::VALIDATION) ? result.failureCount : 0;
            
            if (result.type == TestType::EXPLORATORY) {
                exploratoryTests++;
            } else {
                validationTests++;
            }
        }
        
        info("--------------------------------------------------------------------------------");
        info("总计: 验证式测试 " + std::to_string(validationTests) + " 个, 探索性测试 " + std::to_string(exploratoryTests) + " 个");
        info("验证式测试失败用例数: " + std::to_string(totalFailures));
        info("");
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

// 定义非验证式测试宏
#define EXPLORATORY_TEST(name) \
    static void test_##name(); \
    static bool registered_##name = []() { \
        TestFramework::getInstance().addTest(#name, test_##name, "default", TestFramework::TestType::EXPLORATORY); \
        return true; \
    }(); \
    static void test_##name()

// 定义带分组的非验证式测试宏
#define EXPLORATORY_TEST_GROUP(name, group) \
    static void test_##name(); \
    static bool registered_##name = []() { \
        TestFramework::getInstance().addTest(#name, test_##name, #group, TestFramework::TestType::EXPLORATORY); \
        return true; \
    }(); \
    static void test_##name()

#endif