#ifndef TEST_FRAMEWORK_HPP
#define TEST_FRAMEWORK_HPP

#include <iostream>
#include <vector>
#include <functional>
#include <string>

class TestFramework {
public:
    using TestFunction = std::function<void()>;
    
    static TestFramework& getInstance() {
        static TestFramework instance;
        return instance;
    }
    
    void addTest(const std::string& name, TestFunction test) {
        tests.push_back({name, test});
    }
    
    bool runTests() {
        std::cout << "正在运行 " << tests.size() << " 个测试...\n" << std::endl;
        bool allPassed = true;
        
        for (const auto& test : tests) {
            std::cout << "正在运行测试: " << test.name << std::endl;
            try {
                test.func();
                std::cout << "  结果: 通过\n" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "  结果: 失败 - " << e.what() << "\n" << std::endl;
                allPassed = false;
            } catch (...) {
                std::cout << "  结果: 失败 - 未知错误\n" << std::endl;
                allPassed = false;
            }
        }
        
        std::cout << "测试套件执行完成。" << (allPassed ? "所有测试通过。" : "部分测试失败。") << std::endl;
        return allPassed;
    }

private:
    struct Test {
        std::string name;
        TestFunction func;
    };
    
    std::vector<Test> tests;
    
    TestFramework() = default;
};

#define TEST(name) \
    static void test_##name(); \
    static bool registered_##name = []() { \
        TestFramework::getInstance().addTest(#name, test_##name); \
        return true; \
    }(); \
    static void test_##name()

#endif // TEST_FRAMEWORK_HPP