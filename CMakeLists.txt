cmake_minimum_required(VERSION 3.25)
project("trapla")

set(CMAKE_CXX_STANDARD 17)

# 查找所有源文件
file(GLOB_RECURSE SOURCES src/*.cpp src/**/*.cpp)

# 排除main.cpp以便创建单独的可执行文件
list(FILTER SOURCES EXCLUDE REGEX "main.cpp")

# 创建主可执行文件
add_executable(trapla src/main.cpp ${SOURCES})

# 创建测试可执行文件
add_executable(foot_test tests/foot_test.cpp ${SOURCES})

# 设置包含目录
target_include_directories(trapla PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(foot_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 创建目录结构以匹配测试中使用的相对路径
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests)