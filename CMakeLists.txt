cmake_minimum_required(VERSION 3.10)
project(trapla)

set(CMAKE_CXX_STANDARD 17)

# 查找所有源文件
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp")

# 创建可执行文件
add_executable(trapla ${SOURCES} ${HEADERS})

# 指定包含目录
target_include_directories(trapla PRIVATE include)

# 创建测试可执行文件，排除main.cpp
file(GLOB_RECURSE TEST_SOURCES "src/*.cpp" "src/*/*.cpp")
list(FILTER TEST_SOURCES EXCLUDE REGEX "main.cpp")

# foot测试
add_executable(foot_test tests/foot_test.cpp ${TEST_SOURCES})
target_include_directories(foot_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# A*算法测试
add_executable(aStar_test tests/aStar_test.cpp ${TEST_SOURCES})
target_include_directories(aStar_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 地面和平面接触测试
add_executable(ground_test tests/ground_test.cpp ${TEST_SOURCES})
target_include_directories(ground_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 创建目录结构以匹配测试中使用的相对路径
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests)