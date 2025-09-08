cmake_minimum_required(VERSION 3.10)
project(trapla)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加头文件目录
include_directories(include)

# 添加源文件（排除main.cpp）
file(GLOB_RECURSE SOURCES "src/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "main.cpp")

# 创建主可执行文件
add_executable(trapla src/main.cpp ${SOURCES})

# 添加测试可执行文件
add_executable(aStar_test tests/aStar_test.cpp ${SOURCES})
add_executable(ground_test tests/ground_test.cpp ${SOURCES})
add_executable(utils_test tests/utils_test.cpp ${SOURCES})
add_executable(comparison_test tests/comparison_test.cpp ${SOURCES})

# 链接数学库和文件系统库（如果需要）
if(UNIX)
    target_link_libraries(trapla m)
    target_link_libraries(aStar_test m stdc++fs)
    target_link_libraries(ground_test m stdc++fs)
    target_link_libraries(utils_test m stdc++fs)
    target_link_libraries(comparison_test m stdc++fs)
else()
    # Windows下链接文件系统库
    target_link_libraries(aStar_test stdc++fs)
    target_link_libraries(ground_test stdc++fs)
    target_link_libraries(utils_test stdc++fs)
    target_link_libraries(comparison_test stdc++fs)
endif()