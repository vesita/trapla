cmake_minimum_required(VERSION 3.10)
project(trapla)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB SOURCE src/*/*.cpp)

# 添加可执行文件
add_executable(trapla src/main.cpp 
                    ${SOURCE})

# 添加测试文件
add_executable(constraints_test tests/constraints_test.cpp 
                    ${SOURCE})

# 设置包含目录
target_include_directories(trapla PRIVATE include)
target_include_directories(constraints_test PRIVATE include)

# 链接数学库（在某些系统上需要）
if(WIN32)
    target_link_libraries(trapla PRIVATE ws2_32)
    target_link_libraries(constraints_test PRIVATE ws2_32)
else()
    target_link_libraries(trapla PRIVATE m)
    target_link_libraries(constraints_test PRIVATE m)
endif()

# 指定C++标准
set_target_properties(trapla PROPERTIES CXX_STANDARD 17)
set_target_properties(constraints_test PROPERTIES CXX_STANDARD 17)