cmake_minimum_required(VERSION 4.0)
set(CMAKE_CXX_STANDARD 17)
project("trapla")

file(GLOB_RECURSE SOURCES src/*.cpp src/**/*.cpp)

add_executable(trapla ${SOURCES})
target_include_directories(trapla PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)