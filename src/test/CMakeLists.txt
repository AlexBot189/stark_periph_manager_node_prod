cmake_minimum_required(VERSION 3.10)
project(stark_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# demo_algo — 算法控制示例 (纯C, 唯一测试程序)
set_source_files_properties(demo_algo.c PROPERTIES LANGUAGE C)
add_executable(demo_algo demo_algo.c)
target_include_directories(demo_algo PRIVATE ..)
target_link_libraries(demo_algo rt pthread m)
