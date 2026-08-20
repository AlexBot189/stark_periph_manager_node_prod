cmake_minimum_required(VERSION 3.10)
project(stark_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# demo_algo — 算法控制示例 (纯C, 唯一测试程序)
set_source_files_properties(demo_algo.c PROPERTIES LANGUAGE C)
add_executable(demo_algo demo_algo.c)
target_include_directories(demo_algo PRIVATE ..)
# 定义 STARK_LATENCY_TRACE=1: 让 stark_client.h 的 _stark_now_us() 打真实时间戳,
# 否则 mailbox 帧 timestamp_us=0, RT 侧 ctrl_e2e/mbox_age 端到端测量会失效.
target_compile_definitions(demo_algo PRIVATE STARK_LATENCY_TRACE=1)
target_link_libraries(demo_algo rt pthread m)
