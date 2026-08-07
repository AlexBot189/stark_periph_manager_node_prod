cmake_minimum_required(VERSION 3.0.0)

project(log_helper_test VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)

# 查找源文件
file(GLOB_RECURSE TEST_FILES "src/*.cpp")

# 遍历源文件列表，生成可执行程序
foreach(TEST_FILE IN LISTS TEST_FILES)
    get_filename_component(TARGET_NAME ${TEST_FILE} NAME_WE)
    add_executable(${TARGET_NAME} ${TEST_FILE})
    target_link_libraries(${TARGET_NAME} log_helper pthread)
endforeach()
