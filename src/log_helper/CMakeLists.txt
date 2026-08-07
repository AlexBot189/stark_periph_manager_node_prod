cmake_minimum_required(VERSION 3.0.0)

project(log_helper VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(log_helper SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/LogHelper.cpp
)

########################install########################
# 设置默认值
set(MYPROJECT_INSTALL_PREFIX "/usr")

# 如果命令行有设置，则以命令行为准
if(${CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT})
  set(CMAKE_INSTALL_PREFIX ${MYPROJECT_INSTALL_PREFIX} CACHE PATH "" FORCE)
endif()

# 其他模块可以直接使用include_directories和target_link_libraries，头文件只写到这一层级
# set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(${PROJECT_NAME}_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_PROJECT_NAME}/lib${PROJECT_NAME}.so)

# 导出cmake文件
configure_file(log_helperConfig.cmake.in log_helperConfig.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/log_helperConfig.cmake DESTINATION share/${PROJECT_NAME}/cmake)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_cmake
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}_cmake
        FILE log_helperTargetConfig.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
        # DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY include/spdlog DESTINATION include)
install(FILES include/LogHelper.h DESTINATION include/${PROJECT_NAME})

message(STATUS "CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT: " ${CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT})
message(STATUS "DEFAULT INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
    COMMENT "Removing installed files")

option(BUILD_TEST "Build test target" OFF)
if(BUILD_TEST)
  add_subdirectory(test)
endif()