##########################
#
#	version: 0.1
#  	description: stark_periph_manager_node
#
##########################

cmake_minimum_required(VERSION 2.8.12)
project(stark_periph_manager_node)

option(ENABLE_ROS       "启用 ROS 适配器"   ON)
option(ENABLE_WEBSERVER "启用 Web 调试面板" ON)

option(USE_CROSS_COMPILE "Use Cross-compile" ON)
if(USE_CROSS_COMPILE)
	set(TARGET_ARCHITECTURE "arm64")
else()
	set(TARGET_ARCHITECTURE "amd64")
endif()

include(eco_config.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/exbot/build-dep/rv1126b/build/config)
include(eco_config_global)

set(COMMON_INCLUDE_PATH "")

message("源码根目录: ${CMAKE_CURRENT_SOURCE_DIR}")

# 头文件路径 (仅框架变量, 自定义全在 eco_config.cmake 的 CUSTOM_INLCUDE_PATH 中)
include_directories(
	${CUSTOM_INLCUDE_PATH}
	${EROSMSG_INCLUDE_PATH}
	${PACKAGE_INCLUDE_PATH}
	${COMMON_INCLUDE_PATH}
)

# 库搜索路径
link_directories(
	${CUSTOM_LIBRARY_PATH}
	${PACKAGE_LIBRARY_PATH}
	${COMMON_LIBRARY_PATH}
)

# ════════════════════════════════════════════════════════════
# 主目标: stark_periph_manager_node (eco_build 自动扫描 src/)

include(eco_build)

target_link_libraries(${PROJECT_NAME} gpiod)

if(ENABLE_ROS)
	find_package(roscpp REQUIRED)
	find_package(std_msgs REQUIRED)
	include_directories(${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
	add_definitions(-DENABLE_ROS)
endif()

if(ENABLE_WEBSERVER)
	add_definitions(-DENABLE_WEBSERVER)
endif()

# ════════════════════════════════════════════════════════════
# libmotor_hal.so — 电机 CAN/CANopen 驱动

add_custom_build(
	NAME motor_hal
	TYPE shared
	SRCS
		"${MH_DIR}/can_driver.c"
		";${MH_DIR}/canopen_frames.c"
		";${MH_DIR}/sdo_client.c"
		";${MH_DIR}/nmt_master.c"
		";${MH_DIR}/heartbeat.c"
		";${MH_DIR}/pdo_handler.c"
		";${MH_DIR}/pdo_mapper.c"
		";${MH_DIR}/feedback_parser.c"
		";${MH_DIR}/motor_hal_startup.c"
		";${MH_DIR}/utils.c"
		";${MH_DIR}/motor_hal.c"
	DEPS "pthread;m"
)

# ════════════════════════════════════════════════════════════
# IMU 传感器驱动 — 源码直接编译进主目标 (eco_build 扫描 src/sensor/imu_hal)
#   不再单独建 libimu_hal.so；排除 demo/example (带 main, 与节点冲突)

# ════════════════════════════════════════════════════════════
# imu_read_sensor — IMU 测试程序 (读融合数据, 独立编译 imu_hal 源码)

add_custom_build(
	NAME imu_read_sensor
	TYPE bin
	SRCS
		"${IH_DIR}/example/read_sensor.c"
		";${IH_DIR}/emd_gaf.c"
		";${IH_DIR}/invn_mag_ict1531x.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_driver.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_driver_advanced.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_edmp.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_edmp_wearable.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_edmp_mrm.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_transport.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_i2cm.c"
		";${IH_DIR}/driver/icm45608/imu/inv_imu_selftest.c"
		";${IH_DIR}/driver/icm45608/imu/sif_classifier_ir.c"
		";${IH_DIR}/driver/icm45608/imu/sif_feature_extract_ir.c"
		";${IH_DIR}/driver/Ict1531x/Ict1531x.c"
		";${IH_DIR}/hal/emd_hal.c"
		";${IH_DIR}/tools/Invn/EmbUtils/Message.c"
	DEPS "gpiod;pthread;m"
)

# ════════════════════════════════════════════════════════════
# demo_algo — 调试/算法测试

add_custom_build(
	NAME demo_algo
	TYPE bin
	SRCS "${LOCAL_SRC_PATH}/test/demo_algo.c"
	DEPS "rt;pthread;m"
)

target_link_libraries(${PROJECT_NAME} motor_hal)

include(eco_install)
