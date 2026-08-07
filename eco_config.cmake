##########################
#
#  	eco_config.cmake
#  	description: stark_periph_manager_node 编译配置
#
##########################

set(EROSMSG_INCLUDE_PATH ${ECO_WORKSPACE_DIR}/eros/release/include)

set(PROJECT_TYPE_NAME "rk3576")

add_definitions(-DMODULE_NAME="stark_periph_node")

set(NEED_ENCRYPT FALSE)
set(NO_STRICT TRUE)
set(MEMORY_CHECK_TYPE "no")
set(NEED_SYMBOLS TRUE)

# 编译目标: 可执行文件
set(COMPILE_TARGET_TYPE "bin")

# 依赖库
set(DEPENDENCY_HARDWARE_LIST "")
set(DEPENDENCY_THIRD_PARTY_LIST "log_helper")
set(DEPENDENCY_SYSTEM_LIST "pthread;rt")
if(ENABLE_ROS)
	set(DEPENDENCY_ROS_LIST "roscpp;std_msgs")
else()
	set(DEPENDENCY_ROS_LIST "")
endif()

set(PUBLIC_HEADER_FOLDER "")

# 主目标忽略目录 (由 add_custom_build 独立编译)
set(IGNORE_SOURCES_FOLDER
	"src/test"
	"src/3rd_party"
	"src/log_helper"
	"src/doc"
	"src/config"
	"src/motor_hal"
	"src/imu_hal"
)
if(NOT ENABLE_ROS)
	list(APPEND IGNORE_SOURCES_FOLDER "src/ros")
endif()
if(NOT ENABLE_WEBSERVER)
	list(APPEND IGNORE_SOURCES_FOLDER "src/web")
endif()

set(IGNORE_SOURCES_FILES "")

set(ECO_CMAKE_LOG_LEVEL 1)

# 源码路径
set(LOCAL_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(LOCAL_INCLUDE_PATH "${LOCAL_SRC_PATH}")

set(MH_DIR "${LOCAL_SRC_PATH}/motor_hal")
set(IH_DIR "${LOCAL_SRC_PATH}/imu_hal")
set(LOG_DIR "${LOCAL_SRC_PATH}/log_helper")

# 平台配置
if(${BUILD_PLATFORM} STREQUAL "rv1126b")
	set(CUSTOM_LIBRARY_PATH
		"${LOG_DIR}/build"
		";${IH_DIR}/hal/gpiod/lib"
	)
	set(CUSTOM_INLCUDE_PATH
		"${ECO_WORKSPACE_DIR}/eros/release/include/"
		";${LOCAL_INCLUDE_PATH}"
		";${MH_DIR}"
		";${MH_DIR}/inc"
		";${IH_DIR}"
		";${IH_DIR}/inc"
		";${IH_DIR}/driver"
		";${IH_DIR}/driver/icm45608"
		";${IH_DIR}/driver/icm45608/imu"
		";${IH_DIR}/driver/Ict1531x"
		";${IH_DIR}/hal"
		";${IH_DIR}/hal/gpiod/include"
		";${IH_DIR}/tools"
		";${IH_DIR}/tools/Invn/EmbUtils"
		";${LOCAL_SRC_PATH}/3rd_party"
		";/home/exbot/build-dep/rv1126b/0.1.8/rv1126b/usr/include"
		";/home/exbot/ros_build_env_melodic/rk3576/ros/staging/root/opt/ros/melodic/include"
		";/home/exbot/ros_build_env_melodic/rk3576/boost/install/root/usr/include"
	)
elseif(${BUILD_PLATFORM} STREQUAL "rk3576")
	set(CUSTOM_LIBRARY_PATH
		"${LOG_DIR}/build"
		";${IH_DIR}/hal/gpiod/lib"
	)
	set(CUSTOM_INLCUDE_PATH
		"${ECO_WORKSPACE_DIR}/eros/release/include/"
		";${LOCAL_INCLUDE_PATH}"
		";${MH_DIR}"
		";${MH_DIR}/inc"
		";${IH_DIR}"
		";${IH_DIR}/inc"
		";${IH_DIR}/driver"
		";${IH_DIR}/driver/icm45608"
		";${IH_DIR}/driver/icm45608/imu"
		";${IH_DIR}/driver/Ict1531x"
		";${IH_DIR}/hal"
		";${IH_DIR}/hal/gpiod/include"
		";${IH_DIR}/tools"
		";${IH_DIR}/tools/Invn/EmbUtils"
		";${LOCAL_SRC_PATH}/3rd_party"
		";${LOCAL_SRC_PATH}/3rd_party/usr/include"
	)
elseif(${BUILD_PLATFORM} STREQUAL "x86")
	set(CUSTOM_LIBRARY_PATH
		"${LOG_DIR}/build"
	)
	set(CUSTOM_INLCUDE_PATH
		"/opt/ros/melodic/include"
		";${LOCAL_INCLUDE_PATH}"
		";${MH_DIR}"
		";${MH_DIR}/inc"
		";${IH_DIR}"
		";${IH_DIR}/inc"
		";${IH_DIR}/driver"
		";${IH_DIR}/driver/icm45608"
		";${IH_DIR}/driver/icm45608/imu"
		";${IH_DIR}/driver/Ict1531x"
		";${IH_DIR}/hal"
		";${IH_DIR}/hal/gpiod/include"
		";${IH_DIR}/tools"
		";${IH_DIR}/tools/Invn/EmbUtils"
		";${LOCAL_SRC_PATH}/3rd_party"
		";${LOCAL_SRC_PATH}/3rd_party/usr/include"
	)
endif()

if(${BUILD_TYPE} STREQUAL "debug")
else()
endif()
