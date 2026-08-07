#!/bin/bash
##########################
#
#  	make.sh
#  	description: stark_periph_manager_node 编译入口
#  	用法: ./make.sh [debug|release]
#
##########################

CUR_DIR=$(pwd)
export ECO_PROJECT_NAME=${CUR_DIR##*/}
export ECO_WORKSPACE_DIR=~/workspace/project/k850/embuild
export DETAILED_BUILDING_MESSAGE=true
export ECO_PKG_PROJECT_NAME=${ECO_PROJECT_NAME}

/home/exbot/build-dep/rv1126b/build/sub_make.sh $@
