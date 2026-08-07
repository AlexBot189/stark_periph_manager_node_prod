#!/bin/bash

rm -rf /home/zhiqiang.yang/workspace/project/k850/embuild/eros/log_helper/build/*
cd /home/zhiqiang.yang/workspace/project/k850/embuild/eros/log_helper/build
cmake ../ -DCMAKE_TOOLCHAIN_FILE=/home/exbot/ros_build_env_melodic/platform/board/rk3576/toolchainfile.cmake -DCMAKE_INSTALL_PREFIX=/home/exbot/build-dep/rk3576/0.1.8/rk3576/usr
make
sudo make unistall
sudo make install
sudo chown exbot:exbot /home/exbot/build-dep/rk3576/0.1.8/ -R
cd -
