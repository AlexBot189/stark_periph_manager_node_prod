#!/bin/bash

rm -rf /home/colin/workspace/project/k850/embuild/eros/log_helper/build/*
cd /home/colin/workspace/project/k850/embuild/eros/log_helper/build
cmake ../ -DCMAKE_INSTALL_PREFIX=/home/exbot/build-dep/k850/0.1.8/x86/usr
make
sudo make unistall
sudo make install
sudo chown exbot:exbot /home/exbot/build-dep/k850/0.1.8/ -R
cd -