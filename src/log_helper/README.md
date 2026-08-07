## 编译命令

### arm

```shell
cmake ../ -DCMAKE_TOOLCHAIN_FILE=/home/exbot/ros_build_env_melodic/platform/board/k850/toolchainfile.cmake -DCMAKE_INSTALL_PREFIX=/home/exbot/build-dep/k850/0.1.8/k850/usr && make
```

### x86

```shell
cmake ../ -DCMAKE_INSTALL_PREFIX=/home/exbot/build-dep/k850/0.1.8/x86/usr && make
```



## `log_helper`日志库使用说明

### 常规使用

若无特殊要求，各模块在使用日志库的时候，只需要设置第`1`项就可以了，其它的均使用默认值就好！

```shell
# 1.需要在eco_config.cmake中设置日志模块名，日志文件名及其父目录就是这个名字
add_definitions(-DMODULE_NAME="eros_master")
```

### 进阶：环境变量设置

==以下属于进阶用法，供了解！==

```shell
# 2.设置日志大小，此处单位为kb（默认单个日志大小为10M：10 * 1024）
export ECO_LOG_SIZE=10240

# 3.设置日志滚动数据（默认为1：数量为1，则最多会生成2个文件）
export ECO_LOG_COUNT=1

# 4.设置日志等级（默认为2：可以输字符串，也可以输枚举值0~6）
# trace debug info warn err fatal off
export ECO_LOG_LEVEL=2 或者 export ECO_LOG_LEVEL=info

# 6.设置日志输出的地方（默认为2：可以输字符串，也可以输枚举值0~2）
# console file console_file
export ECO_LOG_SINK=2 或者 export ECO_LOG_SINK=console_file

# 6.设置日志flush时间（默认为5s）
export ECO_LOG_TIME=5

# 7.设置日志持久化目录的home目录(默认为空)
export ECO_HOME=""

# 8.设置日志持久化的路径（默认为/tmp/log）,实际的路径为: ECO_HOME + ECO_LOG_PATH + MOUDLE_NAME
# 非特殊模块，不要去改这个变量！！！
export ECO_LOG_PATH=/tmp/log
```

