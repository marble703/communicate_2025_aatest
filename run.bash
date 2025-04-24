#!/bin/bash

colcon build --symlink-install
source install/setup.bash

# 检测是否可以使用 gnome-terminal
USE_GNOME_TERMINAL=false
if command -v gnome-terminal &> /dev/null; then
    # 尝试打开一个临时窗口，检测是否成功
    if gnome-terminal -- bash -c "exit 0" &> /dev/null; then
        USE_GNOME_TERMINAL=true
    fi
fi

# 打不开gnome-terminal就用xterm
if $USE_GNOME_TERMINAL; then
    TERMINAL="gnome-terminal --title"
    CMD_SUFFIX="-- bash -c"
else
    TERMINAL="xterm -title"
    CMD_SUFFIX="-e bash -c"
fi


# 启动 communicate_2025
$TERMINAL "Communicate 2025" $CMD_SUFFIX "ros2 launch communicate_2025 launch.py; exec bash" &

# 启动 communicate_2025_aatest
$TERMINAL "Communicate 2025 aatest" $CMD_SUFFIX "ros2 launch communicate_2025_aatest launch.py; exec bash" &

# 等待用户输入 'q' 退出
while true; do
    read -r -n 1 -s input
    
    if [[ "$input" == "q" ]]; then
        break
    fi
done

# 获取并杀死相关进程
pid1=$(ps x | grep "communicate_2025" | grep -v grep | awk '{print $1}')
pid2=$(ps x | grep "communicate_2025_aatest" | grep -v grep | awk '{print $1}')
kill $pid1 $pid2