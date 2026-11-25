#!/bin/bash

# 检查是否提供了参数
if [ -z "$1" ]; then
  echo "Usage: $0 <pwm_value>"
  exit 1
fi

# 定义要追加的命令
commands=$(cat <<EOF
#!/bin/sh
sleep 10
sudo /usr/bin/jetson_clocks
sudo sh -c 'echo 200 > /sys/devices/pwm-fan/target_pwm'
EOF
)

# 追加命令到 /etc/rc.local 文件
echo "$commands" | sudo tee /etc/rc.local > /dev/null

# 确保 /etc/rc.local 文件具有可执行权限
sudo chmod +x /etc/rc.local