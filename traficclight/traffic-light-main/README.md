# Jetson Nano + YOLOv10交通信号灯识别

## 1. 摄像头配置

```shell
# 命令行下
# 修改摄像头参数
v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG
v4l2-ctl --device=/dev/video0 --set-parm=15
# 查看修改结果
v4l2-ctl --device=/dev/video0 --all
# 可以看到有如下参数信息
Video input : 0 (Camera 1: ok)
Format Video Capture:
        Width/Height      : 1920/1080
        Pixel Format      : 'MJPG'
        Field             : None
        Bytes per Line    : 0
        Size Image        : 4147789
        Colorspace        : sRGB
        Transfer Function : Default (maps to sRGB)
        YCbCr/HSV Encoding: Default (maps to ITU-R 601)
        Quantization      : Default (maps to Full Range)
        Flags             : 
Crop Capability Video Capture:
        Bounds      : Left 0, Top 0, Width 1920, Height 1080
        Default     : Left 0, Top 0, Width 1920, Height 1080
        Pixel Aspect: 1/1
```

## 2. 启动服务器

### 2.1 更改配置文件config.json

```json
{
    "socket": {
        "host" : "192.168.20.120",      // ip地址，已弃用
        "port" : 12345,                 // 端口号
        "broadcast_port" : 37020        // 广播端口号
    },
    "camera": {
        "mode" : "camera",              // 为"camera"时调用摄像头，为"video"时使用本地视频测试
        "device" : "/dev/video0",       // 摄像头路径或视频路径
        "width" : 1920,                 // 摄像头宽度，需与查看的摄像头配置一致
        "height" : 1080,                // 摄像头高度，需与查看的摄像头配置一致
        "framerate" : 15,               // 摄像头帧率，需与查看的摄像头配置一致
        "pformat" : "MJPG"              // 摄像头采集图像格式，可用MJPG和YUYV格式，使用MJPG速度更快
    },
    "model": {
        "mpath" : "models/traffic_light.engine",
        "imgsz" : 640,                  // 模型输入大小
        "conf" : 0.5,                   // 置信度
        "save" : false,                 // 是否保存图片结果到"./save"文件夹下用于推流
        "input" : "images",             // 输入层名称，默认即可
        "output" : "output0"            // 输出层名称，默认即可
    },
    "debug": {
        "islog": true                   // 启用DEBUG打印，已弃用
    },
    "app": {
        "freq": 1,                      // 识别（即结果发送）频率
        "python": "/home/dlinano/miniforge3/envs/TensorRT/bin/python",
        "restart_time": 10              // 重启app等待时间（second）
    },
    "updater": {
        "repo_url": "https://gitee.com/bugrequiem/traffic-light.git", //项目远程仓库地址
        "temp_path": "./temp"           // 下载临时位置，已弃用
    }
}
```
- 使用vim编辑器修改config配置
```shell
# 在项目目录（~/workspace/traffic-light）下使用命令
vim config.json
# 上下左右键选择光标位置
# 键盘按下i键进入编辑模式。左下角会出现“-- 插入 --”字样
# 编辑需要更改的配置项
# 按下键盘ESC键退出编辑模式，进入命令模式
# 键盘输入“:wq”按下回车执行命令保存并退出，若不想保存，使用命令“:q!”
```

### 2.2 启动和停止app

```shell
cd ~/workspace/traffic-light        # 进入项目目录
./app_start.sh                      # 启动
./app_stop.sh                       # 停止
```
- 程序启动后，摄像头进程会在初始化结束后立刻运行

### 2.3 简要测试socket接口

- 使用client.py测试socket接口
    ```shell
    python client.py
    ```
- 使用test_cmd.py测试socket接口
    ```shell
    python test_cmd.py
    # test_cmd.py启动后，键盘输入start或stop并回车就能开启/停止检测
    ```

### 2.4 日志文件

- app运行日志文件为`app.log`，仅包含app自身运行时的日志输出
- app所有输出日志文件为`app_start.log`，包含app以及app所使用的库的所有日志输出
- 查看日志
    ```shell
    # 可将app.log换成app_start.log
    cat app.log                 # 查看所有级别的日志
    cat app.log | grep "ERROR"  # 查看错误级别的日志
    ```

### 2.5 异常处理

- 当socket出现异常，将尝试重新启动socket进程，且模型识别进程将**暂停**，客户端需要**重新连接**
- 当摄像头出现异常，将关闭摄像头进程并向客户端发送错误消息，且模型识别进程将**暂停**。程序会自动尝试重新启动摄像头进程，需要客户端重新发送启动检测消息。
- 当模型检测出现异常，将关闭检测进程并向客户端发送错误消息，摄像头进程仍然会**继续执行**。程序会自动尝试重新启动检测进程，需要客户端重新发送启动检测消息。

### 2.6 设置程序和风扇开机自启动

```shell
# 在项目目录下运行以下命令
./add_reboot_cron.sh        # 开启程序开机自启动
./fan.sh                    # 开启风扇开机自启动
```

### 2.7 app更新

```shell
# 手动更新
# 在项目目录下运行以下命令
./app_update.sh
```
- 手动启动app后或开机自启动后app会自动更新
- app自动重启时不会更新

### 2.8 推流识别图像

```shell
# 进入workspace文件夹下载mjpg-streamer库
cd ~/workspace
git clone https://gitee.com/bugrequiem/mjpg-streamer.git
# 进入项目目录
cd ~/workspace/traffic-light
# 在项目目录下运行以下命令开启推流进程
# 如果推流进程已经开启，则会关闭推流进程
./mjpeg-streamer.sh
# 开启推流进程以后，局域网下访问设备ip的8080端口即可查看推流页面
```