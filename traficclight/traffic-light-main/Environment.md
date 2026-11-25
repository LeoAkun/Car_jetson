# Jetson Nano环境配置

## Jetpack

- 烧录4.6.1版本的Jetpack

## 创建Conda环境

- 安装miniforge
    - 选择Linux操作系统，aarch64 (arm64)架构的安装脚本进行下载
- 创建并激活python3.8虚拟环境
    ```shell
    conda create -n tensorrt python=3.8
    conda activate tensorrt
    ```
- 安装pytorch
    - 下载[torch和torchvision](https://pan.baidu.com/s/1Y1XFZGFf8kg5p-aiTtBRqw)的whl文件，提取码: bsum
    - 使用pip安装
        ```shell
        pip install torch-*.whl torchvision-*.whl
        ```
- TensorRT相进性绑定
    - 下载[tensorrt](https://pan.baidu.com/s/1PZBwBbCEMnh5lmq9vII_mA)的whl文件，提取码: 188a
    - 使用pip安装
        ```shell
        python -m pip install tensorrt-*.whl
        ```
- 配置opencv绑定Gstreamer
    - 升级g++和gcc版本到`8.4.0`
    - 克隆opencv仓库
        ```shell
        git clone https://github.com/opencv/opencv.git
        cd opencv
        sudo mkdir build
        cd build
        ```
    - 找到虚拟环境下的libpython3.8.so路径
        ```shell
        sudo find / -name libpython3.8.so
        ```
    - cmake指令
        ```shell
        #cmake指令
        sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D INSTALL_PYTHON_EXAMPLES=ON \
            -D INSTALL_C_EXAMPLES=OFF \
            -D WITH_TBB=ON \
            -D WITH_V4L=ON \
            -D BUILD_TESTS=OFF \
            -D BUILD_PERF_TESTS=OFF \
            -D WITH_QT=ON \
            -D WITH_OPENGL=ON \
            -D WITH_GSTEAMER=ON \ #打开Gstreamer
            -D BUILD_opencv_python3=ON \
            -D BUILD_opencv_python2=OFF \
            -D PYTHON3_LIBRARY=/path/to/your/libpython3.8.so \ #找到虚拟环境tensorrt下的libpython3.8.so
            -D PYTHON_DEFAULT_EXECUTABLE=$(python -c "import sys; print(sys.executable)")   \
            -D PYTHON3_EXECUTABLE=$(python -c "import sys; print(sys.executable)")   \
            -D PYTHON3_NUMPY_INCLUDE_DIRS=$(python -c "import numpy; print (numpy.get_include())") \
            -D PYTHON3_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") ..
        ```
    - cmake build成功后，编译
        ```shell
        sudo make -j8
        sudo make install
        ```
    - 在虚拟环境中查看opencv信息
        ```python
        import cv2
        print(cv2.getBuildInformation())
        ```

## 参考

- [THU-MIG/yolov10](https://github.com/THU-MIG/yolov10)
- [Jetson nano部署YOLOV8并利用TensorRT加速推理实现行人检测追踪](https://zhuanlan.zhihu.com/p/665546297)