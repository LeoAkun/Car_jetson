# IMU预积分 b0 异常：只读代码审计与离线实验

本文件仅审计和离线计算，不启动 ROS 节点、不读取硬件、不修改生产源码或参数。IMU当前硬件健康结论由主调查的独立采样报告给出。

## 已证实的故障链

`management_stack.log:87–97` 记录 `gtsam::IndeterminantLinearSystemException`，邻近变量 `b0`，进程退出码 -6。`b0` 是 GTSAM 因子图初始 IMU 偏置变量，不是设备错误码。该异常没有被捕获，C++ terminate 导致 SIGABRT，随后 Nav2 依赖的增量里程计缺失。日志并未给出抛出异常的具体 update 调用栈。

源码 `slam/src/LIO-SAM/src/imuPreintegration.cpp`：

- 344、380、424–425 行对 optimizer.update 没有 try/catch；588 行 executor.spin 也无外层异常处理。476–493 行速度/偏置检测仅在 update 成功之后执行，无法保护本次异常。
- 328–342 行初始化了 prevPose_、prevVel_、prevBias_，没有同步初始化 prevState_。419 行首次优化预测使用默认 NavState，而不是初始激光位姿。432 行只有首次优化成功后才同步。此为明确初始化缺口，但不能由现有证据认定它单独导致 b0 异常。
- 257 行初始速度先验 sigma=10000 m/s，约束非常弱；这可能影响数值条件，但单凭此设置不能证明故障。
- 294–424 行未检查 correction 时间严格递增、有效积分样本数、有限数值或累计 deltaT>0。396 行与534行也不拒绝非正或异常大的 IMU dt。413 行用 sqrt(deltaT) 生成偏置噪声；零积分可能构造退化图。
- 两个 IMU/激光回调分别处于不同回调组，但均获取同一 mtx（292、504 行），因此不存在这些回调同时修改因子图的明显数据竞争。不同订阅之间的执行先后仍不保证与录包到达顺序完全相同。
- TransformFusion 104–112 行弹出旧数据后没有再次检查 imuOdomQueue 是否为空即访问 front/back。这是另一个代码缺口，但通常表现为越界或非法访问，不能拿来解释本次已明确记录的 GTSAM b0 异常。

## 实际输入与离线复现结果

bag 审计提取首两次 mapping correction：

1. header=1794210540.1906886，pose=(0,0,0)，cov[0]=0。
2. header=1794210540.2908626，pose=(0.0017968435,-0.0015929659,-0.0000647252)，cov[0]=0。

依 bag 到达顺序构造预积分队列，二者之间有10条有效 IMU，累计 deltaT=0.100204706192017秒；所有 dt 为正。未支持零积分或重复/回退时间这一触发解释。bag到达顺序是订阅端观察，不能精确复原已崩溃进程内每个回调的执行顺序。

使用本机 GTSAM 4.1.1、相同先验、噪声、float类型转换、外参、首两次实际correction和实际10条IMU，构建了完全离线的最小图计算：

- 现有设置（默认prevState、速度sigma=10000、Cholesky）首次图扩展的两次update均通过。
- 将prevState正确初始化、速度sigma改为1或0.1、或改用QR，12种组合均通过。此为诊断实验，未应用到生产参数。
- 人工构造0条IMU或累计1微秒的积分可产生相同 b0 异常，但与本次bag恢复出的0.1002秒正常积分不符，不能作为本次根因。

实验源码 `gtsam_actual_first.cpp`，输入 `first_step_actual.txt`，结果 `actual_first_results.txt` 均在本目录。`gtsam_minimal.cpp` 为静态人工数据的对照实验。

## 安装产物和取证限制

build/installed 二进制整体SHA不同（安装时RPATH变更），但 `.text` 段 SHA256 均为 `c107cadd34f07b482562c54dbba274ea84999ecf155b2d45077b3f98278732b3`，可执行代码一致。build object时间2026-10-15，源码mtime2026-01-29；未发现两者明显时间倒置。库为libgtsam.so.4.1.1。

`/var/lib/systemd/coredump` 为空，`/var/crash` 仅有11月7日ros2的历史报告，未找到本次IMU预积分core。无法还原具体异常update调用栈及运行时矩阵。当前能确定的是预积分因子图数值求解失败与异常处理缺失，不能把未复现的初始化/噪声缺口写成已经锁定的唯一根因。
