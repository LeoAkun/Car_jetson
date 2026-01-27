# USB2CAN_alystii driver
This module is used to drive the USB2CAN_alystii device

注:此文件说明使用的是USB接口的CAN盒, 本公司已不提供技术支持, 仅供参考

## 1.USB2CAN_alystii介绍

## 2.USB2CAN_alystii简单使用

## 3. 固定设备权限
在Linux系统下,USB2CAN_alystii需要设备普通用户可读写权限以使用该设备,而每次插入设备都会被分配不同的设备端口号,无法通过脚本来执行,因此需要使用`udev`的规则,通过设备的`idVendor`和`idProduct`自动识别该设备,并设置可读写权限.

1. 执行 lsusb 来查看usb设备的 idVendor idProduct
2. 在 /etc/udev/rules.d/ 中写入.rules文件，文件名以数字开头，数字小的优先级高
例如：
执行以下命令
```
echo  'ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", MODE:="0666", GROUP:="dialout",  SYMLINK+="usb2can_alystii"' >/etc/udev/rules.d/70-usb2can-alystii.rules
```
(ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053"来自步骤1)
(SYMLINK+="usb2can_alystii" 为自己定义的设备名称)
(/etc/udev/rules.d/ 为 .rules 文件所在路径)
 然后重新拔插设备.
 ** GROUP 可以不用 **
***注:有的教程里会加入KERNEL="ttyUSB"等之类,但是KERNEL值设置错误会导致无法自动识别设备,因此在不确定Kernel Value的情况下,可以不加KERNEL字段,同样能够达到预期目的 ***
3. 执行 service udev reload
        service udev restart
4. 在launch文件中使用该usb设备的node处增加：
    <param name="serial_port" type="string" value="/dev/usb2can_alystii"/>
    其中serial_port为ros代码中调用的端口名称，usb2can_alystii为步骤2中定义的名称
    或者：修改<node>中的端口名称为/dev/usb2can_alystii
5. 使用ls /dev/usb2can_alystii来查看是否成功

**TODO::研究udev规则机制**

## 4. USB2CAN_alystii驱动主要接口函数说明

## 5. Subscriber/Publisher & Topic
**Subscriber:**  

| type | topic |
| :--- | :---  |
| yunle_msgs::ecu | "/ecu" |  

**Publisher:**  

| type | topic |
| :--- | :---  |
| yunle_msgs::vehicle_status | "/vehicle_status" |
| yunle_msgs::battery | "/battery" |

## 20220625更新
增加JD01车型的can驱动。对于转角已进行修正，发送10°则代表前轮转角为10度。

## TODO::
1. 增加车型和协议对照表
2. 其他车型修正角度和发送数值之间的关系
3. JD01增加前转和前后双转两种协议