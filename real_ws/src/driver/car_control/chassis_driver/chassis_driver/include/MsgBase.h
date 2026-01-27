#ifndef CANLIB_MSG_BASE_H
#define CANLIB_MSG_BASE_H


#include "utils.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sstream>
#include <stdint-gcc.h>
#include <string.h>
#include <string>
#include "controlcan.h"
#include <yunle_msgs/msg/battery.hpp>
#include <yunle_msgs/msg/vehicle_status.hpp>
#include <yunle_msgs/msg/ecu.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <tf/tf.h>

// #include <ros/ros.h>

#include <iostream>

// reveive vehicle status msg protocol
#define CCU_SHIFT_LEVEL_OFFSET 0
#define CCU_SHIFT_LEVEL_LENGTH 2     // 档位  1：D  2：N  3：R
#define CCU_WHEEL_DIRECTION_OFFSET 7 // 实际方向盘角度方向 0左边 1右边
#define CCU_WHEEL_ANGLE_OFFSET 8
#define CCU_WHEEL_ANGLE_LEGNTH 12 // 实际方向盘角度 单位 0.1度  最大值 1200 代表 120度
#define CCU_SPEED_OFFSET 20
#define CCU_SPEED_LENGTH 9 // 当前车速 单位 0.1m/s 最大值 510 代表 51m/s
#define CCU_DRIVE_MODE_OFFSET 29
#define CCU_DRIVE_MODE_LENGTH 3 // 当前驾驶模式 0不影响  1自动驾驶模式  2驾驶员PAD模式  3驾驶员方向盘模式\
                                   自动驾驶模式的优先级低于 方向盘模式 也就是 在自动驾驶模式下，调整控制版，车辆\
                                   优先采取控制版的命令
#define CCU_ACCELERATE_LEVEL_OFFSET 32
#define CCU_ACCELERATE_LEVEL_LENGTH 2 // 当前加速档位
#define CCU_BRAKE_LEVEL_OFFSET 34
#define CCU_BRAKE_LEVEL_LENGTH 2 // 当前减速档位
#define CCU_TOTAL_ODOMETER_OFFSET 36
#define CCU_TOTAL_ODOMETER_LENGTH 20 // 车辆累计里程数 单位：km

// reveive vehicle battery msg protocol
#define BMU_VOLTAGE_OFFSET 0
#define BMU_VOLTAGE_LENGTH 16 // 动力电池 电压 单位：V Physical voltage= BSU_Battery_Out_Voltage * 0.1
#define BMU_AMPERE_OFFSET 16
#define BMU_AMPERE_LENGTH 16 // 动力电池 电流 单位：A Physical current= (BSU_Battery_Out_Current - 4000) * 0.1
#define BMU_CAPACITY_OFFSET 32
#define BMU_CAPACITY_LENGTH 8 // 动力电池 电量 单位：百分比 Physical current= BSU_SysSOC  *  0.4%
#define BMU_SYS_STATUS_OFFSET 40
#define BMU_SYS_STATUS_LENGTH 2 // BSU 系统状态：  0 正常 \
                                                 1 不影响车辆正常行驶的故障\
                                                 2 影响车辆正常行使，需要驾驶员限制驾驶的故障\
                                                 3 驾驶员需要立即停车，并请求救援的故障
#define BMU_CHARGE_STATUS_OFFSET 42 // BSU 充电状态： 0未充电  1正在充电

#define BMU_NEW_VOLTAGE_OFFSET 0
#define BMU_NEW_VOLTAGE_LENGTH 16  // 动力电池 电压V = value * 0.1
#define BMU_NEW_DASHBOARD_OFFSET 16
#define BMU_NEW_DASHBOARD_LENGTH 16 // SOC仪表显示
#define BMU_NEW_AMPERE_OFFSET 32
#define BMU_NEW_AMPERE_LENGTH 16  // 动力电池 电流A = value * 0.1
#define BMU_NEW_SOC_OFFSET 48
#define BMU_NEW_SOC_LENGTH 16  // 动力电池 荷电状态


// 车轮编码器,RPM,扭矩,电流等
#define CCU_TORQUE_MEASURED_OFFSET 0  // 反馈扭矩
#define CCU_TORQUE_MEASURED_LENGTH 16
#define CCU_RPM_MEASURED_OFFSET 16  // 反馈转速RPM
#define CCU_RPM_MEASURED_LENGTH 16
#define CCU_DC_MEASURED_OFFSET 32  // 反馈MCU电流
#define CCU_DC_MEASURED_LENGTH 16


// 车轮轮速反馈，分为前左、前右、后左、后右
#define FRONT_LEFT_RPM 0
#define FRONT_LEFT_RPM_LENGTH 16
#define FRONT_RIGHT_RPM 16
#define FRONT_RIGHT_RPM_LENGTH 16
#define REAR_LEFT_RPM 32
#define REAR_LEFT_RPM_LENGTH 16
#define REAR_RIGHT_RPM 48
#define REAR_RIGHT_RPM_LENGTH 16

#define SPEED_RPM_28X 0
#define SPEED_RPM_28X_LENGTH 32


// For imu, 发送车身倾角和车重给vcu, 用以坡道驻车, 坡道起步等控制
#define SCU_POSTURE_ANGLE_OFFSET 0
#define SCU_POSTURE_ANGLE_LENGTH 16
#define SCU_POSTURE_WEIGHT_OFFSET 16
#define SCU_POSTURE_WEIGHT_LENGTH 32

#define PAYLOAD_LENGTH 8

#define BUFFER_SIZE 8

namespace USB2CAN
{
enum ShiftLevel
{
    NOT_DETECTED = 0,
    D = 1, // 前进
    N = 2, // 停车
    R = 3  // 倒车
};

enum DriveMode
{
    AUTO_MODE = 1,
    PAD_MODE = 2,
    WHEEL_MODE = 3
};

enum SteerDirection
{
    left = 0,
    right = 1
};

class SendMsgBase{
protected:
    yunle_msgs::msg::Ecu ecuMsg;
    VCI_CAN_OBJ canMsg;
    double pre_steer;
    virtual void initCanMsg() {}
    virtual void setDriveMode(DriveMode driveMode) {}
    virtual void setBrakeMode(int v) {}
    virtual void setAccMode(int v) {}
    virtual void setShiftLevel(ShiftLevel shiftLevel) {}
    virtual void setEBrake(bool need = false) {}
    virtual void rosMsg2canMsg() {}
public:
    SendMsgBase() {}

    SendMsgBase(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    SendMsgBase(yunle_msgs::msg::Ecu ecuMsg, double pre_steer)
    {
        this->ecuMsg = ecuMsg;
        this->pre_steer = pre_steer;
    }

    virtual ~SendMsgBase() {}

    virtual VCI_CAN_OBJ getMessage(){VCI_CAN_OBJ obj; return obj;}

    void print()
    {
        fprintf(stdout, "send ecu/CCUMsg: "); // printf:格式化输出到屏幕stdout; fprintf格式化输出到文件FILE; sprintf格式化输出到字符创char*
        fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
        fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
        this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
        this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
        fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
        fprintf(stdout, "Data: ");
        for (int i = 0; i < 8; i++)
        {
            fprintf(stdout, "%02X ", this->canMsg.Data[i]);
        }
        fprintf(stdout, "...\n");
    }

};

class BatteryMsg
{
private:
    VCI_CAN_OBJ canMsg;
    bool protocol_new;

    yunle_msgs::msg::Battery rosMsg_battery_status;

    double getVoltage();

    double getAmpere();

    double getBatteryCapacity();

    int getBsuSysStatus();

    int getChargeStatus();

    void createMessage();

public:
    BatteryMsg(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
        this->protocol_new = false;
    }

    BatteryMsg(VCI_CAN_OBJ in_canMsg, bool protocol_new)
    {
        this->canMsg = in_canMsg;
        this->protocol_new = protocol_new;
    }

    void print();

    yunle_msgs::msg::Battery getMessage();
};

class VehicleStatusMsg
{
private:
    // uint8_t packet[BUFFER_SIZE];
    VCI_CAN_OBJ canMsg;

    yunle_msgs::msg::VehicleStatus rosMsg_vehicle_status;

    int shiftLevel();

    double speed();

    double wheelAngle();

    SteerDirection wheelDirection();

    int driveMode();

    void createMessage();

public:
    VehicleStatusMsg(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    void print();

    std::string toString();

    yunle_msgs::msg::VehicleStatus getMessage();
};

class SendPostureMsg{
protected:
    sensor_msgs::msg::Imu imuMsg;
    VCI_CAN_OBJ canMsg;
    double weight;  // kg
    virtual void initImuMsg();
    virtual void setAngle(double angle);
    virtual void setWeight(double weight);
    virtual void rosMsg2canMsg();
public:
    SendPostureMsg() {
        initImuMsg();
    }

    SendPostureMsg(VCI_CAN_OBJ canData)
    {
        this->canMsg = canData;
    }

    SendPostureMsg(sensor_msgs::msg::Imu imuMsg, double weight)
    {
        this->imuMsg = imuMsg;
        this->weight = weight;
        initImuMsg();
    }

    virtual ~SendPostureMsg() {}

    virtual VCI_CAN_OBJ getMessage();

    void print()
    {
        fprintf(stdout, "send imu/PostureMsg: \n"); // printf:格式化输出到屏幕stdout; fprintf格式化输出到文件FILE; sprintf格式化输出到字符创char*
        fprintf(stdout, "ID: 0x%08X; \n", this->canMsg.ID);
        fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
        this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
        this->canMsg.RemoteFlag == 0 ? printf(" Data     \n") : printf(" Remote   \n");
        fprintf(stdout, "DataLen: %02X; \n", this->canMsg.DataLen);
        fprintf(stdout, "Data: \n");
        for (int i = 0; i < 8; i++)
        {
            fprintf(stdout, "%02X ", this->canMsg.Data[i]);
        }
        fprintf(stdout, "...\n");
    }
};

enum WheelLR{
  WHEEL_LEFT = 1,
  WHEEL_RIGHT = -1
};

class WheelStatus {
private:
    VCI_CAN_OBJ canMsg;
    WheelLR lr;  // 1:左轮, -1:右轮
    int torque = 0;  // 扭矩
    int rpm = 0;     // 转速
    int dc = 0;      // 电流

    int _torque();
    int _rpm();
    int _dc();

public:
    WheelStatus(VCI_CAN_OBJ in_canMsg, WheelLR in_lr){
        this->canMsg = in_canMsg;
        this->lr = in_lr;
        this->init();
    }

    void init();

    int get_torque();

    int get_rpm();

    int get_dc();

};

class WheelSpeed_28x{
private:
    VCI_CAN_OBJ canMsg;
    float speedRpm  = 0.0;
    void _parse_speed_rpm();
    virtual int _adjust_rpm(int rpm);
public:
    WheelSpeed_28x(VCI_CAN_OBJ in_canMsg){
        this->canMsg = in_canMsg;
        this->init();
    }
    void init();
    float get_speed_rpm();

};

class WheelSpeed{
private:
    VCI_CAN_OBJ canMsg;
    float frontLeftRpm  = 0.0;
    float frontRightRpm = 0.0;
    float rearLeftRpm   = 0.0;
    float rearRightRpm  = 0.0;

    void _get_speed_rpm();
    virtual int _adjust_rpm(int rpm);

public:
    WheelSpeed(VCI_CAN_OBJ in_canMsg){
        this->canMsg = in_canMsg;
        this->init();
    }
    void init();
    float get_front_left_rpm();
    float get_front_right_rpm();
    float get_rear_left_rpm();
    float get_rear_right_rpm();
};



}
#endif
