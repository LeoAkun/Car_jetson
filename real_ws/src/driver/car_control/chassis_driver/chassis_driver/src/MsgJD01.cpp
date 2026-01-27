#include "MsgJD01.h"


namespace USB2CAN
{
    void MsgJD01::initCanMsg(){
        bzero(canMsg.Data, BUFFER_SIZE);
        canMsg.ID = 0x121;
        canMsg.SendType = 0x01;   // 0:正常发送,发送失败则重发  1:只发送一次
        canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
        canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
        canMsg.DataLen = 0x08;

        this->setAccMode(1);           // 设置加速档位
        this->setDriveMode(AUTO_MODE); // 设置驾驶模式 ： 自动驾驶
        this->setBrakeMode(1);         // 设置减速档位
    }

    void MsgJD01::setDriveMode(DriveMode driveMode){
        writeInt(canMsg.Data, JD01_SCU_DRIVE_MODE_REQ_OFFSET, JD01_SCU_DRIVE_MODE_REQ_LENGTH, driveMode);
    }

    void MsgJD01::setBrakeMode(int v){
        writeInt(canMsg.Data, JD01_SCU_BRAKE_MODE_OFFSET, JD01_SCU_BRAKE_MODE_LENGTH, v);
    }

    void MsgJD01::setAccMode(int v){
        writeInt(canMsg.Data, JD01_SCU_ACC_MODE_OFFSET, JD01_SCU_ACC_MODE_LENGTH, v);
    }

    void MsgJD01::setShiftLevel(ShiftLevel shiftLevel){
        writeInt(canMsg.Data, JD01_SCU_SHIFT_LEVEL_OFFSET, JD01_SCU_SHIFT_LEVEL_LENGTH, shiftLevel);
    }

    void MsgJD01::setEBrake(bool need){
        int v = 0;
        if (need)
        {
            v = 1;
        }
        writeInt(canMsg.Data, JD01_SCU_EBRAKE_OFFSET, JD01_SCU_EBRAKE_LENGTH, v);
    }
    void MsgJD01::setTorque(bool need){
        int _v = 0;
        if (need)
        {
            _v = 1;
        }
        writeInt(canMsg.Data, JD01_SCU_TORQUE_SHIFT_OFFSET, JD01_SCU_TORQUE_SHIFT_LENGTH, _v);
    }
    

    void MsgJD01::rosMsg2canMsg(){
        this->setBrakeMode(ecuMsg.brake);
        this->setShiftLevel(ShiftLevel(ecuMsg.shift));
        this->setSpeed(ecuMsg.motor);
        this->setWheelAngle(ecuMsg.steer, ecuMsg.rear_wheel_flag);
        this->setEBrake(ecuMsg.brake);
        this->setTorque(ecuMsg.set_torque);
    }

    void MsgJD01::setSpeed(double v) {
        int speed = (int)lround(v * 10);
        writeInt(canMsg.Data, JD01_SCU_TARGET_SPEED_OFFSET, JD01_SCU_TARGET_SPEED_LENGTH, speed);
    }

    void MsgJD01::setWheelAngle(double angle, bool rear_wheel_flag) {
        // 注意: JD01的转角为1精度
        // 120对应27°前轮转角, 即实际ratio = 120 / 27 = 4.444
        int v = int(angle*4.4444);
        // jd01转角极限值为120, 超过将引起反转
        if (v > 120) v = 120;
        if (v < -120) v = -120;
        //if (ecuMsg.shift == ecuMsg.SHIFT_D)
        {
            writeInt(canMsg.Data, JD01_SCU_STEER_WHEEL_F_OFFSET, JD01_SCU_STEER_WHEEL_F_LENGTH, v);
            // else if (ecuMsg.shift == ecuMsg.SHIFT_R)
            if(rear_wheel_flag)  //后轮转向
            {
                writeInt(canMsg.Data, JD01_SCU_STEER_WHEEL_R_OFFSET, JD01_SCU_STEER_WHEEL_R_LENGTH, v);
            }
        }
    }

    VCI_CAN_OBJ MsgJD01::getMessage(){
        this->rosMsg2canMsg();
        VCI_CAN_OBJ return_msg = this->canMsg;
        return return_msg;
    }

}
