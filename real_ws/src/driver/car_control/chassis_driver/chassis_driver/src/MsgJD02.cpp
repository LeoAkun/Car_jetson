#include "MsgJD02.h"


namespace USB2CAN
{
    void MsgJD02::initCanMsg(){
        bzero(canMsg.Data, BUFFER_SIZE);
        canMsg.ID = 0x122;
        canMsg.SendType = 0x01;   // 0:正常发送,发送失败则重发  1:只发送一次
        canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
        canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
        canMsg.DataLen = 0x08;

        this->setAccMode(1);           // 设置加速档位
        this->setDriveMode(AUTO_MODE); // 设置驾驶模式 ： 自动驾驶
        this->setBrakeMode(1);         // 设置减速档位
    }

    void MsgJD02::setDriveMode(DriveMode driveMode){
        writeInt(canMsg.Data, JD02_SCU_DRIVE_MODE_REQ_OFFSET, JD02_SCU_DRIVE_MODE_REQ_LENGTH, driveMode);
    }

    void MsgJD02::setBrakeMode(int v){
        writeInt(canMsg.Data, JD02_SCU_BRAKE_MODE_OFFSET, JD02_SCU_BRAKE_MODE_LENGTH, v);
    }

    void MsgJD02::setAccMode(int v){
        writeInt(canMsg.Data, JD02_SCU_ACC_MODE_OFFSET, JD02_SCU_ACC_MODE_LENGTH, v);
    }

    void MsgJD02::setShiftLevel(ShiftLevel shiftLevel){
        writeInt(canMsg.Data, JD02_SCU_SHIFT_LEVEL_OFFSET, JD02_SCU_SHIFT_LEVEL_LENGTH, shiftLevel);
    }

    void MsgJD02::setEBrake(bool need){
        int v = 0;
        if (need)
        {
            v = 1;
        }
        writeInt(canMsg.Data, JD02_SCU_EBRAKE_OFFSET, JD02_SCU_EBRAKE_LENGTH, v);
    }

    void MsgJD02::rosMsg2canMsg(){
        this->setBrakeMode(ecuMsg.brake);
        this->setShiftLevel(ShiftLevel(ecuMsg.shift));
        this->setDifferentialDrive(ecuMsg.motor, ecuMsg.steer);
        this->setEBrake(ecuMsg.brake);
    }

    // Tag: Control steer calibration here
    void MsgJD02::setDifferentialDrive(double v, double angle){
        // 0.4m是轮距
        writeInt(canMsg.Data, JD02_SCU_LMOTOR_SPD_RPM_OFFSET, JD02_SCU_LMOTOR_SPD_RPM_LENGTH, v*100+0.4*angle);
        writeInt(canMsg.Data, JD02_SCU_RMOTOR_SPD_RPM_OFFSET, JD02_SCU_RMOTOR_SPD_RPM_LENGTH, v*100-0.4*angle);
    }

    VCI_CAN_OBJ MsgJD02::getMessage(){
        this->rosMsg2canMsg();
        VCI_CAN_OBJ return_msg = this->canMsg;
        return return_msg;
    }

}
