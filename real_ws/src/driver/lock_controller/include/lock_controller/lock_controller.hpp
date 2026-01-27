#ifndef LOCK_CONTROLLER_HPP_
#define LOCK_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "lock_controller/msg/lock_status.hpp"
#include "lock_controller/msg/all_lock_status.hpp"
#include "lock_controller/srv/unlock_command.hpp"
#include "lock_controller/srv/query_lock_status.hpp"
#include "lock_controller/srv/unlock_multiple.hpp"
#include "lock_controller/srv/long_power_control.hpp"

namespace lock_controller
{

class LockController : public rclcpp::Node
{
public:
  LockController();
  ~LockController();

private:
  // RS485串口相关
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;
  uint8_t default_board_address_;

  // 发布器
  rclcpp::Publisher<msg::LockStatus>::SharedPtr lock_status_pub_;
  rclcpp::Publisher<msg::AllLockStatus>::SharedPtr all_lock_status_pub_;

  // 服务
  rclcpp::Service<srv::UnlockCommand>::SharedPtr unlock_service_;
  rclcpp::Service<srv::QueryLockStatus>::SharedPtr query_status_service_;
  rclcpp::Service<srv::UnlockMultiple>::SharedPtr unlock_multiple_service_;
  rclcpp::Service<srv::LongPowerControl>::SharedPtr long_power_service_;

  // 定时器用于接收主动反馈
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // 初始化串口
  bool initSerial();

  // 计算BCC/异或校验
  uint8_t calculateChecksum(const std::vector<uint8_t>& data);

  // 发送命令并接收响应
  bool sendCommand(const std::vector<uint8_t>& command, std::vector<uint8_t>& response, int timeout_ms = 200);

  // 服务回调函数
  void unlockCallback(
    const std::shared_ptr<srv::UnlockCommand::Request> request,
    std::shared_ptr<srv::UnlockCommand::Response> response);

  void queryStatusCallback(
    const std::shared_ptr<srv::QueryLockStatus::Request> request,
    std::shared_ptr<srv::QueryLockStatus::Response> response);

  void unlockMultipleCallback(
    const std::shared_ptr<srv::UnlockMultiple::Request> request,
    std::shared_ptr<srv::UnlockMultiple::Response> response);

  void longPowerControlCallback(
    const std::shared_ptr<srv::LongPowerControl::Request> request,
    std::shared_ptr<srv::LongPowerControl::Response> response);

  // 定时器回调：接收主动反馈
  void feedbackTimerCallback();

  // 协议命令构建
  std::vector<uint8_t> buildUnlockCommand(uint8_t board_addr, uint8_t lock_addr);
  std::vector<uint8_t> buildQueryCommand(uint8_t board_addr, uint8_t lock_addr);
  std::vector<uint8_t> buildQueryAllCommand(uint8_t board_addr);
  std::vector<uint8_t> buildUnlockMultipleCommand(uint8_t board_addr, const std::vector<bool>& lock_mask);
  std::vector<uint8_t> buildLongPowerCommand(uint8_t board_addr, uint8_t lock_addr, bool enable);

  // 解析响应
  bool parseUnlockResponse(const std::vector<uint8_t>& response, bool& is_locked);
  bool parseQueryResponse(const std::vector<uint8_t>& response, bool& is_locked);
  bool parseQueryAllResponse(const std::vector<uint8_t>& response, std::vector<bool>& lock_states);
};

} // namespace lock_controller

#endif // LOCK_CONTROLLER_HPP_
