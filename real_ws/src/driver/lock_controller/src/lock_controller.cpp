#include "lock_controller/lock_controller.hpp"
#include <chrono>
#include <cstring>

namespace lock_controller
{

LockController::LockController() : Node("lock_controller")
{
  // 声明参数
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB3");
  this->declare_parameter<int>("baud_rate", 9600);
  this->declare_parameter<int>("board_address", 1);

  // 获取参数
  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("baud_rate", baud_rate_);
  int board_addr;
  this->get_parameter("board_address", board_addr);
  default_board_address_ = static_cast<uint8_t>(board_addr);

  RCLCPP_INFO(this->get_logger(), "Lock Controller initializing...");
  RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "Board address: %d", default_board_address_);

  // 初始化串口
  if (!initSerial()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
    return;
  }

  // 创建发布器
  lock_status_pub_ = this->create_publisher<msg::LockStatus>("lock_status", 10);
  all_lock_status_pub_ = this->create_publisher<msg::AllLockStatus>("all_lock_status", 10);

  // 创建服务
  unlock_service_ = this->create_service<srv::UnlockCommand>(
    "unlock",
    std::bind(&LockController::unlockCallback, this, std::placeholders::_1, std::placeholders::_2));

  query_status_service_ = this->create_service<srv::QueryLockStatus>(
    "query_status",
    std::bind(&LockController::queryStatusCallback, this, std::placeholders::_1, std::placeholders::_2));

  unlock_multiple_service_ = this->create_service<srv::UnlockMultiple>(
    "unlock_multiple",
    std::bind(&LockController::unlockMultipleCallback, this, std::placeholders::_1, std::placeholders::_2));

  long_power_service_ = this->create_service<srv::LongPowerControl>(
    "long_power_control",
    std::bind(&LockController::longPowerControlCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 创建定时器接收主动反馈
  feedback_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&LockController::feedbackTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Lock Controller initialized successfully");
}

LockController::~LockController()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
  }
}

bool LockController::initSerial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Error getting serial attributes");
    close(serial_fd_);
    return false;
  }

  // 设置波特率
  speed_t speed = B9600;
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1
  tty.c_cflag &= ~PARENB;  // 无奇偶校验
  tty.c_cflag &= ~CSTOPB;  // 1个停止位
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8位数据位
  tty.c_cflag &= ~CRTSCTS; // 无硬件流控
  tty.c_cflag |= CREAD | CLOCAL; // 开启接收，忽略modem控制线

  // 原始模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  // 设置超时
  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Error setting serial attributes");
    close(serial_fd_);
    return false;
  }

  // 清空缓冲区
  tcflush(serial_fd_, TCIOFLUSH);

  return true;
}

uint8_t LockController::calculateChecksum(const std::vector<uint8_t>& data)
{
  uint8_t checksum = 0;
  for (uint8_t byte : data) {
    checksum ^= byte;
  }
  return checksum;
}

bool LockController::sendCommand(const std::vector<uint8_t>& command, std::vector<uint8_t>& response, int timeout_ms)
{
  // 发送命令
  ssize_t written = write(serial_fd_, command.data(), command.size());
  if (written != static_cast<ssize_t>(command.size())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write command to serial port");
    return false;
  }

  // 等待响应
  auto start_time = std::chrono::steady_clock::now();
  response.clear();

  while (true) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

    if (elapsed > timeout_ms) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for response");
      return false;
    }

    uint8_t buffer[256];
    ssize_t n = read(serial_fd_, buffer, sizeof(buffer));

    if (n > 0) {
      response.insert(response.end(), buffer, buffer + n);

      // 检查是否收到完整响应 (至少5个字节)
      if (response.size() >= 5) {
        // 忽略82开头的主动反馈，只接受8A或80开头的被动反馈
        if (response[0] == 0x8A || response[0] == 0x80 || response[0] == 0x90) {
          return true;
        } else if (response[0] == 0x82) {
          // 忽略主动反馈，继续等待
          response.clear();
        }
      }
    }

    usleep(10000); // 10ms
  }

  return false;
}

std::vector<uint8_t> LockController::buildUnlockCommand(uint8_t board_addr, uint8_t lock_addr)
{
  std::vector<uint8_t> cmd = {0x8A, board_addr, lock_addr, 0x11};
  uint8_t checksum = calculateChecksum(cmd);
  cmd.push_back(checksum);
  return cmd;
}

std::vector<uint8_t> LockController::buildQueryCommand(uint8_t board_addr, uint8_t lock_addr)
{
  std::vector<uint8_t> cmd = {0x80, board_addr, lock_addr, 0x33};
  uint8_t checksum = calculateChecksum(cmd);
  cmd.push_back(checksum);
  return cmd;
}

std::vector<uint8_t> LockController::buildQueryAllCommand(uint8_t board_addr)
{
  std::vector<uint8_t> cmd = {0x80, board_addr, 0x00, 0x33};
  uint8_t checksum = calculateChecksum(cmd);
  cmd.push_back(checksum);
  return cmd;
}

std::vector<uint8_t> LockController::buildUnlockMultipleCommand(uint8_t board_addr, const std::vector<bool>& lock_mask)
{
  // 将锁掩码转换为3个字节
  uint8_t state1 = 0, state2 = 0, state3 = 0;

  // state3: 锁1-8
  for (size_t i = 0; i < 8 && i < lock_mask.size(); i++) {
    if (lock_mask[i]) {
      state3 |= (1 << i);
    }
  }

  // state2: 锁9-16
  for (size_t i = 8; i < 16 && i < lock_mask.size(); i++) {
    if (lock_mask[i]) {
      state2 |= (1 << (i - 8));
    }
  }

  // state1: 锁17-24
  for (size_t i = 16; i < 24 && i < lock_mask.size(); i++) {
    if (lock_mask[i]) {
      state1 |= (1 << (i - 16));
    }
  }

  std::vector<uint8_t> cmd = {0x90, board_addr, state1, state2, state3};
  uint8_t checksum = calculateChecksum(cmd);
  cmd.push_back(checksum);
  return cmd;
}

std::vector<uint8_t> LockController::buildLongPowerCommand(uint8_t board_addr, uint8_t lock_addr, bool enable)
{
  uint8_t header = enable ? 0x9A : 0x9B;
  std::vector<uint8_t> cmd = {header, board_addr, lock_addr, 0x11};
  uint8_t checksum = calculateChecksum(cmd);
  cmd.push_back(checksum);
  return cmd;
}

bool LockController::parseUnlockResponse(const std::vector<uint8_t>& response, bool& is_locked)
{
  if (response.size() < 5) {
    return false;
  }

  if (response[0] != 0x8A) {
    return false;
  }

  // 状态位：0x00表示开锁成功，0x11表示锁关闭
  is_locked = (response[3] == 0x11);
  return true;
}

bool LockController::parseQueryResponse(const std::vector<uint8_t>& response, bool& is_locked)
{
  if (response.size() < 5) {
    return false;
  }

  if (response[0] != 0x80) {
    return false;
  }

  // 状态位：0x11表示锁关闭，0x00表示锁打开
  is_locked = (response[3] == 0x11);
  return true;
}

bool LockController::parseQueryAllResponse(const std::vector<uint8_t>& response, std::vector<bool>& lock_states)
{
  if (response.size() < 6) {
    return false;
  }

  if (response[0] != 0x80) {
    return false;
  }

  lock_states.clear();

  uint8_t state1 = response[2]; // 锁17-24
  uint8_t state2 = response[3]; // 锁9-16
  uint8_t state3 = response[4]; // 锁1-8

  // 解析state3 (锁1-8)
  for (int i = 0; i < 8; i++) {
    lock_states.push_back((state3 & (1 << i)) != 0);
  }

  // 解析state2 (锁9-16)
  for (int i = 0; i < 8; i++) {
    lock_states.push_back((state2 & (1 << i)) != 0);
  }

  // 解析state1 (锁17-24)
  for (int i = 0; i < 8; i++) {
    lock_states.push_back((state1 & (1 << i)) != 0);
  }

  return true;
}

void LockController::unlockCallback(
  const std::shared_ptr<srv::UnlockCommand::Request> request,
  std::shared_ptr<srv::UnlockCommand::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Unlock request: board=%d, lock=%d",
              request->board_address, request->lock_address);

  auto cmd = buildUnlockCommand(request->board_address, request->lock_address);
  std::vector<uint8_t> resp;

  if (sendCommand(cmd, resp)) {
    bool is_locked;
    if (parseUnlockResponse(resp, is_locked)) {
      response->success = true;
      response->message = is_locked ? "Lock still locked (may be feedback type)" : "Lock opened successfully";

      // 发布状态
      auto status_msg = msg::LockStatus();
      status_msg.board_address = request->board_address;
      status_msg.lock_address = request->lock_address;
      status_msg.is_locked = is_locked;
      lock_status_pub_->publish(status_msg);

      RCLCPP_INFO(this->get_logger(), "Unlock result: %s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "Failed to parse response";
      RCLCPP_ERROR(this->get_logger(), "Failed to parse unlock response");
    }
  } else {
    response->success = false;
    response->message = "Failed to send command or receive response";
    RCLCPP_ERROR(this->get_logger(), "Failed to send unlock command");
  }
}

void LockController::queryStatusCallback(
  const std::shared_ptr<srv::QueryLockStatus::Request> request,
  std::shared_ptr<srv::QueryLockStatus::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Query status request: board=%d, lock=%d",
              request->board_address, request->lock_address);

  if (request->lock_address == 0) {
    // 查询所有锁状态
    auto cmd = buildQueryAllCommand(request->board_address);
    std::vector<uint8_t> resp;

    if (sendCommand(cmd, resp)) {
      std::vector<bool> lock_states;
      if (parseQueryAllResponse(resp, lock_states)) {
        response->success = true;
        response->all_lock_states = lock_states;
        response->message = "Query all locks successful";

        // 发布所有锁状态
        auto status_msg = msg::AllLockStatus();
        status_msg.board_address = request->board_address;
        status_msg.lock_states = lock_states;
        all_lock_status_pub_->publish(status_msg);

        RCLCPP_INFO(this->get_logger(), "Queried %zu locks", lock_states.size());
      } else {
        response->success = false;
        response->message = "Failed to parse response";
      }
    } else {
      response->success = false;
      response->message = "Failed to send command or receive response";
    }
  } else {
    // 查询单个锁状态
    auto cmd = buildQueryCommand(request->board_address, request->lock_address);
    std::vector<uint8_t> resp;

    if (sendCommand(cmd, resp)) {
      bool is_locked;
      if (parseQueryResponse(resp, is_locked)) {
        response->success = true;
        response->is_locked = is_locked;
        response->message = is_locked ? "Lock is closed" : "Lock is open";

        // 发布状态
        auto status_msg = msg::LockStatus();
        status_msg.board_address = request->board_address;
        status_msg.lock_address = request->lock_address;
        status_msg.is_locked = is_locked;
        lock_status_pub_->publish(status_msg);

        RCLCPP_INFO(this->get_logger(), "Query result: %s", response->message.c_str());
      } else {
        response->success = false;
        response->message = "Failed to parse response";
      }
    } else {
      response->success = false;
      response->message = "Failed to send command or receive response";
    }
  }
}

void LockController::unlockMultipleCallback(
  const std::shared_ptr<srv::UnlockMultiple::Request> request,
  std::shared_ptr<srv::UnlockMultiple::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Unlock multiple request: board=%d, mask_size=%zu",
              request->board_address, request->lock_mask.size());

  auto cmd = buildUnlockMultipleCommand(request->board_address, request->lock_mask);
  std::vector<uint8_t> resp;

  if (sendCommand(cmd, resp, 300)) {
    response->success = true;
    response->message = "Multiple unlock command sent successfully";
    RCLCPP_INFO(this->get_logger(), "Multiple unlock successful");
  } else {
    response->success = false;
    response->message = "Failed to send command or receive response";
    RCLCPP_ERROR(this->get_logger(), "Failed to send multiple unlock command");
  }
}

void LockController::longPowerControlCallback(
  const std::shared_ptr<srv::LongPowerControl::Request> request,
  std::shared_ptr<srv::LongPowerControl::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Long power control request: board=%d, lock=%d, enable=%d",
              request->board_address, request->lock_address, request->enable);

  auto cmd = buildLongPowerCommand(request->board_address, request->lock_address, request->enable);
  std::vector<uint8_t> resp;

  if (sendCommand(cmd, resp)) {
    response->success = true;
    response->message = request->enable ? "Long power enabled" : "Long power disabled";
    RCLCPP_INFO(this->get_logger(), "Long power control successful");
  } else {
    response->success = false;
    response->message = "Failed to send command or receive response";
    RCLCPP_ERROR(this->get_logger(), "Failed to send long power command");
  }
}

void LockController::feedbackTimerCallback()
{
  // 检查是否有主动反馈数据
  uint8_t buffer[256];
  ssize_t n = read(serial_fd_, buffer, sizeof(buffer));

  if (n >= 5) {
    std::vector<uint8_t> data(buffer, buffer + n);

    // 检查是否是主动反馈 (82开头)
    if (data[0] == 0x82) {
      uint8_t board_addr = data[1];
      uint8_t lock_addr = data[2];
      bool is_locked = (data[3] == 0x11);

      // 发布锁状态变化
      auto status_msg = msg::LockStatus();
      status_msg.board_address = board_addr;
      status_msg.lock_address = lock_addr;
      status_msg.is_locked = is_locked;
      lock_status_pub_->publish(status_msg);

      RCLCPP_INFO(this->get_logger(), "Active feedback: board=%d, lock=%d, locked=%d",
                  board_addr, lock_addr, is_locked);
    }
  }
}

} // namespace lock_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lock_controller::LockController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
