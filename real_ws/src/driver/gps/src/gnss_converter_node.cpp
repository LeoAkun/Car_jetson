#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

class GNSSConverterNode : public rclcpp::Node
{
public:
    GNSSConverterNode() : Node("gnss_converter_node")
    {
        // 声明参数
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("frame_id", "gnss_link");

        // 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // 创建发布器 - 发布标准 GPS 消息
        navsat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/pose_with_covariance", 10);

        // 打开串口
        try {
            serial_ = std::make_shared<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(100));

            if (serial_->isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", port.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
                return;
            }
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port %s: %s", port.c_str(), e.what());
            return;
        }

        // 初始化时间戳
        last_log_time_ = this->now();

        // 创建定时器读取串口数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GNSSConverterNode::readSerialData, this));

        RCLCPP_INFO(this->get_logger(), "GNSS Converter Node started");
    }

    ~GNSSConverterNode()
    {
        if (serial_ && serial_->isOpen()) {
            serial_->close();
        }
    }

private:
    void readSerialData()
    {
        if (!serial_ || !serial_->isOpen()) {
            return;
        }

        try {
            // 读取一行数据
            if (serial_->available()) {
                std::string line = serial_->readline(65536, "\n");

                if (!line.empty()) {
                    processNMEA(line);
                }
            }
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
        }
    }

    void processNMEA(const std::string &line)
    {
        // 去除换行符和回车符
        std::string clean_line = line;
        clean_line.erase(std::remove(clean_line.begin(), clean_line.end(), '\r'), clean_line.end());
        clean_line.erase(std::remove(clean_line.begin(), clean_line.end(), '\n'), clean_line.end());

        // 检查是否为 GNGGA 或 GPGGA 消息
        if (clean_line.find("$GNGGA") == 0 || clean_line.find("$GPGGA") == 0) {
            parseGGA(clean_line);
        }
    }

    void parseGGA(const std::string &line)
    {
        std::vector<std::string> tokens = split(line, ',');

        // GNGGA 格式：$GNGGA,时间,纬度,N/S,经度,E/W,质量,卫星数,HDOP,海拔,M,大地水准面高度,M,,*校验
        if (tokens.size() < 15) {
            return;
        }

        try {
            // 提取纬度
            std::string lat_str = tokens[2];
            std::string lat_dir = tokens[3];

            // 提取经度
            std::string lon_str = tokens[4];
            std::string lon_dir = tokens[5];

            // 提取定位质量
            int quality = tokens[6].empty() ? 0 : std::stoi(tokens[6]);

            // 提取卫星数
            int num_sats = tokens[7].empty() ? 0 : std::stoi(tokens[7]);

            // 提取 HDOP
            double hdop = tokens[8].empty() ? 99.99 : std::stod(tokens[8]);

            // 提取海拔高度
            double altitude = tokens[9].empty() ? 0.0 : std::stod(tokens[9]);

            // 检查定位质量（0=无效，1=GPS，2=DGPS，4=RTK固定解，5=RTK浮点解）
            if (quality == 0 || lat_str.empty() || lon_str.empty()) {
                return;
            }

            // 转换纬度（DDMM.MMMMM -> DD.DDDDDD）
            double latitude = parseLatLon(lat_str);
            if (lat_dir == "S") latitude = -latitude;

            // 转换经度（DDDMM.MMMMM -> DDD.DDDDDD）
            double longitude = parseLatLon(lon_str);
            if (lon_dir == "W") longitude = -longitude;

            // 创建并发布 NavSatFix 消息
            auto msg = sensor_msgs::msg::NavSatFix();

            // 设置消息头
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id_;

            // 设置定位状态
            msg.status.status = mapQualityToStatus(quality);
            msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                                sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                                sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
                                sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;

            // 设置位置信息（十进制度数）
            msg.latitude = latitude;
            msg.longitude = longitude;
            msg.altitude = altitude;

            // 设置协方差（基于 HDOP）
            // position_covariance 是 3x3 矩阵，按行展开成 9 个元素
            double position_variance = std::pow(hdop * 2.0, 2);
            msg.position_covariance[0] = position_variance;  // East variance
            msg.position_covariance[4] = position_variance;  // North variance
            msg.position_covariance[8] = position_variance * 2.0;  // Up variance (通常垂直精度较差)

            // 设置协方差类型
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

            // 发布消息
            navsat_pub_->publish(msg);

            // 打印调试信息（每秒一次）
            auto current_time = this->now();
            if ((current_time - last_log_time_).seconds() >= 1.0) {
                std::string status_str = getStatusString(quality);
                RCLCPP_INFO(this->get_logger(),
                           "GPS - Lat: %.8f°, Lon: %.8f°, Alt: %.2fm | Status: %s | Sats: %d, HDOP: %.2f",
                           latitude, longitude, altitude, status_str.c_str(), num_sats, hdop);
                last_log_time_ = current_time;
            }

        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing GGA: %s", e.what());
        }
    }

    double parseLatLon(const std::string &str)
    {
        // 输入格式：DDMM.MMMMM 或 DDDMM.MMMMM
        if (str.empty()) return 0.0;

        double value = std::stod(str);

        // 提取度数部分
        int degrees = static_cast<int>(value / 100.0);

        // 提取分钟部分
        double minutes = value - (degrees * 100.0);

        // 转换为十进制度数
        return degrees + (minutes / 60.0);
    }

    int8_t mapQualityToStatus(int quality)
    {
        // 将 NMEA 质量码映射到 NavSatStatus
        switch (quality) {
            case 0:
                return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            case 1:
                return sensor_msgs::msg::NavSatStatus::STATUS_FIX;  // GPS fix
            case 2:
                return sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;  // DGPS fix
            case 4:
            case 5:
                return sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;  // RTK fix
            default:
                return sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
    }

    std::string getStatusString(int quality)
    {
        switch (quality) {
            case 0: return "No Fix";
            case 1: return "GPS Fix";
            case 2: return "DGPS Fix";
            case 4: return "RTK Fixed";
            case 5: return "RTK Float";
            default: return "Unknown";
        }
    }

    std::vector<std::string> split(const std::string &str, char delimiter)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;

        while (std::getline(ss, token, delimiter)) {
            tokens.push_back(token);
        }

        return tokens;
    }

    // 成员变量
    std::shared_ptr<serial::Serial> serial_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    rclcpp::Time last_log_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
