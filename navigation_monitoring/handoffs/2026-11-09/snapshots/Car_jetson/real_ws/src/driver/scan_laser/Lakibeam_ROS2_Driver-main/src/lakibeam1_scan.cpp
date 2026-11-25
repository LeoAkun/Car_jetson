#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include "../include/data_type.h"
#include "../include/remote.h"

#define DEG2RAD(x) ((x)*M_PI / 180.f)
using namespace std;
class lakibeam1_scan : public rclcpp::Node
{
public:
	lakibeam1_scan():Node("laser_scan_publisher")
	{
		declare_parameters();
		get_parameters();
		scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, 1000);
		info();
		// scan_config();
		if (create_socket() != 0)
		{
			throw std::runtime_error("Unable to bind the LiDAR UDP socket");
		}
		scan_publish();
	}
	~lakibeam1_scan() override
	{
		if (sockfd >= 0)
		{
			close(sockfd);
		}
	}
protected:
	void get_parameters()
	{
		get_parameter<string>("frame_id",frame_id);
		get_parameter<std::string>("port",port);
		get_parameter<string>("hostip",hostip);
		get_parameter<string>("sensorip",sensorip);
		get_parameter<string>("output_topic",output_topic);
		get_parameter<string>("scanfreq",scanfreq);
		get_parameter<string>("filter",filter);
		get_parameter<string>("laser_enable",laser_enable);
		get_parameter<string>("scan_range_start",scan_range_start);
		get_parameter<string>("scan_range_stop",scan_range_stop);
		get_parameter<bool>("inverted",inverted);
		get_parameter<int>("angle_offset",angle_offset);
	};

	void declare_parameters()
	{
		declare_parameter<string>("frame_id",frame_id);
		declare_parameter<string>("port",port);
		declare_parameter<string>("hostip",hostip);
		declare_parameter<string>("sensorip",sensorip);
		declare_parameter<string>("output_topic",output_topic);
		declare_parameter<string>("scanfreq",scanfreq);
		declare_parameter<string>("filter",filter);
		declare_parameter<string>("laser_enable",laser_enable);
		declare_parameter<string>("scan_range_start",scan_range_start);
		declare_parameter<string>("scan_range_stop",scan_range_stop);
		declare_parameter<bool>("inverted",inverted);
		declare_parameter<int>("angle_offset",angle_offset);
	};
	void info()
	{
		RCLCPP_INFO(get_logger(),"frame_id:%s", frame_id.c_str());
		RCLCPP_INFO(get_logger(),"output_topic:%s", output_topic.c_str());
		RCLCPP_INFO(get_logger(),"inverted:%s", (inverted ? "True" : "False"));
		RCLCPP_INFO(get_logger(),"hostip:%s", hostip.c_str());
		RCLCPP_INFO(get_logger(),"sensorip:%s", sensorip.c_str());
		RCLCPP_INFO(get_logger(),"port:%s", port.c_str());
		RCLCPP_INFO(get_logger(),"scanfreq:%s", scanfreq.c_str());
		RCLCPP_INFO(get_logger(),"filter:%s", filter.c_str());
		RCLCPP_INFO(get_logger(),"laser_enable:%s", laser_enable.c_str());
		RCLCPP_INFO(get_logger(),"scan_range_start:%s", scan_range_start.c_str());
		RCLCPP_INFO(get_logger(),"scan_range_stop:%s", scan_range_stop.c_str());

	};
	void scan_config()
	{
		RCLCPP_INFO(get_logger(),"scan_config");
		sensor_config(sensorip, "/api/v1/sensor/scanfreq", scanfreq);
		sensor_config(sensorip, "/api/v1/sensor/laser_enable", laser_enable);
		sensor_config(sensorip, "/api/v1/sensor/scan_range/start", scan_range_start);
		sensor_config(sensorip, "/api/v1/sensor/scan_range/stop", scan_range_stop);		
		RCLCPP_INFO(get_logger(),"scan_config1");

	};
	int create_socket()
    {
		RCLCPP_INFO(get_logger(),"create_socket");
		// rclcpp::sleep_for(std::chrono::milliseconds(2000));
		// get_telemetry_data(sensorip);
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockfd == -1)
        {
			RCLCPP_ERROR(get_logger(),"Failed to create socket: %s", std::strerror(errno));
            return -1;
        }

		timeval timeout{};
		timeout.tv_sec = 0;
		timeout.tv_usec = 200000;
		if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
		{
			RCLCPP_ERROR(get_logger(),"Unable to set UDP receive timeout: %s", std::strerror(errno));
			close(sockfd);
			sockfd = -1;
			return -1;
		}

        memset(&ser_addr, 0, sizeof(ser_addr));
        ser_addr.sin_family = AF_INET;
        ser_addr.sin_addr.s_addr = inet_addr(hostip.c_str());
        ser_addr.sin_port = htons(atoi(port.c_str()));

        if(bind(sockfd, (struct sockaddr*)&ser_addr, sizeof(ser_addr)) < 0)
        {
			RCLCPP_ERROR(get_logger(),"Socket bind error: %s", std::strerror(errno));
			close(sockfd);
			sockfd = -1;
            return -1;
        }
        return 0;
    };
	void scan_publish()
	{
		double inf = std::numeric_limits<double>::infinity();
		RCLCPP_INFO(get_logger(),"scan_publish");
		// rclcpp::sleep_for(std::chrono::milliseconds(2000));
		// get_telemetry_data(sensorip);
		while (rclcpp::ok())
		{
			if(scan_vec_ready == 0)
			{
				while(rclcpp::ok())
				{
					if(j == 12)
					{
						unsigned int len = sizeof(clent_addr);
						const ssize_t received = recvfrom(
							sockfd, &MSOP_Data, sizeof(MSOP_Data), 0,
							(struct sockaddr*)&clent_addr, &len);
						if (received < 0)
						{
							if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
							{
								continue;
							}
							RCLCPP_ERROR(get_logger(), "UDP receive failed: %s", std::strerror(errno));
							return;
						}
						if (received != static_cast<ssize_t>(sizeof(MSOP_Data)))
						{
							RCLCPP_WARN(get_logger(), "Ignoring short LiDAR datagram: %zd bytes", received);
							continue;
						}
						if(MSOP_Data.BlockID[0].Azimuth == 0)
						{
							scan_end = scan_begin;
							scan_begin = rclcpp::Clock().now();
						}			
						if((MSOP_Data.BlockID[1].Azimuth - MSOP_Data.BlockID[0].Azimuth) > 0)
						{
							resolution = (MSOP_Data.BlockID[1].Azimuth - MSOP_Data.BlockID[0].Azimuth) / 16;
						}
						j = 0;
					}

					for(;j < 12; j++)
					{
						for(i = 0; i < 16; i++)
						{
							bm_response_scan_t response_ptr;
							response_ptr.angle = (MSOP_Data.BlockID[j].Azimuth + (resolution * i));
							if(MSOP_Data.BlockID[j].DataFlag == 0xEEFF)
							{
								if(response_ptr.angle == 0)
								{
									if(!scan_vec.empty() & (scan_vec_ready == 0))
									{
										scan_vec_ready = 1;
										if(scan_vec.size() < 1200)
										{
											j = 12;
										}
										break;
									}
								}
								response_ptr.dist = MSOP_Data.BlockID[j].Result[i].Dist_1;
								response_ptr.rssi = MSOP_Data.BlockID[j].Result[i].RSSI_1;
								scan_vec.push_back(response_ptr);
							}
						}
						if(scan_vec_ready == 1)
						{
							break;
						}
					}
					if(scan_vec_ready == 1)
					{
						break;
					}
				}
			}
            

			if(scan_vec_ready == 1)
			{
				sensor_msgs::msg::LaserScan scan;
				uint16_t num_readings;
				float duration = (scan_begin - scan_end).seconds();

				num_readings = scan_vec.size();
				scan.header.stamp = this->now();  // 用发布时刻而非扫描开始时刻，避免UDP积压导致时间戳过旧
				scan.header.frame_id = frame_id;
				scan.angle_min = DEG2RAD(-180 + angle_offset);
				scan.angle_max = DEG2RAD(180 + angle_offset);
				scan.angle_increment = 2.0 * M_PI / num_readings;
				scan.scan_time = duration;
				scan.time_increment = duration/(float)num_readings/2;
				scan.range_min = 0.0;
				scan.range_max = 100.0;
				scan.ranges.resize(num_readings);
				scan.intensities.resize(num_readings);

				for(int i = 0;i < num_readings; i++)
				{
					if (!inverted)
					{
						scan.ranges[i] = (float)scan_vec[i].dist / 1000;
						scan.intensities[i] = scan_vec[i].rssi;
						if(scan.ranges[i] == 0)
						{
							scan.ranges[i] = inf;
							scan.intensities[i] = 0;
						}
					}
					else
					{
						scan.ranges[num_readings - i - 1] = (float)scan_vec[i].dist / 1000;
						scan.intensities[num_readings - i - 1] = scan_vec[i].rssi;
						if(scan.ranges[num_readings - i - 1] == 0)
						{
							scan.ranges[num_readings - i - 1] = inf;
							scan.intensities[num_readings - i - 1] = 0;
						}
					}
				}

				scan_pub->publish(scan);
				// RCLCPP_INFO(get_logger(), "New topic %s published, total data points: %d", output_topic.c_str(), num_readings);
				scan_vec.clear();
				scan_vec_ready = 0;
			}
		}
		close(sockfd);
		sockfd = -1;
	}

private:
    string hostip, sensorip, port, frame_id, output_topic,scanfreq,filter,laser_enable,scan_range_start,scan_range_stop;
    int resolution=25, scan_vec_ready=0, angle_offset;
    bool inverted;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Time scan_begin, scan_end;
    struct sockaddr_in ser_addr, clent_addr; 
	int i = 0, j = 12, points_cnt;
	int sockfd = -1;
	unsigned int last_timestamp_;
    std::vector <bm_response_scan_t> scan_vec;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		// The node's receive loop runs in its constructor and owns the socket;
		// there are no executor callbacks to spin.
		auto node = make_shared<lakibeam1_scan>();
		node.reset();
	}
	catch (const std::exception& error)
	{
		RCLCPP_ERROR(rclcpp::get_logger("lakibeam1_scan"), "%s", error.what());
		rclcpp::shutdown();
		return 1;
	}
	if (rclcpp::ok())
	{
		rclcpp::shutdown();
	}
	return 0;
}
