#include <sys/timeb.h>
#include <thread>
#include <iostream>
#include <fstream>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

#include<cstreamer/cstreamer.h>
using namespace csapi;
CStreamer* p_cstreamer;


//#define  DEBUG_DUMP 
int dump_index = 0;

bool  first = false;

class Cs20 : public rclcpp::Node
{
  public:
    Cs20()
    : Node("cs_20"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      34ms, std::bind(&Cs20::timer_callback, this));
    }

  private:
    void timer_callback(){
   	    auto message = std_msgs::msg::String();
    	message.data = "Hello, world! " + std::to_string(count_++);
      	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      	publisher_->publish(message);

		uint16_t* depth_data;
		int depth_width = 0;
		int depth_height = 0;
		CSAPI_ERROR err = p_cstreamer->GetDepth(&depth_data, depth_width, depth_height);


		if (err == CSAPI_ERROR::SUCCESS)
		{
			std::string pre =  std::to_string(count_) + "-" + std::to_string(depth_width) + "x" + std::to_string(depth_height);
	
			delete[]depth_data;

		}

    }




    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char** argv)
{

	p_cstreamer = CStreamer::CreateInstance();

	/*double fx = 556.4338691598663;
	double fy = 559.1483291542506;
	double cx = 326.0565296284727;
	double cy = 245.587547558196;*/

	//获取设备类型
	std::vector<DeviceTypeInfo> vec_device_type;
	if (p_cstreamer->GetDeviceType(vec_device_type) != CSAPI_ERROR::SUCCESS) {
		return -1;
	}
	DeviceTypeInfo device_type = vec_device_type.at(0);
	//开启选择设备
	if (p_cstreamer->OpenDevice(device_type) != CSAPI_ERROR::SUCCESS) {
		return -1;
	}
	//获取支持分辨率类型
	std::vector<Format> vec_format;
	if (p_cstreamer->GetSupportFormat(vec_format, StreamType::CS_TOF) != CSAPI_ERROR::SUCCESS) {
		return -1;
	}
	//开始读取流
	Format format = vec_format.at(0);
	if (p_cstreamer->StartStream(format, StreamType::CS_TOF) != CSAPI_ERROR::SUCCESS) {
		return -1;
	}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Cs20>());
  	rclcpp::shutdown();

	p_cstreamer->StopStream();

	return 0;
}
