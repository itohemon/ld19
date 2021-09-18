#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tofbf.h"
#include <string>

#define RADIAN_TO_ANGLED(angle) ((angle)*180000/3141.59)

int main(int argc , char **argv)
{
	std::string port_name;
	std::string lidar_topic;

	ros::init(argc, argv, "product");
	ros::NodeHandle nh;                    /* create a ROS Node */
	ros::NodeHandle pnh("~");
	pnh.getParam("usb_port", port_name);
	pnh.getParam("lidar_topic", lidar_topic);

	ROS_INFO("[%s] param: %s",
		ros::this_node::getName().c_str(), port_name.c_str());
	ROS_INFO("[%s] param: %s",
		ros::this_node::getName().c_str(), lidar_topic.c_str());
 	
	LiPkg * lidar = new LiPkg;
  
    CmdInterfaceLinux cmd_port;

	if(port_name.empty())
	{
		std::cout<<"Can't find LiDAR LD19"<< std::endl;
	}

	std::cout<<"FOUND LiDAR_LD19"  <<std::endl;
	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		if(lidar->Parse((uint8_t*)byte, len))
		{
			lidar->AssemblePacket();  
		}
	});

	if(cmd_port.Open(port_name))
		std::cout<<"LiDAR_LD19 started successfully "  <<std::endl;
	
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(lidar_topic.c_str(), 1); /*create a ROS topic */
	
	while (ros::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub.publish(lidar->GetLaserScan());  // Fixed Frame:  lidar_frame
			lidar->ResetFrameReady();
#if 0
			sensor_msgs::LaserScan data = lidar->GetLaserScan();
			unsigned int lens = (data.angle_max - data.angle_min) / data.angle_increment;  
			std::cout << "current_speed: " << lidar->GetSpeed() << " " 
			          << "len: " << lens << " "
					  << "angle_min: " << RADIAN_TO_ANGLED(data.angle_min) << " "
					  << "angle_max: " << RADIAN_TO_ANGLED(data.angle_max) << std::endl; 
			std::cout << "----------------------------" << std::endl;
			for (int i = 0; i < lens; i++)
			{
				std::cout << "range: " <<  data.ranges[i] << " " 
						  << "intensites: " <<  data.intensities[i] << std::endl;
			}
			std::cout << "----------------------------" << std::endl;
#endif
		}
	}
    return 0;
}

