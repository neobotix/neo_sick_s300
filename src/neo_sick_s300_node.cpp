/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../include/SickS300Receiver.h"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


class SickS300ReceiverROS : public SickS300Receiver
{
public:
	double scan_cycle_time = 0;
	double scan_duration = 0;
	double scan_delay = 0;
	double angle_min = 0;
	double angle_max = 0;
	double range_min = 0;
	double range_max = 0;
	std::string frame_id;

	SickS300ReceiverROS()
	{
		nh.param("scan_cycle_time", scan_cycle_time, 0.040);
		nh.param("scan_duration", scan_duration, 0.025);	// scan_cycle_time * 270/360
		nh.param("scan_delay", scan_delay, 0.030);			// 20 ms transmission + 10 ms processing
		nh.param("angle_min", angle_min, -135.0/180.0 * M_PI);
		nh.param("angle_max", angle_max, 135.0/180.0 * M_PI);
		nh.param("range_min", range_min, 0.01);
		nh.param("range_max", range_max, 25.0);
		nh.param("frame_id", frame_id, std::string("/base_laser_link"));

		m_topic_scan = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
	}

protected:
	void handle_scan(uint32_t scan_id, const std::vector<point_t>& points) override
	{
		auto msg = boost::make_shared<sensor_msgs::LaserScan>();

		msg->header.stamp = ros::Time::now() - ros::Duration(scan_duration) - ros::Duration(scan_delay);
		msg->header.frame_id = frame_id;

		const size_t num_points = points.size();

		msg->angle_min = angle_min;
		msg->angle_max = angle_max;
		msg->range_min = range_min;
		msg->range_max = range_max;
		msg->angle_increment = (angle_max - angle_min) / (num_points - 1);
		msg->time_increment = scan_duration / (num_points - 1);
		msg->scan_time = scan_cycle_time;

		msg->ranges.resize(num_points);
		msg->intensities.resize(num_points);
		for(size_t i = 0; i < num_points; ++i)
		{
			msg->ranges[i] = points[i].distance;
			msg->intensities[i] = points[i].reflector ? 1.f : 0.f;
		}

		m_topic_scan.publish(msg);
	}

	void handle_debug_msg(const std::string& msg) override
	{
		ROS_INFO("%s", msg.c_str());
	}

private:
	ros::NodeHandle nh;
	ros::Publisher m_topic_scan;

};


//#######################
//#### main programm ####
int main(int argc, char **argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_sick_s300_node");

	// keep a node handle outside the loop to prevent auto-shutdown
	ros::NodeHandle nh;

	std::string port;
	int baud_rate = 0;

	nh.param("port", port, std::string("/dev/ttyUSB0"));
	nh.param("baud", baud_rate, 500000);

	while(ros::ok())
	{
		SickS300ReceiverROS receiver;

		try {
			receiver.open(port, baud_rate);
		}
		catch(std::exception& ex)
		{
			ROS_ERROR("Failed to open S300 port '%s': %s",  port.c_str(), ex.what());
			ros::WallDuration(1).sleep();
			continue;
		}

		ROS_INFO("Opened S300 port %s", port.c_str());

		while(ros::ok())
		{
			try {
				receiver.read(1000);
			}
			catch(std::exception& ex)
			{
				if(ros::ok())
				{
					ROS_ERROR("S300 port '%s' read error: %s", port.c_str(), ex.what());
					ros::WallDuration(1).sleep();
				}
				break;
			}
		}
	}

	return 0;
}
