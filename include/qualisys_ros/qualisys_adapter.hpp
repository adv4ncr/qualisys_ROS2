/**
 * 
 * 
*/

// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
// https://github.com/ros2/demos/blob/humble/lifecycle/README.rst

// QTM video: https://www.qualisys.com/video-tutorials/how-to-use-the-c-sdk-for-qtm-2/
// QTM git: https://github.com/qualisys/qualisys_cpp_sdk
// QTM git rigidBodyStreaming: https://github.com/qualisys/qualisys_cpp_sdk/blob/master/RigidBodyStreaming/RigidBodyStreaming.cpp

// TF2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
// lifecycle nodes api: https://docs.ros2.org/latest/api/rclcpp_lifecycle/namespacerclcpp__lifecycle.html
// lifecycle nodes states: https://design.ros2.org/articles/node_lifecycle.html



#ifndef QUALISYS_ROS_HPP
#define QUALISYS_ROS_HPP

#include <string>
#include <chrono>
#include <thread>
#include <sys/mman.h>
#include <sched.h>
#include <fstream>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
//#include <lifecycle_msgs/lifecycle_msgs/msg/state.hpp>
#include "lifecycle_msgs/msg/state.hpp"
//#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>


#include "RTProtocol.h"
#include "RTPacket.h"

using namespace std::chrono_literals;
using namespace rclcpp_lifecycle::node_interfaces;

namespace qualisys_ros
{

class QualisysAdapter : public rclcpp_lifecycle::LifecycleNode
{

public:
		explicit QualisysAdapter(const std::string & node_name, bool intra_process_comms = false)
  			: rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
		{
			// read parameter
			qtm_ip_address_ = this->declare_parameter<std::string>("qtm_ip_address", "149.201.162.66");
			qtm_base_port_ = this->declare_parameter<u_short>("qtm_server_base_port", 22222);
			qtm_udp_port_ = this->declare_parameter<u_short>("qtm_server_udp_port", 6734);
			qtm_password_ = this->declare_parameter<std::string>("qtm_password", "password");
			qtm_data_available_ = false;
			qtm_stream_frames_ = false;
			
			RCLCPP_INFO(this->get_logger(), "[QTM] IP: %s PORT: %d", qtm_ip_address_.c_str(), qtm_base_port_);

			// initialize transform broadcaster
			tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		}

		LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
		LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
		LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
		LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
		LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);


		// adapter_thread functions
		void thread_function();
		bool configure_sched_fifo(int priority);
		bool has_realtime_kernel();
		bool thread_run_;


private:

	// publisher
	//std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

	// read package function
	void process_package();

	// TF broadcaster
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	geometry_msgs::msg::TransformStamped t_;

	// Qualisys Variables
	CRTProtocol rtProtocol_;
	CRTPacket::EPacketType packetType_;
	CRTPacket* rtPacket_;

	std::string qtm_ip_address_, qtm_password_;
	u_short qtm_base_port_, qtm_udp_port_;
	bool qtm_data_available_;
	bool qtm_stream_frames_;
	unsigned int qtm_frame_i;
	rclcpp::Time qtm_arrival_time_;
	float qtm_t_[3];
    float qtm_R_[9];

	// transformation
	tf2::Quaternion q_;
	tf2::Matrix3x3 R_;

	// utils
	rclcpp::Clock clock;

	// rt thread
	std::unique_ptr<std::thread> adapter_thread_ptr;

	// testing (reading on file)
	const bool read_measurement_file_ = false;

};

} // namespace qualisys_ros

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exe;

	std::shared_ptr<qualisys_ros::QualisysAdapter> lc_node = 
		std::make_shared<qualisys_ros::QualisysAdapter>("qualisys_adapter");

  	exe.add_node(lc_node->get_node_base_interface());
  	exe.spin();

	rclcpp::shutdown();
	return 0;
}


#endif // QUALISYS_ROS_HPP