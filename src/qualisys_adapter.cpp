#include "qualisys_ros/qualisys_adapter.hpp"

namespace qualisys_ros
{

LifecycleNodeInterface::CallbackReturn QualisysAdapter::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
{
	//RCLCPP_DEBUG_STREAM(this->get_logger(), "main_thread_id: " << std::this_thread::get_id());

	//return LifecycleNodeInterface::CallbackReturn::SUCCESS;

	// connect to QTM
	if(!rtProtocol_.Connected() && !rtProtocol_.Connect(qtm_ip_address_.c_str(), qtm_base_port_, &qtm_udp_port_))
	{
		RCLCPP_ERROR(this->get_logger(), "[QTM] not connected: %s", rtProtocol_.GetErrorString());
		return LifecycleNodeInterface::CallbackReturn::FAILURE;
	}

	// // take control over QTM
	// if(!rtProtocol_.TakeControl(qtm_password_.c_str()))
	// {
	// 	RCLCPP_ERROR(this->get_logger(), "[QTM] no control: %s", rtProtocol_.GetErrorString());
	// 	return LifecycleNodeInterface::CallbackReturn::FAILURE;
	// }

	// if(read_measurement_file_)
	// {
	// 	// start file
	// 	if(!rtProtocol_.StartRTOnFile())
	// 	{
	// 		RCLCPP_ERROR(this->get_logger(), "[QTM] file not started: %s", rtProtocol_.GetErrorString());
	// 		return LifecycleNodeInterface::CallbackReturn::FAILURE;
	// 	}
	// }
	// else if(!rtProtocol_.StartCapture())
	// {
	// 	RCLCPP_ERROR(this->get_logger(), "[QTM] cannot start capture: %s", rtProtocol_.GetErrorString());
	// 	return LifecycleNodeInterface::CallbackReturn::FAILURE;	
	// }

	// get 6DOF rigid body settings from QTM
	if(!qtm_data_available_ && !rtProtocol_.Read6DOFSettings(qtm_data_available_))
	{
		RCLCPP_ERROR(this->get_logger(), "[QTM] 6DOF data not read: %s", rtProtocol_.GetErrorString());
		return LifecycleNodeInterface::CallbackReturn::FAILURE;
	}


	if(!qtm_stream_frames_)
	{
		if (!rtProtocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, /*0*/ qtm_udp_port_, NULL, CRTProtocol::cComponent6d))
		{
			RCLCPP_ERROR(this->get_logger(), "[QTM] data not streamed: %s", rtProtocol_.GetErrorString());
			return LifecycleNodeInterface::CallbackReturn::FAILURE;
		}
		qtm_stream_frames_ = true;
		RCLCPP_INFO(this->get_logger(), "[QTM] Starting to streaming 6DOF data: %s", rtProtocol_.GetErrorString());
	}


	return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/// Transition callback for state activating
/**
 * on_activate callback is being called when the lifecycle node
 * enters the "activating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "active" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
LifecycleNodeInterface::CallbackReturn QualisysAdapter::on_activate(const rclcpp_lifecycle::State & previous_state)
{
	// The parent class method automatically transition on managed entities
	// (currently, LifecyclePublisher).
	// pub_->on_activate() could also be called manually here.
	// Overriding this method is optional, a lot of times the default is enough.
	LifecycleNode::on_activate(previous_state);
	RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

	adapter_thread_ptr = std::unique_ptr<std::thread>(
		new std::thread(
			[this]()
			{
				this->thread_run_ = true;
				int const kSchedPriority = 50;

				if (this->has_realtime_kernel())
				{
					// set scheduler
					if (!this->configure_sched_fifo(kSchedPriority))
						RCLCPP_WARN(this->get_logger(), "Could not enable FIFO RT scheduling policy");
					// Lock memory
					if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
						RCLCPP_WARN(this->get_logger(), "mlockall failed");
					// #TODO integrate rttest to measure pagefaults
        		}
 
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "RT kernel is recommended for better performance");
				}

				//RCLCPP_DEBUG_STREAM(this->get_logger(), "adapter_thread_id: " << std::this_thread::get_id());

				while(rclcpp::ok() && this->thread_run_)
				{
					this->thread_function();
				}
			}
		)
	);

	return LifecycleNodeInterface::CallbackReturn::SUCCESS;

}


/// Transition callback for state deactivating
/**
 * on_deactivate callback is being called when the lifecycle node
 * enters the "deactivating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "active".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "active"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
LifecycleNodeInterface::CallbackReturn QualisysAdapter::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
	RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

	// stop thread
	this->thread_run_ = false;
	this->adapter_thread_ptr->join();

	// stop capturing
	rtProtocol_.StopCapture();

	return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup
/**
 * on_cleanup callback is being called when the lifecycle node
 * enters the "cleaningup" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "unconfigured" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
LifecycleNodeInterface::CallbackReturn QualisysAdapter::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
{
	RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
	return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state shutting down
/**
 * on_shutdown callback is being called when the lifecycle node
 * enters the "shuttingdown" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "finalized" state or stays
 * in its current state.
 * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
 * TRANSITION_CALLBACK_FAILURE transitions to current state
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
LifecycleNodeInterface::CallbackReturn QualisysAdapter::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
{
	RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");

	// stop thread if running
	if(thread_run_)
	{
		thread_run_ = false;
		adapter_thread_ptr->join();
	}

	// disconnect qtm
	rtProtocol_.StopCapture();
	rtProtocol_.Disconnect();

	return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/**
 * Main streaming function running in a seperate high priority thread
*/
void QualisysAdapter::thread_function()
{

	// switch (this->get_current_state().id())
	// {

	// case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
	// if()
		
		switch(rtProtocol_.Receive(packetType_, true, 1000000))
		{
			case CNetwork::ResponseType::success:
				// successfully received packet
				switch (packetType_)
				{
					// packet contains data
					case CRTPacket::PacketData:
						process_package();
						break;
					case CRTPacket::PacketError:
						RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clock, 1000, 
							"Error when streaming frames: " << rtProtocol_.GetRTPacket()->GetErrorString());
						break;
					case CRTPacket::PacketNoMoreData:
						RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "No more data");
						break;
					
					default:
						break;
				}
				break;
			case CNetwork::ResponseType::disconnect:
				RCLCPP_WARN(this->get_logger(), "RTProtocol disconnected");
				break;
			case CNetwork::ResponseType::error:
				RCLCPP_WARN(this->get_logger(), "RTProtocol error");
				break;
			case CNetwork::ResponseType::timeout:
				RCLCPP_WARN(this->get_logger(), "RTProtocol timeout");
				break;


		}
		//std::this_thread::sleep_for(5ms);
	// 	break;
	
	// default:
	// 	this->thread_run_ = false;
	// 	//std::this_thread::sleep_for(100ms);
	// 	break;
	// }
	
}

/**
 * Get and process QTM package and send as tf2
*/
void QualisysAdapter::process_package()
{
	rtPacket_ = rtProtocol_.GetRTPacket();
	// rtPacket_->GetTimeStamp() is in ms
	//qtm_arrival_time_ = rclcpp::Time(rtPacket_->GetTimeStamp()*1000);
	qtm_arrival_time_ = this->get_clock()->now(); // #TODO

	for(qtm_frame_i=0; qtm_frame_i<rtPacket_->Get6DOFBodyCount(); qtm_frame_i++)
	{
		
		if(rtPacket_->Get6DOFBody(qtm_frame_i, qtm_t_[0], qtm_t_[1], qtm_t_[2], qtm_R_))
		{
			t_.header.stamp = qtm_arrival_time_;
			t_.header.frame_id = "world";
			t_.child_frame_id = rtProtocol_.Get6DOFBodyName(qtm_frame_i);

			// convert rotation matrix to quaternion
			R_.setValue(
				qtm_R_[0], qtm_R_[3], qtm_R_[6],
				qtm_R_[1], qtm_R_[4], qtm_R_[7],
				qtm_R_[2], qtm_R_[5], qtm_R_[8]);
			R_.getRotation(q_);

			t_.transform.rotation.w = q_.w();
			t_.transform.rotation.x = q_.x();
			t_.transform.rotation.y = q_.y();
			t_.transform.rotation.z = q_.z();

			t_.transform.translation.x = qtm_t_[0] / 1000.;
			t_.transform.translation.y = qtm_t_[1] / 1000.;
			t_.transform.translation.z = qtm_t_[2] / 1000.;

			//for(auto n : qtm_t_) std::cout << "[QTM_DBG] qtm_t_: " << n << std::endl;
			tf_broadcaster_->sendTransform(t_);
		}
	}


}


// source: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/realtime.cpp
bool QualisysAdapter::has_realtime_kernel()
{
	std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
	bool has_realtime = false;
	if (realtime_file.is_open())
	{
		realtime_file >> has_realtime;
	}
	return has_realtime;
}

bool QualisysAdapter::configure_sched_fifo(int priority)
{
	struct sched_param schedp;
	memset(&schedp, 0, sizeof(schedp));
	schedp.sched_priority = priority;
	// initially: SCHED_FIFO #TODO what to use?
	return !sched_setscheduler(0, SCHED_DEADLINE, &schedp);
}

}