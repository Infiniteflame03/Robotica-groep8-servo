#include "PositionNode.h"
#include "MicrorosTranslation.h"
#include "rclcpp/rclcpp.hpp"

class GripperSequenceNode : public rclcpp::Node {
public:
	GripperSequenceNode(ServoControl& servo_control)
	: Node("gripper_sequence_node"), step_(0), servo_(servo_control)
	{
		//servo_.torque(1, false);
		angles_ = {150, 70, 150, 230, 150};

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(4000),
			std::bind(&GripperSequenceNode::timerCallback, this)
		);
		direction_ = false;
	}

private:
	void timerCallback() {

		//servo_.setDistance(700);
		/*
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position: %d", servo_.getClawAngle());
		if (step_ < angles_.size()) {
			servo_.setGripperAngle(angles_[step_]);

			//servo_.setClawAngle(angles_[step_]);
			//servo_.setAngle(angles_[step_]/2);

			if (step_ % 2 == 1) {
                direction_ = !direction_;
                servo_.turn(12, direction_, 1023);
				//servo_.setDistance(40.f);
				//int currentPosition = servo_.getDistance();
        		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position: %d", currentPosition);
			} else {
		        servo_.stop(12);
				//servo_.setGripperAngle(angles_[step_]);
			}
			++step_;
		} else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		    servo_.stop(12);
			timer_->cancel();  // Stop the timer when sequence is done
			RCLCPP_INFO(this->get_logger(), "Gripper sequence complete.");
		}
		*/
	}

	size_t step_;
	ServoControl& servo_;
	std::vector<int> angles_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool direction_;
};

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);
	auto servoControl = ServoControl(62, 11, 12, 1, 22, 2, "/dev/i2c-1", 0x70);
	//auto gripper_node = std::make_shared<GripperSequenceNode>(servoControl);
	auto translator =  std::make_shared<MicrorosTranslation>();
	auto pos_node = std::make_shared<PositionNode>(servoControl);
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(translator);
	//exec.add_node(gripper_node);
	exec.add_node(pos_node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
