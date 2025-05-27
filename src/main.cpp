#include "PositionNode.h"
#include "rclcpp/rclcpp.hpp"

class GripperSequenceNode : public rclcpp::Node {
public:
	GripperSequenceNode()
	: Node("gripper_sequence_node"), step_(0), servo_(22, 2, 11, 1, 62)
	{
		angles_ = {300, 150, 0, 150};
		timer_ = this->create_wall_timer(
			std::chrono::seconds(2),
			std::bind(&GripperSequenceNode::timerCallback, this)
		);
		direction_ = false;
	}
	ServoControl & getServo() {
		return servo_;
	}

private:
	void timerCallback() {
		if (step_ < angles_.size()) {
			if (step_ % 2 == 1) {
				direction_ = !direction_;
				servo_.setDistance(40.f);
				int currentPosition = servo_.getDistance();
        		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position: %d", currentPosition);
			} else {
				servo_.setGripperAngle(angles_[step_]);
			}
			++step_;
		} else {
			timer_->cancel();  // Stop the timer when sequence is done
			RCLCPP_INFO(this->get_logger(), "Gripper sequence complete.");
		}
	}

	size_t step_;
	ServoControl servo_;
	std::vector<int> angles_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool direction_;
};

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);
	auto gripper_node = std::make_shared<GripperSequenceNode>();
	auto pos_node = std::make_shared<PositionNode>(gripper_node->getServo());  // Assuming getServo() exists
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(gripper_node);
	exec.add_node(pos_node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
