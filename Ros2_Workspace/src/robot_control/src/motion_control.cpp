#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> arm_joints = {
    "Link1-BaseLink",
    "Link2-Link1",
    "Link3-Link2",
    "Link4-Link3",
    "Link5-Link4",
    "Link6-Link5"
};

class MotionControlPublisherCpp : public rclcpp::Node
{
public:
    MotionControlPublisherCpp()
        : Node("motion_control_cpp")
    {
        // Create the publisher of the desired arm 
        arm_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 1); 

        // Create a timer to periodically call the timerCallback function
        timer_ = create_wall_timer(5s, std::bind(&MotionControlPublisherCpp::timerCallback, this));

        frame_id_ = "base_link";

        // Desired time from the trajectory start to arrive at the trajectory point.
        // Needs to be less than or equal to the timer period above to allow
        // the robotic arm to smoothly transition between points.
        duration_sec_ = 2;
        duration_nanosec_ = 0.5 * 1e9;  // (seconds * 1e9)

        // Set the desired goal poses for the robotic arm.
        arm_positions_ = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Home location
            {-1.345, -0.90, 0.264, -0.296, 0.389, -1.5},  // Pick location
            {-1.345, -0.90, 0.264, -0.296, 0.389, -1.5},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Back to Home location
            {1.345, -0.90, 0.264, -0.296, 0.389, -1.5},  // Place location
            {1.345, -0.90, 0.264, -0.296, 0.389, -1.5}
        };

        // Keep track of the current trajectory we are executing
        index_ = 0;
    }

private:
    void timerCallback()
    {
        // Create new JointTrajectory messages for arm 
        auto msg_arm = trajectory_msgs::msg::JointTrajectory();
        msg_arm.header.frame_id = frame_id_;
        msg_arm.joint_names = arm_joints;

        // Create JointTrajectoryPoints for arm 
        auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();

        point_arm.positions = arm_positions_[index_];

        point_arm.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);

        msg_arm.points.push_back(point_arm);

        arm_pose_publisher_->publish(msg_arm);

        // Reset the index 
        if (index_ == arm_positions_.size() - 1) {
            index_ = 0;
        } else {
            index_++;
        }
    }

    // Publishers for arm 
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pose_publisher_;

    // Timer for periodic callback
    rclcpp::TimerBase::SharedPtr timer_;

    // Frame ID for the joint trajectories
    std::string frame_id_;

    // Duration for each trajectory point
    int duration_sec_;
    int duration_nanosec_;

    // Desired goal poses for the robotic arm and gripper
    std::vector<std::vector<double>> arm_positions_;
 
    // Index to keep track of the current trajectory point
    size_t index_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotionControlPublisherCpp>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
