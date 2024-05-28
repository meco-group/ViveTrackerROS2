// System
#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"
// Libsurvive
#include "survive.h"
// CasADi
#include "casadi/casadi.hpp"

class ViveTrackerNode : public rclcpp::Node
{
public:
  ViveTrackerNode() : Node("vive_tracker")
  {
    
    std::cout << "The sine of 0 is 0" << std::endl;
  }

  ~ViveTrackerNode()
  {
    std::cout << "Killing ViveTrackerNode" << std::endl;
  }

  int counter = 0;
  bool wait_set00, wait_set000200, wait_set200000, wait_point_4, set_next_turn, calibration_ok;
  double offset_x, offset_y, offset_z, alpha, cos_alpha, sin_alpha, offset_orientation;
  double offset_pitch, offset_roll, offset_yaw;
  bool transformation_done = true;

  // write publisher for pose of the tracker
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  bool done_00, done_200000, done_000200, done_point_4;
  tf2::Matrix3x3 tracker_Rot_0_0_0, tracker_Rot_200_0_0, tracker_Rot_0_200_0, tracker_Rot_point_4;
  geometry_msgs::msg::TransformStamped transform_w_in_v, tf_lh_to_world;
  geometry_msgs::msg::PoseStamped tracker_pose_0_0_0, tracker_pose_200_0_0, tracker_pose_0_200_0, tracker_pose_point_4, calibration_pose, tracker_pose_vive, tracker_pose_world;
  
  // casadi::DM T_calibration;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViveTrackerNode>());
  rclcpp::shutdown();
  return 0;
}