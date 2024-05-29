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
#include "tf2/LinearMath/Quaternion.h"
// Libsurvive
#include "survive.h"
// CasADi
#include "casadi/casadi.hpp"


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


  // casadi::DM T_calibration;

  // void tracker_pose_process_meco(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose)
  // {

  // };
};


void tracker_pose_process_meco(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose)
{
  std::cout << "Tracker pose process meco" << std::endl;

  survive_default_pose_process(so, timecode, pose);

  if(!transformation_done)
  {
    std::cout << "Calibration not done yet" << std::endl;
  }
  else{

    // T_calibration = casadi::DM::vertcat({
    //   casadi::DM::horzcat({0.211650, -0.977305, -0.008950, 0.035416 }),
    //   casadi::DM::horzcat({0.977041, 0.211804, -0.023006, 0.005643 }),
    //   casadi::DM::horzcat({0.024379, -0.003876, 0.999695, 0.244721}),
    //   casadi::DM::horzcat({0.000000, 0.000000,   0.000000,   1.00000})});

    tf2::Matrix3x3 R_sol = tf2::Matrix3x3(
              (double) 0.211650, (double) -0.977305, (double) -0.008950, 
              (double) 0.977041, (double) 0.211804, (double)  -0.023006, 
              (double) 0.024379, (double) -0.003876, (double)  0.999695);

    tf2::Quaternion rotation_quat;
    R_sol.getRotation(rotation_quat);


    // calibration_pose.pose.position.x = (double) 0.035416;
    // calibration_pose.pose.position.y = (double) 0.005643;
    // calibration_pose.pose.position.z = (double) 0.244721;
    // calibration_pose.pose.orientation.x = rotation_quat[0];
    // calibration_pose.pose.orientation.y = rotation_quat[1];
    // calibration_pose.pose.orientation.z = rotation_quat[2];
    // calibration_pose.pose.orientation.w = rotation_quat[3];

    // transform_w_in_v.transform.translation.x = (double) 0.035416;
    // transform_w_in_v.transform.translation.y = (double) 0.005643;
    // transform_w_in_v.transform.translation.z = (double) 0.244721;
    // transform_w_in_v.transform.rotation.x = rotation_quat[0];
    // transform_w_in_v.transform.rotation.y = rotation_quat[1];
    // transform_w_in_v.transform.rotation.z = rotation_quat[2];
    // transform_w_in_v.transform.rotation.w = rotation_quat[3];

    // tracker_pose_vive.pose.position.x = pose->Pos[0];
    // tracker_pose_vive.pose.position.y = pose->Pos[1];
    // tracker_pose_vive.pose.position.z = pose->Pos[2];
    // tracker_pose_vive.pose.orientation.x = pose->Rot[0];
    // tracker_pose_vive.pose.orientation.y = pose->Rot[1];
    // tracker_pose_vive.pose.orientation.z = pose->Rot[2];
    // tracker_pose_vive.pose.orientation.w = pose->Rot[3];

    calibration_ok = true;
    done_00 = true;
    done_000200 = true;
    done_200000 = true;
  }
}

int main(int argc, char ** argv)
{

  struct SurviveContext *ctx;

  wait_set00 = false;
  wait_set000200 = false;
  wait_set200000 = false;
  wait_point_4 = false;
  set_next_turn = false;

  done_00 = false;
  done_000200 = false;
  done_200000 = false;
  done_point_4 = false;
  calibration_ok = false;

  // Initialize Tracker
  // char* av[] = {"pgm_name","--configfile", const_cast<char*>(config_file.c_str()), const_cast<char*>("--no-calibrate"), NULL};
  char* av[] = {"pgm_name","--configfile", const_cast<char*>(""), const_cast<char*>("--no-calibrate"), NULL};
  int ac = sizeof(av) / sizeof(char *) - 1;
  ctx = survive_init(ac, av);

  if ((!ctx))
  {
    throw std::runtime_error("Could not initialize! Exiting.");
    return 1;
  }
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViveTrackerNode>());


  survive_install_pose_fn(ctx, tracker_pose_process_meco);

  rclcpp::shutdown();
  return 0;
}