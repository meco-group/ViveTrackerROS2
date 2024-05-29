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
// #include "casadi/casadi.hpp"
// Eigen
#include <Eigen/Dense>


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

Eigen::Matrix4d T_calibration;


class ViveTrackerNode : public rclcpp::Node
{
public:
  ViveTrackerNode() : Node("vive_tracker")
  {
    std::cout << "The sine of 0 is 0" << std::endl;

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vive_tracker_pose", 10);

  }

  ~ViveTrackerNode()
  {
    std::cout << "Killing ViveTrackerNode" << std::endl;
  }

  // casadi::DM T_calibration;

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

    
    T_calibration << 0.211650, -0.977305, -0.008950, 0.035416,
              0.977041, 0.211804, -0.023006, 0.005643,
              0.024379, -0.003876, 0.999695, 0.244721,
              0.000000, 0.000000, 0.000000, 1.00000;

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

    tracker_pose_vive.pose.position.x = pose->Pos[0];
    tracker_pose_vive.pose.position.y = pose->Pos[1];
    tracker_pose_vive.pose.position.z = pose->Pos[2];
    tracker_pose_vive.pose.orientation.x = pose->Rot[0];
    tracker_pose_vive.pose.orientation.y = pose->Rot[1];
    tracker_pose_vive.pose.orientation.z = pose->Rot[2];
    tracker_pose_vive.pose.orientation.w = pose->Rot[3];

    calibration_ok = true;
    done_00 = true;
    done_000200 = true;
    done_200000 = true;
    done_point_4 = true;
  }
  counter++;
  
}

int main(int argc, char ** argv)
{

  std::cout << "Vive Tracker ROS 2" << std::endl;
  
  struct SurviveContext *ctx;

  // Initialize variables
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

  tracker_pose_vive.header.frame_id = "vive";
  tracker_pose_world.header.frame_id = "map";

  // Initialize ROS 2 Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViveTrackerNode>();
  // rclcpp::spin(std::make_shared<ViveTrackerNode>());
  // auto node = rclcpp::Node::make_shared("vive_tracker");

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

  survive_install_pose_fn(ctx, tracker_pose_process_meco);

  survive_startup(ctx);
  if (ctx->state == SURVIVE_STOPPED)
  {
      throw std::runtime_error("Could not start tracker! Exiting.");
      return 1;
  }

  // Loop until stopped
  rclcpp::Rate rate(10); // 10 Hz

  while (rclcpp::ok())
  {
      if (calibration_ok)
      {
        std::cout << "Calibration done" << std::endl;

        tf2::Quaternion quat_tracker_vive(tracker_pose_vive.pose.orientation.x, tracker_pose_vive.pose.orientation.y, tracker_pose_vive.pose.orientation.z, tracker_pose_vive.pose.orientation.w);
        quat_tracker_vive.normalize();
        tf2::Matrix3x3 Rot_tracker_vive;
        Rot_tracker_vive.setRotation(quat_tracker_vive);

        Eigen::Matrix3d Rot_tracker_vive_Eigen;
        Rot_tracker_vive_Eigen << Rot_tracker_vive[0][0], Rot_tracker_vive[0][1], Rot_tracker_vive[0][2],
                      Rot_tracker_vive[1][0], Rot_tracker_vive[1][1], Rot_tracker_vive[1][2],
                      Rot_tracker_vive[2][0], Rot_tracker_vive[2][1], Rot_tracker_vive[2][2];

        Eigen::Vector3d p_tracker_vive(tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z);

        // RCLCPP_INFO(node->get_logger(), "tracker_pose_vive_raw:\t[x:%.2f, y:%.2f, z:%.2f]", tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z);
        std::cout << "tracker_pose_vive_raw:\t[x:" << tracker_pose_vive.pose.position.x << ", y:" << tracker_pose_vive.pose.position.y << ", z:" << tracker_pose_vive.pose.position.z << "]" << std::endl;

        // Create transformation matrix (in Eigen) using Rot_tracker_vive_Eigen and p_tracker_vive
        Eigen::Matrix4d T_tracker_vive;
        T_tracker_vive << Rot_tracker_vive_Eigen(0,0), Rot_tracker_vive_Eigen(0,1), Rot_tracker_vive_Eigen(0,2), p_tracker_vive(0),
                          Rot_tracker_vive_Eigen(1,0), Rot_tracker_vive_Eigen(1,1), Rot_tracker_vive_Eigen(1,2), p_tracker_vive(1),
                          Rot_tracker_vive_Eigen(2,0), Rot_tracker_vive_Eigen(2,1), Rot_tracker_vive_Eigen(2,2), p_tracker_vive(2),
                          0, 0, 0, 1;

        // Compute pose of tracker in the world frame using T_calibration and T_tracker_vive
        Eigen::Matrix4d T_tracker_world = T_calibration * T_tracker_vive;

        // RCLCPP_INFO(node->get_logger(), "pose_tracker_world:\t[x:%.2f, y:%.2f, z:%.2f]", (double) T_tracker_world(0,3), (double) T_tracker_world(1,3), (double) T_tracker_world(2,3));
        std::cout << "pose_tracker_world:\t[x:" << (double) T_tracker_world(0,3) << ", y:" << (double) T_tracker_world(1,3) << ", z:" << (double) T_tracker_world(2,3) << "]" << std::endl;

        // Extract position and orientation of tracker in the world frame
        Eigen::Vector3d p_tracker_world(T_tracker_world(0,3), T_tracker_world(1,3), T_tracker_world(2,3));
        Eigen::Matrix3d Rot_tracker_world;
        Rot_tracker_world << T_tracker_world(0,0), T_tracker_world(0,1), T_tracker_world(0,2),
                             T_tracker_world(1,0), T_tracker_world(1,1), T_tracker_world(1,2),
                             T_tracker_world(2,0), T_tracker_world(2,1), T_tracker_world(2,2);

        // Convert rotation matrix to tf2::Quaternion
        tf2::Quaternion orientation_tracker_world_quaternion;
        tf2::Matrix3x3 rotation_matrix( T_tracker_world(0,0), T_tracker_world(0,1), T_tracker_world(0,2),
                                       T_tracker_world(1,0), T_tracker_world(1,1), T_tracker_world(1,2),
                                       T_tracker_world(2,0), T_tracker_world(2,1), T_tracker_world(2,2));
        rotation_matrix.getRotation(orientation_tracker_world_quaternion);
        double roll_world, pitch_world, yaw_world;
        rotation_matrix.getRPY(roll_world, pitch_world, yaw_world);

        tf2::Quaternion newQuat;
        newQuat.setRPY( 0, 0, -roll_world);

        // print roll, pitch and yaw
        // RCLCPP_INFO(node->get_logger(), "orientation_tracker:\t[roll:%.2f, pitch:%.2f, yaw:%.2f]", roll_world*360/(2*3.14), pitch_world*360/(2*3.14), yaw_world*360/(2*3.14));
        std::cout << "orientation_tracker:\t[roll:" << roll_world*360/(2*3.14) << ", pitch:" << pitch_world*360/(2*3.14) << ", yaw:" << yaw_world*360/(2*3.14) << "]" << std::endl;

        tracker_pose_world.pose.position.x = (double) T_tracker_world(0,3);
        tracker_pose_world.pose.position.y = (double) T_tracker_world(1,3);
        tracker_pose_world.pose.position.z = (double) T_tracker_world(2,3);
        tracker_pose_world.pose.orientation.x = (double) newQuat.getX();
        tracker_pose_world.pose.orientation.y = (double) newQuat.getY();
        tracker_pose_world.pose.orientation.z = (double) newQuat.getZ();
        tracker_pose_world.pose.orientation.w = (double) newQuat.getW();

        // RCLCPP_INFO(node->get_logger(), "Transformed: [x:%.2f, y:%.2f, z:%.2f], \tRaw: [x:%.2f, y:%.2f, z:%.2f], \t Yaw: %.2f", tracker_pose_world.pose.position.x, tracker_pose_world.pose.position.y, tracker_pose_world.pose.position.z, tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z,-roll_world*360/(2*3.14));
        // std::cout << "Transformed: [x:" << tracker_pose_world.pose.position.x << ", y:" << tracker_pose_world.pose.position.y << ", z:" << tracker_pose_world.pose.position.z << "], \tRaw: [x:" << tracker_pose_vive.pose.position.x << ", y:" << tracker_pose_vive.pose.position.y << ", z:" << tracker_pose_vive.pose.position.z << "], \tYaw: " << -roll_world*360/(2*3.14) << std::endl;
        
      }
      
      pose_publisher->publish(tracker_pose_world);

      rclcpp::spin_some(node);
      rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}