// System
#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
// ROS
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// VIVE
#include "survive.h"
// CasADi
#include <casadi/casadi.hpp>

int counter = 0;
bool wait_set00, wait_set000200, wait_set200000, wait_point_4, set_next_turn, calibration_ok;
double offset_x, offset_y, offset_z, alpha, cos_alpha, sin_alpha, offset_orientation;
double offset_pitch, offset_roll, offset_yaw;
bool transformation_done = true;

//geometry_msgs::Pose tracker_pose;
// geometry_msgs::PoseStamped tracker_pose;
//geometry_msgs::PoseStamped lighthouse_pose;
//geometry_msgs::TransformStamped transformStamped_world_lh;
// geometry_msgs::TransformStamped transformStamped_lh_tracker;
// ros::Publisher pose_publisher;
// ros::Publisher lighthouse_initial_pose_publisher;

ros::Publisher pose_publisher;
bool done_00, done_200000, done_000200, done_point_4;
tf::Matrix3x3 tracker_Rot_0_0_0, tracker_Rot_200_0_0, tracker_Rot_0_200_0, tracker_Rot_point_4;
geometry_msgs::TransformStamped transform_w_in_v, tf_lh_to_world;
geometry_msgs::PoseStamped tracker_pose_0_0_0, tracker_pose_200_0_0, tracker_pose_0_200_0, tracker_pose_point_4, calibration_pose, tracker_pose_vive, tracker_pose_world;

casadi::DM T_calibration;


void tracker_pose_process_meco(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose)
{

  survive_default_pose_process(so, timecode, pose);

  if(!transformation_done){
    if(!wait_set00 && counter == 200){
      if(!set_next_turn){
        std::string inputString;
        std::cout << "Place at x:-0.255, y:-0.255 and z:0 and press enter";
        std::getline(std::cin, inputString);
        set_next_turn = true;
        ros::spinOnce();
        counter = 0;
      } else {
        tracker_pose_0_0_0.pose.position.x = pose->Pos[0];
        tracker_pose_0_0_0.pose.position.y = pose->Pos[1];
        tracker_pose_0_0_0.pose.position.z = pose->Pos[2];
        tracker_pose_0_0_0.pose.orientation.x = pose->Rot[0];
        tracker_pose_0_0_0.pose.orientation.y = pose->Rot[1];
        tracker_pose_0_0_0.pose.orientation.z = pose->Rot[2];
        tracker_pose_0_0_0.pose.orientation.w = pose->Rot[3];

        tracker_Rot_0_0_0.setRotation(tf::Quaternion(pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]));

        offset_x = pose->Pos[0];
        offset_y = pose->Pos[1];
        offset_z = pose->Pos[2];

        tracker_Rot_0_0_0.getRPY(offset_roll, offset_pitch, offset_yaw);

        offset_orientation = offset_yaw;

        // printf("cal pose: [x:%.3f,y:%.3f]\n", (100 * pose->Pos[0]), (100 * pose->Pos[1]));
        // printf("offset: [x:%.3f,y:%.3f]\n", offset_x, offset_y);
        wait_set00 = true;
        set_next_turn = false;
        counter = 0;
        done_00 = true;
      }
    } else if (!wait_set000200 && counter == 200) {
      if(!set_next_turn){
        std::string inputString;
        std::cout << "Place at x:2.15, y:0.0 and z:000 and press enter";
        std::getline(std::cin, inputString);
        set_next_turn = true;
        ros::spinOnce();
        counter = 0;
      } else {
        tracker_pose_0_200_0.pose.position.x = pose->Pos[0];
        tracker_pose_0_200_0.pose.position.y = pose->Pos[1];
        tracker_pose_0_200_0.pose.position.z = pose->Pos[2];
        tracker_pose_0_200_0.pose.orientation.x = pose->Rot[0];
        tracker_pose_0_200_0.pose.orientation.y = pose->Rot[1];
        tracker_pose_0_200_0.pose.orientation.z = pose->Rot[2];
        tracker_pose_0_200_0.pose.orientation.w = pose->Rot[3];

        tracker_Rot_0_200_0.setRotation(tf::Quaternion(pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]));

        wait_set000200 = true;
        set_next_turn = false;
        counter = 0;
        done_000200 = true;
      }
    } else if (!wait_set200000 && counter == 200) {
      if(!set_next_turn){
        // std::cout << "Rotation matrix tracker_Rot_0_200_0: " << std::endl << tracker_Rot_0_200_0 << std::endl;
        std::string inputString;
        std::cout << "Place at x:2.15, y: 3.555, and z:000 and press enter";
        std::getline(std::cin, inputString);
        set_next_turn = true;
        ros::spinOnce();
        counter = 0;
      } else {
        tracker_pose_200_0_0.pose.position.x = pose->Pos[0];
        tracker_pose_200_0_0.pose.position.y = pose->Pos[1];
        tracker_pose_200_0_0.pose.position.z = pose->Pos[2];
        tracker_pose_200_0_0.pose.orientation.x = pose->Rot[0];
        tracker_pose_200_0_0.pose.orientation.y = pose->Rot[1];
        tracker_pose_200_0_0.pose.orientation.z = pose->Rot[2];
        tracker_pose_200_0_0.pose.orientation.w = pose->Rot[3];

        tracker_Rot_200_0_0.setRotation(tf::Quaternion(pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]));

        wait_set200000 = true;
        set_next_turn = false;
        counter = 0;
        done_200000 = true;
      }
    } else if (!wait_point_4 && counter == 200) {
      if(!set_next_turn){
        // std::cout << "Rotation matrix tracker_Rot_0_200_0: " << std::endl << tracker_Rot_0_200_0 << std::endl;
        std::string inputString;
        std::cout << "Place at x:0, y: 3.555, and z:000 and press enter";
        std::getline(std::cin, inputString);
        set_next_turn = true;
        ros::spinOnce();
        counter = 0;
      } else {
        tracker_pose_point_4.pose.position.x = pose->Pos[0];
        tracker_pose_point_4.pose.position.y = pose->Pos[1];
        tracker_pose_point_4.pose.position.z = pose->Pos[2];
        tracker_pose_point_4.pose.orientation.x = pose->Rot[0];
        tracker_pose_point_4.pose.orientation.y = pose->Rot[1];
        tracker_pose_point_4.pose.orientation.z = pose->Rot[2];
        tracker_pose_point_4.pose.orientation.w = pose->Rot[3];

        tracker_Rot_point_4.setRotation(tf::Quaternion(pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]));

        wait_point_4 = true;
        set_next_turn = false;
        counter = 0;
        done_point_4 = true;
      }
    } else {
      // auto measured_calib_points = casadi::DM(3,3);
      // auto ground_truth_points = casadi::DM(3,3);
      if (done_00 && done_000200 && done_200000 && done_point_4 && !calibration_ok){

        std::string inputString;
        std::cout << "Press enter to compute the calibration matrix numerically.";
        std::getline(std::cin, inputString);
      
        // auto measured_calib_points = casadi::DM::horzcat({
        //     casadi::DM::vertcat({tracker_pose_0_0_0.pose.position.x, tracker_pose_0_0_0.pose.position.y, tracker_pose_0_0_0.pose.position.z}),
        //     casadi::DM::vertcat({tracker_pose_0_200_0.pose.position.x, tracker_pose_0_200_0.pose.position.y, tracker_pose_0_200_0.pose.position.z}),
        //     casadi::DM::vertcat({tracker_pose_200_0_0.pose.position.x, tracker_pose_200_0_0.pose.position.y, tracker_pose_200_0_0.pose.position.z})});
        // auto ground_truth_points = casadi::DM::horzcat({
        //     // casadi::DM::vertcat({0,0,0}),
        //     // casadi::DM::vertcat({0,1.3,0}),
        //     // casadi::DM::vertcat({1.3,0,0})});
        //     casadi::DM::vertcat({-0.25,-0.25,0}),
        //     casadi::DM::vertcat({2.15,0.0,0}),
        //     casadi::DM::vertcat({2.15,3.555,0})});

        auto measured_calib_points = casadi::DM::horzcat({
            casadi::DM::vertcat({tracker_pose_0_0_0.pose.position.x, tracker_pose_0_0_0.pose.position.y, tracker_pose_0_0_0.pose.position.z}),
            casadi::DM::vertcat({tracker_pose_0_200_0.pose.position.x, tracker_pose_0_200_0.pose.position.y, tracker_pose_0_200_0.pose.position.z}),
            casadi::DM::vertcat({tracker_pose_200_0_0.pose.position.x, tracker_pose_200_0_0.pose.position.y, tracker_pose_200_0_0.pose.position.z}),
            casadi::DM::vertcat({tracker_pose_point_4.pose.position.x, tracker_pose_point_4.pose.position.y, tracker_pose_point_4.pose.position.z})});
        auto ground_truth_points = casadi::DM::horzcat({
            casadi::DM::vertcat({-0.255,-0.255,0}),
            casadi::DM::vertcat({2.15,0.0,0}),
            casadi::DM::vertcat({2.15,3.555,0}),
            casadi::DM::vertcat({0.0,3.555,0})});

        // double x, y, z; //, o_d;

        // x = pose->Pos[0] - offset_x;
        // y = pose->Pos[1] - offset_y;
        // z = pose->Pos[2] - offset_z;


        auto opti = casadi::Opti();

        auto eul = opti.variable(3, 1);
        auto t = opti.variable(3, 1);

        opti.subject_to(-casadi::pi <= (eul <= casadi::pi));

        auto phi = eul(0);
        auto theta = eul(1);
        auto psi = eul(2);

        auto cr = casadi::MX::cos(phi);
        auto sr = casadi::MX::sin(phi);
        auto cp = casadi::MX::cos(theta);
        auto sp = casadi::MX::sin(theta);
        auto cy = casadi::MX::cos(psi);
        auto sy = casadi::MX::sin(psi);

        auto R = casadi::MX::vertcat({casadi::MX::horzcat({cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr}),
                                casadi::MX::horzcat({sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr}),
                                casadi::MX::horzcat({-sp, cp*sr, cp*cr})});
        
        auto T = casadi::MX::vertcat({casadi::MX::horzcat({R, t}), casadi::MX::horzcat({casadi::MX(1,3),1})});

        measured_calib_points = casadi::DM::vertcat({measured_calib_points, casadi::DM::ones(1,4)});
        ground_truth_points = casadi::DM::vertcat({ground_truth_points, casadi::DM::ones(1,4)});

        auto transf_meas_calib_points = casadi::MX::mtimes({T, measured_calib_points});

        auto V = casadi::MX(1,1);
        auto error_matrix = transf_meas_calib_points - ground_truth_points;

        for(int i = 0; i < 4; i++){
          auto e = casadi::MX::vertcat({error_matrix(0,i), error_matrix(1,i), error_matrix(2,i)});
          V = V + casadi::MX::sumsqr(e);
        }

        opti.minimize(V);

        casadi::Dict opts;
        casadi::Dict ipopt_opts;

        opts["expand"] = true;
        opts["verbose"] = false;
        opts["print_time"] = true;
        opts["error_on_fail"] = true;
        
        ipopt_opts["print_level"] = 3;
        ipopt_opts["linear_solver"] = "ma27";
        ipopt_opts["sb"] = "yes";
        ipopt_opts["tol"] = 1e-8;
        opts["ipopt"] = ipopt_opts;

        opti.solver("ipopt", opts);
        // opti.solver("ipopt");

        auto sol = opti.solve();

        auto eulsol = sol.value(eul);
        auto Rsol = sol.value(R);
        auto tsol = sol.value(t);
        T_calibration = sol.value(T);

        printf("T_calibration: \n");
        printf("%f  |  %f  |  %f  |  %f \n", (double) T_calibration(0,0), (double) T_calibration(0,1), (double) T_calibration(0,2), (double) T_calibration(0,3));
        printf("%f  |  %f  |  %f  |  %f \n", (double) T_calibration(1,0), (double) T_calibration(1,1), (double) T_calibration(1,2), (double) T_calibration(1,3));
        printf("%f  |  %f  |  %f  |  %f \n", (double) T_calibration(2,0), (double) T_calibration(2,1), (double) T_calibration(2,2), (double) T_calibration(2,3));
        printf("%f  |  %f  |  %f  |  %f \n", (double) T_calibration(3,0), (double) T_calibration(3,1), (double) T_calibration(3,2), (double) T_calibration(3,3));

        tf::Matrix3x3 R_sol = tf::Matrix3x3(
                (double) Rsol(0,0), (double) Rsol(0,1), (double) Rsol(0,2), 
                (double) Rsol(1,0), (double) Rsol(1,1), (double) Rsol(1.2), 
                (double) Rsol(2,0), (double) Rsol(2,1), (double) Rsol(2,2));

        // tf::Vector3 translation_vec((double) tsol(0,0), (double) tsol(1,0), (double) tsol(2,0));

        // Wait for enter to be pressed
        std::cout << "Press enter to continue.";
        std::getline(std::cin, inputString);
        

        
        tf::Quaternion rotation_quat;
        R_sol.getRotation(rotation_quat);


        rotation_quat.normalize();

        // transform_w_in_v.header.frame_id = "vive";
        // transform_w_in_v.child_frame_id = "map";

        calibration_pose.pose.position.x = (double) tsol(0,0);
        calibration_pose.pose.position.y = (double) tsol(1,0);
        calibration_pose.pose.position.z = (double) tsol(2,0);
        calibration_pose.pose.orientation.x = rotation_quat[0];
        calibration_pose.pose.orientation.y = rotation_quat[1];
        calibration_pose.pose.orientation.z = rotation_quat[2];
        calibration_pose.pose.orientation.w = rotation_quat[3];

        transform_w_in_v.transform.translation.x = (double) tsol(0,0);
        transform_w_in_v.transform.translation.y = (double) tsol(1,0);
        transform_w_in_v.transform.translation.z = (double) tsol(2,0);
        transform_w_in_v.transform.rotation.x = rotation_quat[0];
        transform_w_in_v.transform.rotation.y = rotation_quat[1];
        transform_w_in_v.transform.rotation.z = rotation_quat[2];
        transform_w_in_v.transform.rotation.w = rotation_quat[3];

        calibration_ok = true;
      }

      tracker_pose_vive.pose.position.x = pose->Pos[0];
      tracker_pose_vive.pose.position.y = pose->Pos[1];
      tracker_pose_vive.pose.position.z = pose->Pos[2];
      tracker_pose_vive.pose.orientation.x = pose->Rot[0];
      tracker_pose_vive.pose.orientation.y = pose->Rot[1];
      tracker_pose_vive.pose.orientation.z = pose->Rot[2];
      tracker_pose_vive.pose.orientation.w = pose->Rot[3];

    }
  }
  else{


    // T_calibration = casadi::DM::vertcat({
    //   casadi::DM::horzcat({0.180781, -0.983196,  -0.025395, -0.029177}),
    //   casadi::DM::horzcat({0.983516, 0.180821,   0.000696,  0.094762}),
    //   casadi::DM::horzcat({0.003908, -0.025102,  0.999677,   -0.032807}),
    //   casadi::DM::horzcat({0.000000, 0.000000,   0.000000,   1.00000})});

    // tf::Matrix3x3 R_sol = tf::Matrix3x3(
    //           (double) 0.180781, (double) -0.983196, (double) -0.025395, 
    //           (double) 0.983516, (double) 0.180821, (double)  0.000696, 
    //           (double) 0.003908, (double) -0.025102, (double)  0.999677);


    // T_calibration = casadi::DM::vertcat({
    //   casadi::DM::horzcat({0.220277, -0.975293, 0.016777, -0.013258 }),
    //   casadi::DM::horzcat({0.975433, 0.220195, -0.006618, -0.032442 }),
    //   casadi::DM::horzcat({0.002761, 0.017822, 0.999837, -0.001516 }),
    //   casadi::DM::horzcat({0.000000, 0.000000,   0.000000,   1.00000})});

    // tf::Matrix3x3 R_sol = tf::Matrix3x3(
    //           (double) 0.220277, (double) -0.975293, (double) 0.016777, 
    //           (double) 0.975433, (double) 0.220195, (double)  -0.006618, 
    //           (double) 0.002761, (double) 0.017822, (double)  0.999837);


    T_calibration = casadi::DM::vertcat({
      casadi::DM::horzcat({0.211650, -0.977305, -0.008950, 0.035416 }),
      casadi::DM::horzcat({0.977041, 0.211804, -0.023006, 0.005643 }),
      casadi::DM::horzcat({0.024379, -0.003876, 0.999695, 0.244721}),
      casadi::DM::horzcat({0.000000, 0.000000,   0.000000,   1.00000})});

    tf::Matrix3x3 R_sol = tf::Matrix3x3(
              (double) 0.211650, (double) -0.977305, (double) -0.008950, 
              (double) 0.977041, (double) 0.211804, (double)  -0.023006, 
              (double) 0.024379, (double) -0.003876, (double)  0.999695);

// 0.211650  |  -0.977305  |  -0.008950  |  0.035416 
// 0.977041  |  0.211804  |  -0.023006  |  0.005643 
// 0.024379  |  -0.003876  |  0.999695  |  0.244721 
// 0.000000  |  0.000000  |  0.000000  |  1.000000
      
    tf::Quaternion rotation_quat;
    R_sol.getRotation(rotation_quat);


    calibration_pose.pose.position.x = (double) 0.035416;
    calibration_pose.pose.position.y = (double) 0.005643;
    calibration_pose.pose.position.z = (double) 0.244721;
    calibration_pose.pose.orientation.x = rotation_quat[0];
    calibration_pose.pose.orientation.y = rotation_quat[1];
    calibration_pose.pose.orientation.z = rotation_quat[2];
    calibration_pose.pose.orientation.w = rotation_quat[3];

    transform_w_in_v.transform.translation.x = (double) 0.035416;
    transform_w_in_v.transform.translation.y = (double) 0.005643;
    transform_w_in_v.transform.translation.z = (double) 0.244721;
    transform_w_in_v.transform.rotation.x = rotation_quat[0];
    transform_w_in_v.transform.rotation.y = rotation_quat[1];
    transform_w_in_v.transform.rotation.z = rotation_quat[2];
    transform_w_in_v.transform.rotation.w = rotation_quat[3];

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
  }
  counter++;

  // //printf("Pose: [x:%.3f,y:%.3f]\n", (100 * pose->Pos[0]), (100 * pose->Pos[1]));
  // ros::Time t_now = ros::Time::now();
  
  // tracker_pose_vive.header.stamp = t_now;
  // tracker_pose_world.header.stamp = t_now;
  // transform_w_in_v.header.stamp = t_now;

  // tracker_pose_vive.pose.position.x = pose->Pos[0];
  // tracker_pose_vive.pose.position.y = pose->Pos[1];
  // tracker_pose_vive.pose.position.z = pose->Pos[2];
  // tracker_pose_vive.pose.orientation.x = pose->Rot[0];
  // tracker_pose_vive.pose.orientation.y = pose->Rot[1];
  // tracker_pose_vive.pose.orientation.z = pose->Rot[2];
  // tracker_pose_vive.pose.orientation.w = pose->Rot[3];

  // tfBuffer.transform(tracker_pose_vive, tracker_pose_world, "map");

  // // tracker_pose_world = tfBuffer.transform(tracker_pose_vive, "map");


  // // printf("Pose: [%s][x:% 02.3f,y:% 02.3f,z:% 02.3f] [% 02.2f,% 02.2f,% 02.2f,% 02.2f]\n", so->codename, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
  // // printf("Pose: [%s][x:% 02.3f,y:% 02.3f,z:% 02.3f] [% 02.2f,% 02.2f,% 02.2f,% 02.2f]\n", so->codename, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

/*
void lighthouse_pose_process(SurviveContext *tctx, uint8_t lighthouse, SurvivePose *pose, SurvivePose *obj_pose)
{
  survive_default_lighthouse_pose_process(tctx, lighthouse, pose, obj_pose);
  lighthouse_pose.pose.position.x = pose->Pos[0];
  lighthouse_pose.pose.position.y = pose->Pos[1];
  lighthouse_pose.pose.position.z = pose->Pos[2];
  lighthouse_pose.pose.orientation.x = pose->Rot[0];
  lighthouse_pose.pose.orientation.y = pose->Rot[1];
  lighthouse_pose.pose.orientation.z = pose->Rot[2];
  lighthouse_pose.pose.orientation.w = pose->Rot[3];
  //printf("LH Pose: [%d][% 02.2f,% 02.2f,% 02.2f] [% 02.2f,% 02.2f,% 02.2f,% 02.2f]\n", lighthouse, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}
*/
// void tracker_pose_tf()
// {
//   static tf2_ros::TransformBroadcaster stbroadc;

//   transform_w_in_v.transform.translation.x = calibration_pose.pose.position.x;
//   transform_w_in_v.transform.translation.y = calibration_pose.pose.position.y;
//   transform_w_in_v.transform.translation.z = calibration_pose.pose.position.z;
//   transform_w_in_v.transform.rotation.x = calibration_pose.pose.orientation.x;
//   transform_w_in_v.transform.rotation.y = calibration_pose.pose.orientation.y;
//   transform_w_in_v.transform.rotation.z = calibration_pose.pose.orientation.z;
//   transform_w_in_v.transform.rotation.w = calibration_pose.pose.orientation.w;

//   stbroadc.sendTransform(transform_w_in_v);


// //   static tf2_ros::TransformBroadcaster br;

// //   transformStamped_lh_tracker.transform.translation.x = tracker_pose.pose.position.x;
// //   transformStamped_lh_tracker.transform.translation.y = tracker_pose.pose.position.y;
// //   transformStamped_lh_tracker.transform.translation.z = tracker_pose.pose.position.z;
// //   transformStamped_lh_tracker.transform.rotation.x = -tracker_pose.pose.orientation.z;
// //   transformStamped_lh_tracker.transform.rotation.y = tracker_pose.pose.orientation.y;
// //   transformStamped_lh_tracker.transform.rotation.z = -tracker_pose.pose.orientation.x;
// //   transformStamped_lh_tracker.transform.rotation.w = tracker_pose.pose.orientation.w;

// // /*
// //   transformStamped_world_lh.transform.translation.x = lighthouse_pose.pose.position.x;
// //   transformStamped_world_lh.transform.translation.y = lighthouse_pose.pose.position.y;
// //   transformStamped_world_lh.transform.translation.z = lighthouse_pose.pose.position.z;
// //   transformStamped_world_lh.transform.rotation.x = lighthouse_pose.pose.orientation.x;
// //   transformStamped_world_lh.transform.rotation.y = lighthouse_pose.pose.orientation.y;
// //   transformStamped_world_lh.transform.rotation.z = lighthouse_pose.pose.orientation.z;
// //   transformStamped_world_lh.transform.rotation.w = lighthouse_pose.pose.orientation.w;
// // */
// //   br.sendTransform(transformStamped_lh_tracker);
// //   //br.sendTransform(transformStamped_world_lh);
// }

// void puslish_pose_and_tf()
// {
//   ros::Time t_now = ros::Time::now();
//   tracker_pose.header.stamp = t_now;
//   //lighthouse_pose.header.stamp = t_now;
//   transformStamped_lh_tracker.header.stamp = t_now;
//   //transformStamped_world_lh.header.stamp = t_now;
//   pose_publisher.publish(tracker_pose);
//   //lighthouse_initial_pose_publisher.publish(lighthouse_pose);
//   tracker_pose_tf();
// }

int main(int argc, char **argv)
{
    std::cout << "ROS Vive Tracker\n";
    struct SurviveContext *ctx;
    std::string config_file;
    std::string nspace(getenv("ROS_NAMESPACE"));

    if (nspace.empty())
    {
      throw std::runtime_error("Could not find namespace. Check if ROS_NAMESPACE is defined");
      return 1;
    }

    // Initialize ROS
    std::cout << "ROS init\n";
    ros::init(argc, argv, "vive_tracker", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    n.param<std::string>("config_file", config_file, "config.conf");
    n.deleteParam("config_file");

    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("tracker_pose", 100);
    
    wait_set00 = false;
    wait_set000200 = false;
    wait_set200000 = false;
    set_next_turn = false;

    done_00 = false;
    done_000200 = false;
    done_200000 = false;
    calibration_ok = false;

    // Configuring frames
    // tracker_pose.header.frame_id = nspace + "_world_frame";
    tracker_pose_vive.header.frame_id = "vive";
    tracker_pose_world.header.frame_id = "map";

    transform_w_in_v.header.frame_id = "vive";
    transform_w_in_v.child_frame_id = "map";
  
    // Initialize Tracker
    char* av[] = {"pgm_name","--configfile", const_cast<char*>(config_file.c_str()), const_cast<char*>("--no-calibrate"), NULL};
    int ac = sizeof(av) / sizeof(char *) - 1;
    std::cout << "Before init\n";
    ctx = survive_init(ac, av);
    if ((!ctx))
    {
      throw std::runtime_error("Could not initialize! Exiting.");
      return 1;
    }
    //survive_install_raw_pose_fn(ctx, testprog_raw_pose_process);
    std::cout << "Before pose\n";

    survive_install_pose_fn(ctx, tracker_pose_process_meco);
    
    //survive_install_lighthouse_pose_fn(ctx, lighthouse_pose_process);
    std::cout << "Before startup\n";
    
    survive_startup(ctx);
    if (ctx->state == SURVIVE_STOPPED)
    {
        throw std::runtime_error("Could not start tracker! Exiting.");
        return 1;
    }

    // Loop until stopped
    
    while (ros::ok() & (survive_poll(ctx) == 0))
    {

        // // if (done_00 && done_000200 && done_200000){
        // // ros::Time t_now = ros::Time::now();
        
        // // tracker_pose_vive.header.stamp = t_now;  
        // // tracker_pose_world.header.stamp = t_now;
        // // transform_w_in_v.header.stamp = t_now;


        // static tf2_ros::TransformBroadcaster stbroadc;

        // transform_w_in_v.transform.translation.x = calibration_pose.pose.position.x;
        // transform_w_in_v.transform.translation.y = calibration_pose.pose.position.y;
        // transform_w_in_v.transform.translation.z = calibration_pose.pose.position.z;
        // transform_w_in_v.transform.rotation.x = calibration_pose.pose.orientation.x;
        // transform_w_in_v.transform.rotation.y = calibration_pose.pose.orientation.y;
        // transform_w_in_v.transform.rotation.z = calibration_pose.pose.orientation.z;
        // transform_w_in_v.transform.rotation.w = calibration_pose.pose.orientation.w;

        // stbroadc.sendTransform(transform_w_in_v);

        // // tf2::doTransform(tracker_pose_vive, tracker_pose_world, transform_w_in_v);

        // //------------------------------------------
        // // tfBuffer.transform(tracker_pose_vive, tracker_pose_world, "map");

        // // // // tracker_pose_world = tfBuffer.transform(tracker_pose_vive, "map");
        // // // printf("Transformed: [x:%.2f, y:%.2f, z:%.2f], \tRaw: [x:%.2f, y:%.2f, z:%.2f]\n", tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z, pose->Pos[0], pose->Pos[1], pose->Pos[2]);
    
        if (calibration_ok){

            //===========================================
            tf::Quaternion quat_tracker_vive(tracker_pose_vive.pose.orientation.x, tracker_pose_vive.pose.orientation.y, tracker_pose_vive.pose.orientation.z, tracker_pose_vive.pose.orientation.w);
            quat_tracker_vive.normalize();
            tf::Matrix3x3 Rot_tracker_vive;
            Rot_tracker_vive.setRotation(quat_tracker_vive);

            casadi::DM Rot_tracker_vive_DM = casadi::DM::vertcat({
                  casadi::DM::horzcat({(double) Rot_tracker_vive[0][0], (double) Rot_tracker_vive[0][1], (double) Rot_tracker_vive[0][2]}), 
                  casadi::DM::horzcat({(double) Rot_tracker_vive[1][0], (double) Rot_tracker_vive[1][1], (double) Rot_tracker_vive[1][2]}), 
                  casadi::DM::horzcat({(double) Rot_tracker_vive[2][0], (double) Rot_tracker_vive[2][1], (double) Rot_tracker_vive[2][2]})
                });

            casadi::DM p_tracker_vive = casadi::DM::vertcat({tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z});
            ROS_INFO("tracker_pose_vive_raw:\t[x:%.2f, y:%.2f, z:%.2f]", tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z);

            casadi::DM T_tracker_vive_DM = casadi::DM::vertcat({
              casadi::DM::horzcat({Rot_tracker_vive_DM,  p_tracker_vive}),
              casadi::DM::horzcat({casadi::DM::zeros(1,3), 1})
            });

            casadi::DM pose_tracker_world = casadi::DM::mtimes(T_calibration, T_tracker_vive_DM);

            ROS_INFO("pose_tracker_world:\t[x:%.2f, y:%.2f, z:%.2f]", (double) pose_tracker_world(0,3), (double) pose_tracker_world(1,3), (double) pose_tracker_world(2,3));

            tf::Quaternion orientation_tracker_world_quaternion;
            tf::Matrix3x3 rotation_matrix((double) pose_tracker_world(0,0),(double) pose_tracker_world(0,1), (double) pose_tracker_world(0,2),
                                          (double) pose_tracker_world(1,0),(double) pose_tracker_world(1,1), (double) pose_tracker_world(1,2),
                                          (double) pose_tracker_world(2,0),(double) pose_tracker_world(2,1), (double) pose_tracker_world(2,2));
            rotation_matrix.getRotation(orientation_tracker_world_quaternion);
            double roll_world, pitch_world, yaw_world;
            rotation_matrix.getRPY(roll_world, pitch_world, yaw_world);

            tf2::Quaternion newQuat;
            newQuat.setRPY( 0, 0, -roll_world );

            // print roll, pitch and yaw
            ROS_INFO("orientation_tracker:\t[roll:%.2f, pitch:%.2f, yaw:%.2f]", roll_world*360/(2*3.14), pitch_world*360/(2*3.14), yaw_world*360/(2*3.14));


            tracker_pose_world.pose.position.x = (double) pose_tracker_world(0,3);
            tracker_pose_world.pose.position.y = (double) pose_tracker_world(1,3);
            tracker_pose_world.pose.position.z = (double) pose_tracker_world(2,3);
            tracker_pose_world.pose.orientation.x = (double) newQuat.getX();
            tracker_pose_world.pose.orientation.y = (double) newQuat.getY();
            tracker_pose_world.pose.orientation.z = (double) newQuat.getZ();
            tracker_pose_world.pose.orientation.w = (double) newQuat.getW();

            //===========================================
            // ROS_INFO("Transformed: [x:%.2f, y:%.2f, z:%.2f], \tRaw: [x:%.2f, y:%.2f, z:%.2f], \t Yaw: %.2f", tracker_pose_world.pose.position.x, tracker_pose_world.pose.position.y, tracker_pose_world.pose.position.z, tracker_pose_vive.pose.position.x, tracker_pose_vive.pose.position.y, tracker_pose_vive.pose.position.z,-roll_world*360/(2*3.14));
        }


        // TODO: take multiple measurements per calibration point and average them, to remove noise.
        // TODO: save calibration transformation in yaml
        // TODO: load calibration transformation from yaml

        // TODO: use customized calibration points (ask user or read from yaml)
        // TODO: Create ROS package around this

        pose_publisher.publish(tracker_pose_world);


        ros::spinOnce();

    }

    return 0;
}