import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import transforms3d as t3d
import numpy as np
import pysurvive
import sys
import casadi as cs
from math import pi

class ViveTrackerNode(Node):
    def __init__(self):
        super().__init__('vive_tracker_py_node')
        self.actx = pysurvive.SimpleContext(sys.argv)
        self.publisher = self.create_publisher(PoseStamped, '/vive/pose', 10)

        # self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 10)
        self.done_00 = False
        self.done_200000 = False
        self.done_000200 = False
        self.done_point_4 = False
        self.wait_set00 = False
        self.wait_set000200 = False
        self.wait_set200000 = False
        self.wait_point_4 = False
        self.set_next_turn = False
        self.T_calibration = None
        self.transform_w_in_v = TransformStamped()
        self.tf_lh_to_world = TransformStamped()
        self.tracker_pose_0_0_0 = PoseStamped()
        self.tracker_pose_200_0_0 = PoseStamped()
        self.tracker_pose_0_200_0 = PoseStamped()
        self.tracker_pose_point_4 = PoseStamped()
        self.calibration_pose = PoseStamped()
        self.tracker_pose_vive = PoseStamped()
        self.tracker_pose_world = PoseStamped()

        self.tracker_pose_vive.header.frame_id = 'vive'
        self.tracker_pose_world.header.frame_id = 'odom'

        self.transform_w_in_v.header.frame_id = 'vive'
        self.transform_w_in_v.child_frame_id = 'odom'

        self.calibration_ok = False
        self.counter = 0

        perform_calibration = input("Do you want to calibrate the tracker? (yes/no, default: no): ")
        if perform_calibration == 'yes':
            self.transformation_done = False
        else:
            self.transformation_done = True


        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_callback()

    def load_calibration_matrix(self, yaml_file):

        with open(yaml_file, 'r') as file:
            lines = file.readlines()

            T_calibration = np.zeros((4,4))
            for i in range(4):
                line = lines[i+1]
                line = line.replace('[', '')
                line = line.replace(']', '')
                line = line.replace(',', '')
                line = line.split()
                line.pop(0)
                print(line)
                for j in range(4):
                    T_calibration[i, j] = float(line[j])

        return T_calibration

    def handle_measurement(self, updated):

        poseObj = updated.Pose()
        poseData = poseObj[0]
        poseTimestamp = poseObj[1]
        # self.get_logger().info("RAW: %s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))

        if not self.transformation_done:

            if not self.wait_set00 and self.counter == 200:
                if not self.set_next_turn:
                    input("Place at x:-1, y:-1 and z:0 and press enter")
                    self.set_next_turn = True
                    self.counter = 0
                else:
                    self.tracker_pose_0_0_0.pose.position.x = poseData.Pos[0]
                    self.tracker_pose_0_0_0.pose.position.y = poseData.Pos[1]
                    self.tracker_pose_0_0_0.pose.position.z = poseData.Pos[2]
                    self.tracker_pose_0_0_0.pose.orientation.x = poseData.Rot[0]
                    self.tracker_pose_0_0_0.pose.orientation.y = poseData.Rot[1]
                    self.tracker_pose_0_0_0.pose.orientation.z = poseData.Rot[2]
                    self.tracker_pose_0_0_0.pose.orientation.w = poseData.Rot[3]

                    self.tracker_Rot_0_0_0 = t3d.quaternions.quat2mat([poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]])
                    self.wait_set00 = True
                    self.set_next_turn = False
                    self.counter = 0
                    self.done_00 = True
            
            elif not self.wait_set000200 and self.counter == 200:
                if not self.set_next_turn:
                    input("Place at x:2, y:0.0 and z:000 and press enter")
                    self.set_next_turn = True
                    self.counter = 0
                else:
                    self.tracker_pose_0_200_0.pose.position.x = poseData.Pos[0]
                    self.tracker_pose_0_200_0.pose.position.y = poseData.Pos[1]
                    self.tracker_pose_0_200_0.pose.position.z = poseData.Pos[2]
                    self.tracker_pose_0_200_0.pose.orientation.x = poseData.Rot[0]
                    self.tracker_pose_0_200_0.pose.orientation.y = poseData.Rot[1]
                    self.tracker_pose_0_200_0.pose.orientation.z = poseData.Rot[2]
                    self.tracker_pose_0_200_0.pose.orientation.w = poseData.Rot[3]

                    self.tracker_Rot_0_200_0 = t3d.quaternions.quat2mat([poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]])
                    self.wait_set000200 = True
                    self.set_next_turn = False
                    self.counter = 0
                    self.done_000200 = True
            
            elif not self.wait_set200000 and self.counter == 200:
                if not self.set_next_turn:
                    input("Place at x:2, y: 4, and z:000 and press enter")
                    self.set_next_turn = True
                    self.counter = 0
                else:
                    self.tracker_pose_200_0_0.pose.position.x = poseData.Pos[0]
                    self.tracker_pose_200_0_0.pose.position.y = poseData.Pos[1]
                    self.tracker_pose_200_0_0.pose.position.z = poseData.Pos[2]
                    self.tracker_pose_200_0_0.pose.orientation.x = poseData.Rot[0]
                    self.tracker_pose_200_0_0.pose.orientation.y = poseData.Rot[1]
                    self.tracker_pose_200_0_0.pose.orientation.z = poseData.Rot[2]
                    self.tracker_pose_200_0_0.pose.orientation.w = poseData.Rot[3]

                    self.tracker_Rot_200_0_0 = t3d.quaternions.quat2mat([poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]])
                    self.wait_set200000 = True
                    self.set_next_turn = False
                    self.counter = 0
                    self.done_200000 = True

            elif not self.wait_point_4 and self.counter == 200:
                if not self.set_next_turn:
                    input("Place at x:0, y: 3, and z:000 and press enter")
                    self.set_next_turn = True
                    self.counter = 0
                else:
                    self.tracker_pose_point_4.pose.position.x = poseData.Pos[0]
                    self.tracker_pose_point_4.pose.position.y = poseData.Pos[1]
                    self.tracker_pose_point_4.pose.position.z = poseData.Pos[2]
                    self.tracker_pose_point_4.pose.orientation.x = poseData.Rot[0]
                    self.tracker_pose_point_4.pose.orientation.y = poseData.Rot[1]
                    self.tracker_pose_point_4.pose.orientation.z = poseData.Rot[2]
                    self.tracker_pose_point_4.pose.orientation.w = poseData.Rot[3]

                    self.tracker_Rot_point_4 = t3d.quaternions.quat2mat([poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]])
                    self.wait_point_4 = True
                    self.set_next_turn = False
                    self.counter = 0
                    self.done_point_4 = True

            else:

                if (self.done_00 and self.done_000200 and self.done_200000 and self.done_point_4 and not self.calibration_ok):

                    input("Press enter to compute the calibration matrix numerically.")

                    measured_calib_points = cs.horzcat(
                        cs.vertcat(self.tracker_pose_0_0_0.pose.position.x, self.tracker_pose_0_0_0.pose.position.y, self.tracker_pose_0_0_0.pose.position.z),
                        cs.vertcat(self.tracker_pose_0_200_0.pose.position.x, self.tracker_pose_0_200_0.pose.position.y, self.tracker_pose_0_200_0.pose.position.z),
                        cs.vertcat(self.tracker_pose_200_0_0.pose.position.x, self.tracker_pose_200_0_0.pose.position.y, self.tracker_pose_200_0_0.pose.position.z),
                        cs.vertcat(self.tracker_pose_point_4.pose.position.x, self.tracker_pose_point_4.pose.position.y, self.tracker_pose_point_4.pose.position.z))
                    # ground_truth_points = cs.horzcat(
                    #     cs.vertcat(-0.255,-0.255,0),
                    #     cs.vertcat(2.15,0.0,0),
                    #     cs.vertcat(2.15,3.555,0),
                    #     cs.vertcat(0.0,3.555,0))
                    ground_truth_points = cs.horzcat(
                        cs.vertcat(-1,-1,0),
                        cs.vertcat(2,0.0,0),
                        cs.vertcat(2,4,0),
                        cs.vertcat(0,3,0))
                    # ground_truth_points = cs.horzcat(
                    #     cs.vertcat(-1,-1,0),
                    #     cs.vertcat(2,-1,0),
                    #     cs.vertcat(2,4,0),
                    #     cs.vertcat(0,5,0))
                    
                    opti = cs.Opti()

                    eul = opti.variable(3,1)
                    t = opti.variable(3,1)

                    opti.subject_to(-cs.pi <= (eul <= cs.pi))

                    phi = eul[0]
                    theta = eul[1]
                    psi = eul[2]

                    cr = cs.cos(phi)
                    sr = cs.sin(phi)
                    cp = cs.cos(theta)
                    sp = cs.sin(theta)
                    cy = cs.cos(psi)
                    sy = cs.sin(psi)

                    R = cs.vertcat(
                        cs.horzcat(cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
                        cs.horzcat(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
                        cs.horzcat(-sp, cp*sr, cp*cr)
                    )


                    T = cs.vertcat(
                        cs.horzcat(R, t),
                        cs.horzcat(cs.MX(1,3), 1)
                    )

                    measured_calib_points = cs.vertcat(measured_calib_points, cs.DM.ones(1,4))
                    ground_truth_points = cs.vertcat(ground_truth_points, cs.DM.ones(1,4))

                    transf_meas_calib_points = cs.mtimes(T, measured_calib_points)

                    V = cs.MX(1,1)
                    error_matrix = transf_meas_calib_points - ground_truth_points

                    for i in range(4):
                        e = cs.vertcat(error_matrix[0,i], error_matrix[1,i], error_matrix[2,i])
                        V += cs.sumsqr(e)

                    opti.minimize(V)

                    ipopt_opts = {
                        "print_level": 3, 
                        # "linear_solver": "ma27",
                        "sb": "yes",
                        "tol": 1e-8,
                    }

                    opts = {
                        "expand": True,
                        "verbose": False,
                        "print_time": True,
                        "error_on_fail": True,
                        "ipopt": ipopt_opts
                    }

                    opti.solver("ipopt", opts)

                    sol = opti.solve()

                    eulsol = sol.value(eul)
                    Rsol = sol.value(R)
                    tsol = sol.value(t)
                    T_calibration = sol.value(T)

                    # Print T_calibration
                    self.get_logger().info(f"Calibration matrix:\n{T_calibration}")

                    self.T_calibration = T_calibration

                    input("Press enter to save the calibration matrix to a yaml file.")

                    # Save T_calibration to yaml file
                    with open('calibration_matrix.yaml', 'w') as file:
                        file.write(f"calibration_matrix:\n")
                        for i in range(4):
                            file.write(f"  - [{T_calibration[i,0]}, {T_calibration[i,1]}, {T_calibration[i,2]}, {T_calibration[i,3]}]\n")
                    
                    self.T_calibration = self.load_calibration_matrix('calibration_matrix.yaml')
                    # rotation_quat = t3d.quaternions.mat2quat(Rsol)

                    # self.get_logger().info("Calibration matrix saved to calibration_matrix.yaml")

                    # self.calibration_pose.pose.position.x = tsol[0]
                    # self.calibration_pose.pose.position.y = tsol[1]
                    # self.calibration_pose.pose.position.z = tsol[2]
                    # self.calibration_pose.pose.orientation.x = rotation_quat[0]
                    # self.calibration_pose.pose.orientation.y = rotation_quat[1]
                    # self.calibration_pose.pose.orientation.z = rotation_quat[2]
                    # self.calibration_pose.pose.orientation.w = rotation_quat[3]

                    # self.transform_w_in_v.transform.translation.x = tsol[0]
                    # self.transform_w_in_v.transform.translation.y = tsol[1]
                    # self.transform_w_in_v.transform.translation.z = tsol[2]
                    # self.transform_w_in_v.transform.rotation.x = rotation_quat[0]
                    # self.transform_w_in_v.transform.rotation.y = rotation_quat[1]
                    # self.transform_w_in_v.transform.rotation.z = rotation_quat[2]
                    # self.transform_w_in_v.transform.rotation.w = rotation_quat[3]

                    self.calibration_ok = True
                    
                    self.get_logger().info("Calibration performed")

        else:
            if self.T_calibration is None:
                self.T_calibration = self.load_calibration_matrix('calibration_matrix.yaml')

            self.tracker_pose_vive.header.stamp = self.get_clock().now().to_msg()
            self.tracker_pose_vive.header.frame_id = 'map'  # or whatever frame you're using
            self.tracker_pose_vive.pose.position.x = poseData.Pos[0]
            self.tracker_pose_vive.pose.position.y = poseData.Pos[1]
            self.tracker_pose_vive.pose.position.z = poseData.Pos[2]
            self.tracker_pose_vive.pose.orientation.x = poseData.Rot[0]
            self.tracker_pose_vive.pose.orientation.y = poseData.Rot[1]
            self.tracker_pose_vive.pose.orientation.z = poseData.Rot[2]
            self.tracker_pose_vive.pose.orientation.w = poseData.Rot[3]

            self.calibration_ok = True
            self.done_00 = True
            self.done_000200 = True
            self.done_200000 = True
            self.done_point_4 = True

        self.counter += 1




    def timer_callback(self):
        for obj in self.actx.Objects():
            self.get_logger().info(str(obj.Name(), 'utf-8'))

        while self.actx.Running():
            updated = self.actx.NextUpdated()
            if updated:
                self.handle_measurement(updated)

                if self.calibration_ok and self.T_calibration is not None:
                    
                    quat_tracker_vive = [self.tracker_pose_vive.pose.orientation.x, self.tracker_pose_vive.pose.orientation.y, self.tracker_pose_vive.pose.orientation.z, self.tracker_pose_vive.pose.orientation.w]

                    Rot_tracker_vive = t3d.quaternions.quat2mat(quat_tracker_vive)
                    p_tracker_vive = np.array([self.tracker_pose_vive.pose.position.x, self.tracker_pose_vive.pose.position.y, self.tracker_pose_vive.pose.position.z])

                    T_tracker_vive = np.eye(4)
                    T_tracker_vive[:3, :3] = Rot_tracker_vive
                    T_tracker_vive[:3, 3] = p_tracker_vive

                    pose_tracker_world = np.dot(self.T_calibration, T_tracker_vive)
                    
                    self.get_logger().info(f"Position tracker in world:\t x: {(pose_tracker_world[0, 3])}, y: {pose_tracker_world[1, 3]}, z: {pose_tracker_world[2, 3]}")

                    R_tracker_world = pose_tracker_world[:3, :3]
                    quat_tracker_world = t3d.quaternions.mat2quat(R_tracker_world)
                    roll_world, pitch_world, yaw_world = t3d.euler.quat2euler(quat_tracker_world)

                    self.get_logger().info(f"Position tracker:\t Roll: {(roll_world*180/(3.141592))}, Pitch: {(pitch_world*180/(3.141592))}, Yaw: {(yaw_world*180/(3.141592))}")

                    new_quat = t3d.euler.euler2quat(yaw_world + 0.5*pi, 0, 0)

                    self.tracker_pose_world.header.stamp = self.get_clock().now().to_msg()
                    self.tracker_pose_world.pose.position.x = pose_tracker_world[0, 3]
                    self.tracker_pose_world.pose.position.y = pose_tracker_world[1, 3]
                    self.tracker_pose_world.pose.position.z = pose_tracker_world[2, 3]
                    self.tracker_pose_world.pose.orientation.x = new_quat[0]
                    self.tracker_pose_world.pose.orientation.y = new_quat[1]
                    self.tracker_pose_world.pose.orientation.z = new_quat[2]
                    self.tracker_pose_world.pose.orientation.w = new_quat[3]

                # Publish the PoseStamped message
                self.publisher.publish(self.tracker_pose_world)
                

def main(args=None):
    rclpy.init(args=args)

    node = ViveTrackerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()