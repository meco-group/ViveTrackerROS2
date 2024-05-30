# import pysurvive
# import sys

# actx = pysurvive.SimpleContext(sys.argv)

# for obj in actx.Objects():
#     print(str(obj.Name(), 'utf-8'))

# while actx.Running():
#     updated = actx.NextUpdated()
#     if updated:
#         poseObj = updated.Pose()
#         poseData = poseObj[0]
#         poseTimestamp = poseObj[1]
#         print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pysurvive
import sys

class ViveTrackerNode(Node):
    def __init__(self):
        super().__init__('vive_tracker_py_node')
        self.actx = pysurvive.SimpleContext(sys.argv)
        self.publisher = self.create_publisher(PoseStamped, '/vive/pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        for obj in self.actx.Objects():
            self.get_logger().info(str(obj.Name(), 'utf-8'))

        while self.actx.Running():
            updated = self.actx.NextUpdated()
            if updated:
                poseObj = updated.Pose()
                poseData = poseObj[0]
                poseTimestamp = poseObj[1]
                self.get_logger().info("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))

                # Create a PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'  # or whatever frame you're using
                pose_msg.pose.position.x = poseData.Pos[0]
                pose_msg.pose.position.y = poseData.Pos[1]
                pose_msg.pose.position.z = poseData.Pos[2]
                pose_msg.pose.orientation.x = poseData.Rot[0]
                pose_msg.pose.orientation.y = poseData.Rot[1]
                pose_msg.pose.orientation.z = poseData.Rot[2]
                pose_msg.pose.orientation.w = poseData.Rot[3]

                # Publish the PoseStamped message
                self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ViveTrackerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()