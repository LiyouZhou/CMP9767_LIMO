import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import geometry_msgs.msg
import numpy as np
from rclpy.time import Time

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Mover(Node):
    """
    A very simple Roamer implementation for LIMO.
    It simply goes straight until any obstacle is within
    2 m distance and then just simply turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        super().__init__('tf_listener')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
        self.tf2_boradcaster = TransformBroadcaster(self)

    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        relevant_ranges_idx_start = int(len(data.ranges)/2) - 10
        relevant_ranges_idx_end = int(len(data.ranges)/2) + 10
        relevant_ranges = data.ranges[relevant_ranges_idx_start:relevant_ranges_idx_end]
        index_min = np.argmin(relevant_ranges)
        min_dist = relevant_ranges[index_min]
        min_angle = data.angle_min + ((relevant_ranges_idx_start + index_min) * data.angle_increment)
        dx  = np.cos(min_angle) * min_dist
        dy  = np.sin(min_angle) * min_dist

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = Time.to_msg(self.get_clock().now())
        t.header.frame_id = "laser_link"
        t.child_frame_id = "cloest_point_link"
        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(min_angle, 0, np.pi)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf2_boradcaster.sendTransform(t)

        # print("Min: ", min_dist)
        t = Twist()
        if min_dist < 0.5:
            t.angular.z = 0.5
        else:
            t.linear.x = 0.8
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
