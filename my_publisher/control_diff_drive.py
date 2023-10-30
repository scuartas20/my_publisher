import rclpy
import numpy as np
#import rospy
from rclpy.node import Node


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from math import atan2



class ControlDiffDrive(Node):

    x = 0.0
    y = 0.0
    theta = 0.0

    def __init__(self):
        global goal_x
        global goal_y

        super().__init__('control_diff_drive')
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_odom,10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_scan,10)
        self.subscription = self.create_subscription(PoseStamped,'/goal_pose',self.listener_goal,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        goal_x = 0.0
        goal_y = 0.0


    


    def listener_odom(self, msg):
        global x
        global y 
        global theta

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        r = R.from_quat([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        (theta,pitch,roll) = r.as_euler('zyx',degrees=True)

        #self.get_logger().info('I heard: "%s"' % roll)

        #goal = Point()
        #goal_x = -2.0
        #goal_y = -4.0

        speed = Twist()
        #rate = rospy.Rate(4)

        angle_to_goal = 0.0
        inc_x = goal_x - x
        inc_y = goal_y - y
        angle_to_goal = (atan2(inc_y,inc_x))*180/3.1415
        #self.get_logger().info('I heard: "%s"' % angle_to_goal)

        if abs(angle_to_goal - theta) > 1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
            speed.angular.x = 0.0
            speed.angular.y = 0.0
        self.publisher_.publish(speed)
        self.get_logger().info('I heard: "%s"' % (angle_to_goal - theta))
            
                


    def listener_scan(self, msg):
        a = 1
#self.get_logger().info('I heard: "%s"' % msg.twist.twist.linear)

    def listener_goal(self, msg):
        global goal_x,goal_y
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

#self.get_logger().info('I heard: "%s"' % msg.twist.twist.linear)


def main(args=None):
    rclpy.init(args=args)

    control_diff_drive = ControlDiffDrive()

    rclpy.spin(control_diff_drive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_diff_drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()