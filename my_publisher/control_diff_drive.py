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
        global goal_x, goal_y, Kp, obstacle_angle, obstacle_distance

        super().__init__('control_diff_drive')
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_odom,10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_scan,10)
        self.subscription = self.create_subscription(PoseStamped,'/goal_pose',self.listener_goal,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        goal_x = 0.0
        goal_y = 0.0
        Kp = -0.05
        obstacle_distance = 1000000000.0
        obstacle_angle = 0.0    


    def listener_odom(self, msg):
        global x
        global y 
        global theta
        global Kp

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        r = R.from_quat([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        (theta,pitch,roll) = r.as_euler('zyx',degrees=True)

        speed = Twist()

        angle_to_goal = 0.0
        inc_x = goal_x - x
        inc_y = goal_y - y
        angle_to_goal = (atan2(inc_y,inc_x))*180/3.1415
        #self.get_logger().info('I heard: "%s"' % angle_to_goal)
        cons_speed = 1.0
        speed.linear.x = cons_speed
        angle_error = theta - angle_to_goal
        speed.angular.z = Kp*angle_error

        if obstacle_distance < 1.0:
            #speed.linear.x = float(cons_speed) - float((1/obstacle_distance))
            speed.linear.x = 0.5
            angle_error = angle_to_goal + (obstacle_angle*180/3.1415)-180.0
            speed.angular.z = float(0.0075*angle_error) 
        if (abs(inc_x)<0.3) and (abs(inc_y)<0.3):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        self.publisher_.publish(speed)
        #self.get_logger().info('I heard: "%s"' % abs(angle_to_goal - theta))
            
                


    def listener_scan(self, msg):
        #valores 16 a 20 son los necesarios para evadir obstaculos
        #Angle min -3.140000104904175
        #Increment 0.1794285774230957
        global obstacle_distance, obstacle_angle
        distances_array = np.array([msg.ranges,np.arange(-3.140000104904175,3.140000104904176,0.1794285774230957)])
        abstraction_array = distances_array[:,13:22]
        if min(abstraction_array[0,:]) < 9000000:
            index_row , index_col = np.where(abstraction_array == (min(abstraction_array[0,:])))
            obstacle_distance = abstraction_array[index_row,index_col]
            obstacle_angle = abstraction_array[1,index_col]
            self.get_logger().info('I heard: "%s"' % obstacle_distance)
        else:
            obstacle_distance = 9099009.0
            obstacle_angle = 0.0


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