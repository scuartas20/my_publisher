import rclpy
import numpy as np
import angles
#import rospy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from math import atan2, sqrt, pow



class ControlDiffDrive(Node):

    x = 0.0
    y = 0.0
    theta = 0.0

    def __init__(self):
        global goal_x, goal_y, Kp_angular,Kp_linear, obstacle_angle_front, obstacle_distance_front, obstacle_angle_right, obstacle_distance_right, obstacle_angle_left, obstacle_distance_left

        super().__init__('control_diff_drive')
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_odom,10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_scan,10)
        self.subscription = self.create_subscription(PoseStamped,'/goal_pose',self.listener_goal,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        goal_x = 0.0
        goal_y = 0.0
        Kp_angular = 0.04
        Kp_linear = 1.5
        obstacle_distance_front = 1000000000.0
        obstacle_angle_front = 0.0    
        obstacle_distance_right = 1000000000.0
        obstacle_angle_right = 0.0    
        obstacle_distance_left = 1000000000.0
        obstacle_angle_left = 0.0    

    def listener_odom(self, msg):
        global x
        global y 
        global theta
        global Kp_angular, Kp_linear

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        r = R.from_quat([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        (theta,pitch,roll) = r.as_euler('zyx',degrees=True)

        speed = Twist()

        angle_to_goal = 0.0
        inc_x = goal_x - x
        inc_y = goal_y - y
        distance_to_goal = sqrt(pow(inc_x,2)+pow(inc_y,2))
        angle_to_goal = (atan2(inc_y,inc_x))*180/3.1415
        if distance_to_goal < 2.0:
            speed.linear.x = 0.5
        else:
            speed.linear.x = 1.5#Kp_linear*distance_to_goal
        angle_error = angles.shortest_angular_distance(theta*3.1415/180,angle_to_goal*3.1415/180)
        speed.angular.z = Kp_angular*angle_error*180/3.1415

        if obstacle_distance_front < 1.0:
            #speed.linear.x = float(cons_speed) - float((1/obstacle_distance))
            speed.linear.x = 1.5
            #Res = sqrt(pow(1,2)+pow(1,2)-(2*(1)*(1)*np.cos((3.1415+angle_to_goal-obstacle_angle_front)*180/3.1415)))
            #new_angle = np.arcsin((1)*np.sin((3.1415+angle_to_goal-obstacle_angle_front)*180/3.1415)/Res)
            #new_angle_ref = new_angle - theta*180/3.1415 - angle_to_goal*180/3.1415
            #angle_error = -180 - (obstacle_angle_front*180/3.1415)
            #angle_error = angles.shortest_angular_distance(2*theta*3.1415/180  - obstacle_angle_front -3.1415 ,theta*3.1415/180)#180 - (obstacle_angle_front*180/3.1415)
            #angle_error = angles.shortest_angular_distance(theta*3.1415/180,-obstacle_angle_front)
            #angle_error = angles.shortest_angular_distance(theta*3.1415/180 + obstacle_angle_front ,theta*3.1415/180)
            #angle_error = angles.shortest_angular_distance(theta*3.1415/180,new_angle_ref*3.1415/180)
            #if obstacle_angle_front < 0:
            angle_error = angles.shortest_angular_distance(theta*3.1415/180 - 3.1415 - (obstacle_angle_front),(theta)*3.1415/180)
            #else:
                #angle_error = angles.shortest_angular_distance(theta*3.1415/180 + 3.1415 - (obstacle_angle_front),(theta)*3.1415/180)
            Kp_distance = 1/(obstacle_distance_front*80)
            speed.angular.z = float(Kp_distance*angle_error*180/3.1415)
            self.get_logger().info('I heard: "%s"' % [int(angle_error*180/3.1415),int(theta),int(2*theta*3.1415/180  + obstacle_angle_front*3.1415/180 + 3.1415),speed.angular.z])

            # if obstacle_distance_front < 0.4:
            #     speed.linear.x = 1.0
            #     angle_error = 180 - (obstacle_angle_front*180/3.1415)
            #     speed.angular.z = float(0.015*angle_error)
            # if obstacle_distance_front < 0.05:
            #     speed.linear.x = -0.8
            #     angle_error = 180 - (obstacle_angle_front*180/3.1415) + theta*180/3.1415
            #     speed.angular.z = float(0.015*angle_error)

        # if (obstacle_distance_right < 1.0):
        #     #speed.linear.x = float(cons_speed) - float((1/obstacle_distance))
        #     speed.linear.x = 0.8
        #     angle_error = angles.shortest_angular_distance((theta*3.1415/180)+(obstacle_angle_right*3.1415/180)-3.1415/2,theta*3.1415/180,)
        #     speed.angular.z = float(0.05*angle_error*180/3.1415)

        # if (obstacle_distance_left < 1.0):
        #     #speed.linear.x = float(cons_speed) - float((1/obstacle_distance))
        #     speed.linear.x = 0.8
        #     angle_error = angles.shortest_angular_distance((theta*3.1415/180)+(obstacle_angle_left*3.1415/180)-3.1415/2,theta*3.1415/180,)
        #     speed.angular.z = float(0.05*angle_error*180/3.1415)

        if (abs(inc_x)<0.3) and (abs(inc_y)<0.3) or (obstacle_distance_front < 0.031):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            
        self.publisher_.publish(speed)
        #self.get_logger().info('I heard: "%s"' % abs(angle_to_goal - theta))
        #self.get_logger().info('I heard: "%s"' % [int(speed.angular.z),int(angle_error),int(angle_to_goal),int(theta)])
            
                


    def listener_scan(self, msg):
        #valores 16 a 20 son los necesarios para evadir obstaculos
        #Angle min -3.140000104904175
        #Increment 0.1794285774230957
        global obstacle_distance_front, obstacle_angle_front, obstacle_distance_right, obstacle_distance_left, obstacle_angle_right, obstacle_angle_left
        distances_array = np.array([msg.ranges,np.arange(-3.140000104904175,3.140000104904176,0.1794285774230957)])
        abstraction_array_front = distances_array[:,11:26]
        abstraction_array_right = distances_array[:,26:27]
        abstraction_array_left = distances_array[:,10:11]
        if min(abstraction_array_front[0,:]) < 9000000:
            index_row , index_col = np.where(abstraction_array_front == (min(abstraction_array_front[0,:])))
            obstacle_distance_front = abstraction_array_front[index_row,index_col]
            obstacle_angle_front = abstraction_array_front[1,index_col]
            #self.get_logger().info('Front: "%s"' % obstacle_distance_front)
        else:
            obstacle_distance_front = 9099009.0
            obstacle_angle_front = 0.0

        if min(abstraction_array_right[0,:]) < 9000000:
            index_row , index_col = np.where(abstraction_array_right == (min(abstraction_array_right[0,:])))
            obstacle_distance_right = abstraction_array_right[index_row,index_col]
            obstacle_angle_right = abstraction_array_right[1,index_col]
            #self.get_logger().info('Right: "%s"' % obstacle_distance_right)
        else:
            obstacle_distance_right = 9099009.0
            obstacle_angle_right = 0.0

        if min(abstraction_array_left[0,:]) < 9000000:
            index_row , index_col = np.where(abstraction_array_left == (min(abstraction_array_left[0,:])))
            obstacle_distance_left = abstraction_array_left[index_row,index_col]
            obstacle_angle_left = abstraction_array_left[1,index_col]
            #self.get_logger().info('Left: "%s"' % obstacle_distance_left)
        else:
            obstacle_distance_left = 9099009.0
            obstacle_angle_left = 0.0


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