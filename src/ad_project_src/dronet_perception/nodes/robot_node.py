#!/usr/bin/env python3
#import imp
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from math import pow, atan2, sqrt
#from Dronet import Dronet
from dronet_perception.msg import CNN_out

import numpy as np

MAX_VEL = 3.0
MAX_STR = 1.0




class Bot:

    def __init__(self):
        rospy.init_node('bot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)
        
        
        self.pose_subscriber = rospy.Subscriber('/robot_pose',
                                                Pose, self.update_pose)

        self.dronet_subscriber = rospy.Subscriber('/cnn_out/predictions',
                                                CNN_out, self.update_pred)

        self.pose = Pose()
        self.pred = CNN_out()
        self.rate = rospy.Rate(10)
        self.steering_angle_ = 0.0
        self.probability_of_collision_ = 0.0

        self.vel_desiered = MAX_VEL
        self.str_desiered = 0.0

        self.use_network_out_ = False

        # tunable parameters
        self.alpha_velocity_ = 0.3
        self.alpha_yaw_ = 0.8
        self.critical_prob_coll_ = 0.8
        self.goals = [[9.0,2.0],[10.0,3.0],[12.0,5.0],[18.0,8.0]]


    def update_pred(self, data):
        self.pred = data
        self.pred.steering_angle = self.limit_val(self.pred.steering_angle,-2.0, 2.0)
        
    def limit_val(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def update_pose(self, data):
        self.pose = data
        self.pose.position.x = round(self.pose.position.x, 4)
        self.pose.position.y = round(self.pose.position.y, 4)

    def distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
                    pow((goal_pose.position.y - self.pose.position.y), 2))


    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        
        qx = self.pose.orientation.x
        qy = self.pose.orientation.y
        qz = self.pose.orientation.z
        qw = self.pose.orientation.w
        
        ox,oy,theta = self.quaternion_to_euler_angle(qw, qx, qy, qz)
        return constant * (self.steering_angle(goal_pose) - np.radians(theta) )

    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2<-1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z 

    def move2goal(self):
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.position.x = float(input("Set your x goal: "))
        goal_pose.position.y = float(input("Set your y goal: "))
        

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        while  (self.distance(goal_pose) >= distance_tolerance):
            
           
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            
            self.vel_desiered = (1.0 -  self.pred.collision_prob) * MAX_VEL
            if(self.vel_desiered<0):
                self.vel_desiered = 0.0
            
            vel_msg.linear.x = (1.0 - self.alpha_velocity_) * vel_msg.linear.x+ self.alpha_velocity_ * self.vel_desiered

            
            
            if (self.pred.collision_prob > self.critical_prob_coll_):
                self.str_desiered = (1.0 - self.alpha_yaw_) *self.str_desiered +  self.alpha_yaw_ * self.pred.steering_angle*10
                print("Desired_Angular_Vel:  ", self.str_desiered)
                vel_msg.angular.z = self.str_desiered
            else:
                vel_msg.angular.z = self.angular_vel(goal_pose)

            
            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

        print("Goal reached...")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = Bot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass