#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters as mf
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import Twist, PoseArray
from visualization_msgs.msg import MarkerArray
import nav_msgs.msg
import tf 
from tf.transformations import euler_from_quaternion
import math
from timeit import default_timer as timer


class SimpleController(object):

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Initialize class variables.
        #init_vel_x = rospy.get_param("~init_vel_x", 0)
        #init_vel_y = rospy.get_param("~init_vel_y", 0)
        #frequency = rospy.get_param("/ctrl_loop_freq")
        frequency = 10
        #wait_count = int(rospy.get_param("/wait_time") * frequency)
        #start_count = int(rospy.get_param("/start_time") * frequency)
        self.run_type = rospy.get_param("/run_type")
        
        self.flag = 0
        self.target_velocity = 0.4
        self.max_velocity = 0.5
        self.max_acceleration = 0.25
        self.target_orientation = math.pi/4  #od -pi d0 pi, orijentacija 0 gleda u +x smjeru
        self.Kp_orientation = 1
        self.Kp_velocity = self.max_acceleration/(frequency*self.max_velocity)

        # Create a publisher for commands.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=frequency)

        if self.target_velocity > self.max_velocity:
            
            self.target_velocity = self.max_velocity

        # Keep program from exiting
        while not rospy.is_shutdown():
            
            self.trenutni_info = rospy.wait_for_message('/odom',nav_msgs.msg.Odometry)
            #print(timer())
            if self.flag == 0:
                self.start = timer()
            if self.flag == 1:
                self.stop = timer()
                self.flag = self.flag + 1
                print(self.stop-self.start)
            self.euler_data = euler_from_quaternion([self.trenutni_info.pose.pose.orientation.x,self.trenutni_info.pose.pose.orientation.y,self.trenutni_info.pose.pose.orientation.z,self.trenutni_info.pose.pose.orientation.w])
            
            self.orientation_error  = self.target_orientation - round(self.euler_data[2],4)
            self.velocity_error = self.target_velocity - self.trenutni_info.twist.twist.linear.x
            
            if self.run_type == 'sim':
                
                self.cmd_vel = Twist()

                self.cmd_vel.linear.x = self.trenutni_info.twist.twist.linear.x + self.velocity_error * self.Kp_velocity

                if self.cmd_vel.linear.x > self.max_velocity:

                    self.cmd_vel.linear.x = self.max_velocity

                self.cmd_vel.linear.y = int(0)
                
                self.cmd_vel.angular.z = self.orientation_error * self.Kp_orientation
                    
                self.cmd_vel_pub.publish(self.cmd_vel)

                self.flag = self.flag + 1

            
    


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('SimpleController')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        rc = SimpleController()
    except rospy.ROSInterruptException:
        pass