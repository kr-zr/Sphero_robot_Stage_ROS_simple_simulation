#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters as mf
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import Twist, PoseArray, Pose
from visualization_msgs.msg import MarkerArray
import nav_msgs.msg
import tf 
from tf.transformations import euler_from_quaternion
import math
from timeit import default_timer as timer
import copy


class PID():  

    def __init__(self, K_p, K_i, K_d, T, min_val, max_val):
        # pojacanja
        self.Kp = K_p
        self.Kd = K_d
        self.Ki = K_i
        
        self.error = 0
        self.reference = 0
        self.prop = 0
        self.derivative = 0
        self.integral = 0
        self.N = 100

        self.u = 0
        # vrijednosti iz koraka k-1
        self.Derivator_prior = 0
        self.Integral_prior  = 0
        self.error_prior     = 0
        # vrijeme diskretizacije
        self.T = T
        # maks i min brzina
        self.max_val = max_val
        self.min_val = min_val

    
    def update(self,measured,reference):
       
        self.measured = measured
        self.reference = reference

        #calculate error
        self.error =round(self.reference - self.measured,5)
        
        
        #PID
        self.prop = self.Kp * self.error
        #derivator Tustin
        
      
        self.derivative = ((2 - self.N * self.T) * self.Derivator_prior + 2 * self.Kd * self.N * self.error - 2 * self.Kd * self.N * self.error_prior)/(2 + self.N * self.T)

        #integrator Tustin
        self.integral = self.Integral_prior + (self.Ki * self.T)/2 * self.error + (self.Ki * self.T)/2 * self.error_prior
        
        #upravljacka velicina
        self.u = self.prop + self.derivative + self.integral


        if(self.u > self.max_val):  
            self.u = self.max_val
            self.integral = self.Integral_prior 

        #negative saturation
        elif (self.u < self.min_val):
            self.u = self.min_val
            self.integral = self.Integral_prior

        
        self.Integral_prior = copy.copy(self.integral)
        self.Derivator_prior = copy.copy(self.derivative)
        self.error_prior = copy.copy(self.error)
   

        #vrati upravljacku velicinu
        return self.u  


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
        self.target_velocity = 0
        self.max_velocity = 0.5
        self.max_acceleration = 0.25
        self.target_orientation = 0  #od -pi do pi, orijentacija 0 gleda u +x smjeru
        self.Kp_orientation = 1
        self.Kp_velocity = 0.6
        self.Ki_velocity = 5
        self.Ts = 0.1

        self.prosli_izlazni_v = 0
        self.prosli_pid_v = 0

        self.velocity_control = PID(self.Kp_velocity,self.Ki_velocity,0,self.Ts,0,0.5/1.168)

        #subscriber za reference
        self.set_reference = rospy.Subscriber("sphero_referenca",Twist,self.reference_callback)

        # Create a publisher for commands.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=frequency)

        if self.target_velocity > self.max_velocity:
            
            self.target_velocity = self.max_velocity


    def reference_callback(self,data):

        if data.linear.x >= 0 and data.linear.x <=0.5: 
            self.target_velocity = data.linear.x
        if data.angular.z >= -math.pi and data.angular.z <= math.pi:  #malo cudno zadavati orijentaciju preko brzine, ali ne znam koji topic da slozim da bi orijentaciju zadao u radijanima
            self.target_orientation = data.angular.z

    def petlja(self):
        
        # Keep program from exiting

        while not rospy.is_shutdown():
            
            self.trenutni_info = rospy.wait_for_message('/odom',nav_msgs.msg.Odometry)
            #print(timer())
            #if self.flag == 0:
             #   self.start = timer()
            #if self.flag == 1:
                #self.stop = timer()
               # self.flag = self.flag + 1
              #  print(self.stop-self.start)
            self.euler_data = euler_from_quaternion([self.trenutni_info.pose.pose.orientation.x,self.trenutni_info.pose.pose.orientation.y,self.trenutni_info.pose.pose.orientation.z,self.trenutni_info.pose.pose.orientation.w])
            
            self.orientation_error  = self.target_orientation - round(self.euler_data[2],4)
            
            if self.run_type == 'sim':
                
                self.cmd_vel = Twist()

                self.velocity_pid = self.velocity_control.update(self.trenutni_info.twist.twist.linear.x,self.target_velocity)

                #print(self.velocity_pid)

                self.cmd_vel.linear.x = 0.8012 * self.prosli_izlazni_v + 0.1161 * self.velocity_pid + 0.1161 * self.prosli_pid_v   #prijelazna funkcija procesa

                self.cmd_vel.linear.y = int(0)
                
                self.cmd_vel.angular.z = self.orientation_error * self.Kp_orientation
                    
                self.cmd_vel_pub.publish(self.cmd_vel)

                #self.flag = self.flag + 1

                self.prosli_izlazni_v = copy.copy(self.cmd_vel.linear.x)    #spremanje proslih vrijednosti
                self.prosli_pid_v = copy.copy(self.velocity_pid)

            
    


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('SimpleController')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        rc = SimpleController()
        rc.petlja()
    except rospy.ROSInterruptException:
        pass