#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters as mf
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import Twist, PoseArray, Point
from visualization_msgs.msg import MarkerArray
import nav_msgs.msg
import tf 
from tf.transformations import euler_from_quaternion
import math
import copy

class PID_velocity():  

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

    
    def update(self,measured_x,measured_y,reference_x,reference_y):
       
        self.measured_x = measured_x
        self.reference_x = reference_x
        self.measured_y = measured_y
        self.reference_y = reference_y

        #calculate error
        self.error = round(math.sqrt(pow(reference_x-measured_x,2)+pow(reference_y-measured_y,2)),2)
        
        
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
   
        if self.u < 0.07:
            self.u = 0

        #vrati upravljacku velicinu
        return self.u  



class SimpleController(object):

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Initialize class variables.
        #init_vel_x = rospy.get_param("~init_vel_x", 0)
        #init_vel_y = rospy.get_param("~init_vel_y", 0)
        self.frequency = 10
        #wait_count = int(rospy.get_param("/wait_time") * frequency)
        #start_count = int(rospy.get_param("/start_time") * frequency)
        self.run_type = rospy.get_param("/run_type")
        
        #self.target_x = -9
        #self.target_y = 8
        #self.init_x = 2.331
        #self.init_y = 3.586
        self.max_velocity = 0.5
        self.max_acceleration = 0.25
        self.Kp_orientation = 1

        self.prosli_izlazni_v = 0
        self.prosli_pid_v = 0

        self.Kp_velocity = 0.3
        self.Ki_velocity = 0.01
        self.Ts = 0.1
        self.initialization = 0
        self.start = 0
        self.brojac = 0

        #self.orientation_control = PID(self.Kp_orientation,0,0,1/self.frequency,math.pi,-math.pi)
        self.velocity_control = PID_velocity(self.Kp_velocity,self.Ki_velocity,0,self.Ts,0,0.5/1.168)

        #subscriber za reference
        self.set_point_reference = rospy.Subscriber('sphero_referentna_tocka',Point,self.point_reference_callback)

        # Create a publisher for commands.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=self.frequency)


    def point_reference_callback(self,data):

        self.target_x = data.x
        self.target_y = data.y
        self.start = 1

    def petlja(self):

        while self.start == 0:            #ceka se da stigne prva referenca

            if self.brojac%100000000 == 0:           #brojac da se ne spama poruka previse
                print("cekam prvu referencu")
            self.brojac = self.brojac + 1

        if self.start == 1:
            print("prva referenca stigla")

        while not rospy.is_shutdown():
            
            self.trenutni_info = rospy.wait_for_message('/odom',nav_msgs.msg.Odometry)
            self.euler_data = euler_from_quaternion([self.trenutni_info.pose.pose.orientation.x,self.trenutni_info.pose.pose.orientation.y,self.trenutni_info.pose.pose.orientation.z,self.trenutni_info.pose.pose.orientation.w])


            self.target_orientation = math.atan2(self.target_y-self.trenutni_info.pose.pose.position.y,self.target_x-self.trenutni_info.pose.pose.position.x)
            

            self.orientation_error  = self.target_orientation - round(self.euler_data[2],4)

            if self.run_type == 'sim':
                
                self.cmd_vel = Twist()

                self.velocity_pid = self.velocity_control.update(self.trenutni_info.pose.pose.position.x,self.trenutni_info.pose.pose.position.y,self.target_x,self.target_y)

                #print(self.velocity_pid)

                self.cmd_vel.linear.x = 0.8012 * self.prosli_izlazni_v + 0.1161 * self.velocity_pid + 0.1161 * self.prosli_pid_v   #prijelazna funkcija procesa

                self.cmd_vel.linear.y = int(0)
                
                self.cmd_vel.angular.z = self.orientation_error * self.Kp_orientation
                    
                self.cmd_vel_pub.publish(self.cmd_vel)

                self.prosli_izlazni_v = copy.copy(self.cmd_vel.linear.x)   #spremanje vrijednosti za slijedeci korak
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