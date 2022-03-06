#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import numpy as np
import time

###########################################################################################################################################

class micro_mouse:
    ########################################################################
    # r--> Radius of wheel used in differential drive bot
    # l--> Distance of separation of wheels

    # w_l --> Angular velocity of left  wheel in rads per second
    # w_r --> Angular velocity of right wheel in rads per second

    # v_max --> Maximum linear velocity of dd_bot
    # w_max --> Maximum angular velocity of dd_bot

    # p_v --> P gain for linear velocity controller
    # p_w --> P gain for angular velocity controller

    r = 0.01225
    l = 0.0406
    w_l = 0
    w_r = 0

    goal_x = 0
    goal_y = 0

    curr_x = 0
    curr_y = 0
    curr_theta = 0

    p_v = 0.8
    p_w = 2

    v_max = 0.5
    w_max = 1
    ########################################################################

    def dd_callback(self,data):
        '''
        Subscriber callback function
        '''
        self.curr_x = data.pose[3].position.x
        self.curr_y = data.pose[3].position.y
        x = data.pose[3].orientation.x
        y = data.pose[3].orientation.y
        z = data.pose[3].orientation.z
        w = data.pose[3].orientation.w
        [roll,pitch,yaw] = euler_from_quaternion([x,y,z,w])
        self.curr_theta = yaw
        #print([self.curr_x,self.curr_y,self.curr_theta])

    def __init__(self):
        rospy.init_node('micro_mouse_controller', anonymous=False)
        self.rate = rospy.Rate(10)
        self.left_wheel_pub  = rospy.Publisher('/my_mm_robot/base_to_first_joint_position_controller/command', Float64, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/my_mm_robot/base_to_second_joint_position_controller/command', Float64, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.dd_callback)
        self.cmd_left  = Float64()
        self.cmd_right = Float64()
        self.dd_map(0.04,1)
        

    def dd_map(self,v,w):
        '''
        Differential drive map function:

        Maps the linear and angular velocity of the dd_bot to
        angular velocity of left and right wheel.
        
        Radius (r) and distance of separation of wheels(l) are
        design parameters of differential drive bot
        '''
        self.w_l = (2*v + w*self.l)/(2*self.r)
        self.w_r = (2*v - w*self.l)/(2*self.r)
        self.cmd_left.data  = self.w_l
        self.cmd_right.data = self.w_r

    def dd_cmd(self):
        ''' 
        Code to publish angular velocities of left and right wheels
        '''
        self.left_wheel_pub.publish(self.cmd_left)
        self.right_wheel_pub.publish(self.cmd_right)
        self.rate.sleep()

    def dd_controller(self):
        ''' 
        simple p controller to move dd_robot from 
        current point to a given point
        '''
        
        while(np.linalg.norm([self.curr_x-self.goal_x,self.curr_y-self.goal_y])>0.02):
            del_x = self.goal_x-self.curr_x
            del_y = self.goal_y-self.curr_y
            curr_theta = np.arctan2(del_y,del_x)
            print("theta =",curr_theta)

            del_theta = -curr_theta + self.curr_theta
            print("del_theta = ",del_theta)
            if(del_theta>0 and del_theta>np.pi):
                del_theta = del_theta-2*np.pi
            elif(del_theta<0 and del_theta<-np.pi):
                del_theta = 2*np.pi + del_theta
            else:
                pass

            del_d = np.linalg.norm([del_x,del_y])
            v = self.p_v*del_d
            w = self.p_w*del_theta
            if(v>self.v_max):
                v = self.v_max
            if(w>self.w_max):
                w = self.w_max
            if(abs(del_theta)>np.pi/36):
                v = 0
            else:
                pass
            print("velocity =" ,v)	
            self.dd_map(v,w)
            self.dd_cmd()
        
        v = 0
        w = 0
        self.dd_map(v,w)
        self.dd_cmd()


###########################################################################################################################################

if __name__ == '__main__':
    try:
        points = np.load('/home/charan/catkin_ws/src/micro_mouse/scripts/shortest_path.npy')
        mm = micro_mouse()
        print("goal_class")
        rospy.sleep(1)
        for i in range(np.shape(points)[0]):
            mm.goal_x = points[i][0]
            mm.goal_y = points[i][1]
            print("goal_points")
            mm.dd_controller()
            
            print("Reached point  :",i+1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
