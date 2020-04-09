#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
PI = 3.1415926535897
class UGVControl:
    def __init__(self):
        rospy.init_node('coordinates_listener', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pub_vel1 = rospy.Publisher('/pioneer1/pioneer/cmd_vel', Twist, queue_size=10)
        self.pub_vel2 = rospy.Publisher('/pioneer2/pioneer/cmd_vel', Twist, queue_size=10)
        self.pub_vel3 = rospy.Publisher('/pioneer3/pioneer/cmd_vel', Twist, queue_size=10)
        self.sub_pose1 = rospy.Subscriber('/pioneer1/odom', Odometry, self.callback1)
        self.sub_pose2 = rospy.Subscriber('/pioneer2/odom', Odometry, self.callback2)
        self.sub_pose3 = rospy.Subscriber('/pioneer3/odom', Odometry, self.callback3)
        self.position_msg1 = Odometry()
        self.position_msg2 = Odometry()
        self.position_msg3 = Odometry()
        self.dist_ob1 = rospy.Subscriber('/pioneer1/base_scan', LaserScan, self.callback_ob1)
        self.dist_ob2 = rospy.Subscriber('/pioneer2/base_scan', LaserScan, self.callback_ob2)
        self.dist_ob3 = rospy.Subscriber('/pioneer3/base_scan', LaserScan, self.callback_ob3)
        self.dist_msg1 = LaserScan()
        self.dist_msg2 = LaserScan()
        self.dist_msg3 = LaserScan()
        #initial VC
        self.mu=float()
        self.vc_x = float()
        self.vc_y = float()
        self.vc_x_dot = float()
        self.vc_y_dot = float()
        self.vc_x_2dot = float()
        self.vc_y_2dot = float()
        self.vc_ref_theta = math.atan2(self.vc_y_dot,self.vc_x_dot)

    def run(self): 
        t1 = rospy.get_time()
        while not rospy.is_shutdown():
            t2 = rospy.get_time()
            t = t2 - t1
            self.get_vc(t)
            minimum_distance1 = self.get_msg_dist1()
            minimum_distance2 = self.get_msg_dist2()
            minimum_distance3 = self.get_msg_dist3()
            if minimum_distance1 <= 0.4 and minimum_distance1 != None:
                vel_msg1 = self.obstacle_avoid(minimum_distance1)
                print('pioneer1 obs')
            else:
                vel_msg1 = self.get_msg_v_cmd1(t)   
                # print('tracking')  
            if minimum_distance2 <= 0.4 and minimum_distance2 != None:
                vel_msg2 = self.obstacle_avoid(minimum_distance2)
                print('pioneer2 obs')
            else:
                vel_msg2 = self.get_msg_v_cmd2(t)   
                # print('tracking')  
            if minimum_distance3 <= 0.4 and minimum_distance3 != None:
                vel_msg3 = self.obstacle_avoid(minimum_distance3)
                print('pioneer3 obs')
            else:
                vel_msg3 = self.get_msg_v_cmd3(t)   
                # print('tracking')
            self.pub_vel1.publish(vel_msg1)
            self.pub_vel2.publish(vel_msg2)
            self.pub_vel3.publish(vel_msg3)
            self.rate.sleep()

    def callback1(self, msg):
        self.position_msg1 = msg
    def callback2(self, msg):
        self.position_msg2 = msg
    def callback3(self, msg):
        self.position_msg3 = msg
    def callback_ob1(self,msg):
        self.dist_msg1 = msg
    def callback_ob2(self,msg):
        self.dist_msg2 = msg
    def callback_ob3(self,msg):
        self.dist_msg3 = msg

    def get_vc(self,t):
        self.mu=1
        self.vc_x = 0.3*t
        self.vc_y = 0.5*math.sin(self.mu*self.vc_x)
        self.vc_x_dot = 0.3
        self.vc_y_dot = 0.5*self.mu*self.vc_x_dot*math.cos(self.mu*self.vc_x)
        self.vc_x_2dot = 0
        self.vc_y_2dot = -0.5*self.mu*self.vc_x_dot*self.mu*self.vc_x_dot*math.sin(self.mu*self.vc_x)
        self.vc_ref_theta = math.atan2(self.vc_y_dot,self.vc_x_dot)
        return

    def get_msg_dist1(self):
        current_distance = self.dist_msg1.ranges
        minimum_dist = None
        try:
            minimum_dist = min(current_distance)
        except ValueError:
            print('empty')
        return minimum_dist

    def get_msg_dist2(self):
        current_distance = self.dist_msg2.ranges
        minimum_dist = None
        try:
            minimum_dist = min(current_distance)
        except ValueError:
            print('empty')
        return minimum_dist

    def get_msg_dist3(self):
        current_distance = self.dist_msg3.ranges
        minimum_dist = None
        try:
            minimum_dist = min(current_distance)
        except ValueError:
            print('empty')
        return minimum_dist

    def obstacle_avoid(self,minimum_distance):
        gamma = 0.5
        w = -3*math.tanh(gamma*(minimum_distance))
        v = 0.07
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        return vel_msg

        #pioneer 1
    def get_msg_v_cmd1(self,t):  
        current_positon_x = self.position_msg1.pose.pose.position.x
        current_positon_y = self.position_msg1.pose.pose.position.y

        # Get the yaw from quaternions  
        q_x = self.position_msg1.pose.pose.orientation.x
        q_y = self.position_msg1.pose.pose.orientation.y
        q_z = self.position_msg1.pose.pose.orientation.z
        q_w = self.position_msg1.pose.pose.orientation.w
        t0 = +2.0 * (q_w * q_x + q_y * q_z)
        t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (q_w * q_y - q_z * q_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(t3, t4)
        
        l_x1 = 0
        l_y1 = 0
        ref_x = self.vc_x+l_x1*math.cos(self.vc_ref_theta)-l_y1*math.sin(self.vc_ref_theta)
        ref_y = self.vc_y+l_x1*math.sin(self.vc_ref_theta)+l_y1*math.cos(self.vc_ref_theta)
        x_dot = 0.3
        y_dot = 0.5*self.mu*x_dot*math.cos(self.mu*ref_x)
        x_2dot = 0
        y_2dot = -0.5*self.mu*x_dot*self.mu*x_dot*math.sin(self.mu*ref_x)

        ref_w = (x_dot*y_2dot-x_2dot*y_dot)/(math.pow(x_dot,2)+math.pow(y_dot,2))
        ref_v = math.sqrt(math.pow(x_dot,2)+math.pow(y_dot,2))
        ref_theta = math.atan2(y_dot,x_dot)

        x_error = ref_x - current_positon_x
        y_error = ref_y - current_positon_y
        theta_error = ref_theta - yaw

        error_x = (ref_x - current_positon_x) * math.cos(yaw) + (ref_y - current_positon_y) * math.sin(yaw)
        error_y = -(ref_x - current_positon_x) * math.sin(yaw) +(ref_y -current_positon_y) * math.cos(yaw)
        error_theta = theta_error
        
        print('e_x=%f'%(x_error))
        print('e_y=%f'%(y_error))
        print('e_theta=%f'%(theta_error))

        # Input control rule
        k1 = 0.4
        k2 = 2
        k3 = 0.5
        v = ref_v * math.cos(error_theta) + k1 * error_x
        w = ref_w + k2 * ref_v * error_y * (math.sin(error_theta)/error_theta) + k3 * error_theta

        # Publish the topic       
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = -w
        return vel_msg



    #     #pioneer 2
    def get_msg_v_cmd2(self,t):  
        current_positon_x = self.position_msg2.pose.pose.position.x
        current_positon_y = self.position_msg2.pose.pose.position.y

        # Get the yaw from quaternions
        q_x = self.position_msg2.pose.pose.orientation.x
        q_y = self.position_msg2.pose.pose.orientation.y
        q_z = self.position_msg2.pose.pose.orientation.z
        q_w = self.position_msg2.pose.pose.orientation.w
        t0 = +2.0 * (q_w * q_x + q_y * q_z)
        t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (q_w * q_y - q_z * q_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(t3, t4)

        
        l_x2 = 0
        l_y2 = 1
        ref_x = self.vc_x+l_x2*math.cos(self.vc_ref_theta)-l_y2*math.sin(self.vc_ref_theta)
        ref_y = self.vc_y+l_x2*math.sin(self.vc_ref_theta)+l_y2*math.cos(self.vc_ref_theta)
        x_dot = 0.3
        y_dot = 0.5*self.mu*x_dot*math.cos(self.mu*ref_x)
        x_2dot = 0
        y_2dot = -0.5*self.mu*x_dot*self.mu*x_dot*math.sin(self.mu*ref_x)


        ref_w = (x_dot*y_2dot-x_2dot*y_dot)/(math.pow(x_dot,2)+math.pow(y_dot,2))
        ref_v = math.sqrt(math.pow(x_dot,2)+math.pow(y_dot,2))
        ref_theta = math.atan2(y_dot,x_dot)
        x_error = ref_x - current_positon_x
        y_error = ref_y - current_positon_y
        theta_error = ref_theta - yaw

        error_x = (ref_x - current_positon_x) * math.cos(yaw) + (ref_y - current_positon_y) * math.sin(yaw)
        error_y = -(ref_x - current_positon_x) * math.sin(yaw) +(ref_y -current_positon_y) * math.cos(yaw)
        error_theta = theta_error
        
        # print('pioneer2 e_x=%f'%(x_error))
        # print('pioneer2 e_y=%f'%(y_error))
        # print('pioneer2 e_theta=%f'%(theta_error))


        # Input control rule
        k1 = 0.4
        k2 = 2
        k3 = 0.5
        v = ref_v * math.cos(error_theta) + k1 * error_x
        w = ref_w + k2 * ref_v * error_y * (math.sin(error_theta)/error_theta) + k3 * error_theta



        # Publish the topic
        
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = -w
        
        return vel_msg
      #pioneer 3
    def get_msg_v_cmd3(self,t):  
        current_positon_x = self.position_msg3.pose.pose.position.x
        current_positon_y = self.position_msg3.pose.pose.position.y

       

        # Get the yaw from quaternions
        q_x = self.position_msg3.pose.pose.orientation.x
        q_y = self.position_msg3.pose.pose.orientation.y
        q_z = self.position_msg3.pose.pose.orientation.z
        q_w = self.position_msg3.pose.pose.orientation.w
        t0 = +2.0 * (q_w * q_x + q_y * q_z)
        t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (q_w * q_y - q_z * q_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(t3, t4)

        
        l_x3 = 0
        l_y3 = -1
        ref_x = self.vc_x+l_x3*math.cos(self.vc_ref_theta)-l_y3*math.sin(self.vc_ref_theta)
        ref_y = self.vc_y+l_x3*math.sin(self.vc_ref_theta)+l_y3*math.cos(self.vc_ref_theta)
        x_dot = 0.3
        y_dot = 0.5*self.mu*x_dot*math.cos(self.mu*ref_x)
        x_2dot = 0
        y_2dot = -0.5*self.mu*x_dot*self.mu*x_dot*math.sin(self.mu*ref_x)


        ref_w = (x_dot*y_2dot-x_2dot*y_dot)/(math.pow(x_dot,2)+math.pow(y_dot,2))
        ref_v = math.sqrt(math.pow(x_dot,2)+math.pow(y_dot,2))
        ref_theta = math.atan2(y_dot,x_dot)
        x_error = ref_x - current_positon_x
        y_error = ref_y - current_positon_y
        theta_error = ref_theta - yaw

        error_x = (ref_x - current_positon_x) * math.cos(yaw) + (ref_y - current_positon_y) * math.sin(yaw)
        error_y = -(ref_x - current_positon_x) * math.sin(yaw) +(ref_y -current_positon_y) * math.cos(yaw)
        error_theta = theta_error
        
        print('pioneer3 e_x=%f'%(x_error))
        print('pioneer3 e_y=%f'%(y_error))
        print('pioneer3 e_theta=%f'%(theta_error))


        # Input control rule
        k1 = 0.4
        k2 = 2
        k3 = 0.5
        v = ref_v * math.cos(error_theta) + k1 * error_x
        w = ref_w + k2 * ref_v * error_y * (math.sin(error_theta)/error_theta) + k3 * error_theta



        # Publish the topic
        
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = -w
        
        return vel_msg


if __name__ == '__main__':
    my_ugv = UGVControl()
    my_ugv.run()
        # #sin
        # #define VC
        # mu=1
        # vc_x = 0.3*t
        # vc_y = 0.5*math.sin(mu*vc_x)
        # vc_x_dot = 0.3
        # vc_y_dot = 0.5*mu*vc_x_dot*math.cos(mu*vc_x)
        # vc_x_2dot = 0
        # vc_y_2dot = -0.5*mu*vc_x_dot*mu*vc_x_dot*math.sin(mu*vc_x)
        
        # vc_ref_w = (vc_x_dot*vc_y_2dot-vc_x_2dot*vc_y_dot)/(math.pow(vc_x_dot,2)+math.pow(vc_y_dot,2))
        # vc_ref_v = math.sqrt(math.pow(vc_x_dot,2)+math.pow(vc_y_dot,2))
        # vc_ref_theta = math.atan2(vc_y_dot,vc_x_dot)