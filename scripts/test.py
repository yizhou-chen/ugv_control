#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
PI = 3.1415926535897
class UGVControl:
    def __init__(self):
        rospy.init_node('coordinates_listener', anonymous=True)
        self.rate = rospy.Rate(60)
        self.pub_vel = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        self.sub_pose = rospy.Subscriber('vrpn_client_node/leo/pose', PoseStamped,self.callback)
        self.position_msg = PoseStamped()
        
    def callback(self, msg):
        self.position_msg = msg
    def run(self):
        t1 = rospy.get_time()
        while not rospy.is_shutdown():
            t2 = rospy.get_time()
            t = t2 - t1
            if t >= 3:
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0 
                
            else:
                vel_msg = Twist()
                vel_msg.linear.x = 10
                vel_msg.angular.z = 0            
            
            self.pub_vel.publish(vel_msg)
            print(t)
            self.rate.sleep()

        
        # vel_msg = Twist()
        # vel_msg.linear.x = 0
        # vel_msg.angular.z = 1
        # self.pub_vel.publish(vel_msg)
        # rospy.sleep(10.)
        # current_positon_x = self.position_msg.pose.position.x
        # current_positon_y = self.position_msg.pose.position.y
        # #print(current_positon_x)
        # # print(self.position_msg)

        # # Get the yaw from quaternions
        # q_x = self.position_msg.pose.orientation.x
        # q_y = self.position_msg.pose.orientation.y
        # q_z = self.position_msg.pose.orientation.z
        # q_w = self.position_msg.pose.orientation.w
        # t0 = +2.0 * (q_w * q_x + q_y * q_z)
        # t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        # roll = math.atan2(t0, t1)
        
        # t2 = +2.0 * (q_w * q_y - q_z * q_x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch = math.asin(t2)
        
        # t3 = +2.0 * (q_w * q_z + q_x * q_y)
        # t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        # yaw = math.atan2(t3, t4)

        # print('yaw=%f'%(yaw*180/PI))
        # print(current_positon_x)
        # # print('real w=%f'%(yaw/t))
        
        # # vel_msg.linear.x = 0
        # # vel_msg.angular.z = 1
        # # self.pub_vel.publish(vel_msg)
if __name__ == '__main__':
    my_ugv = UGVControl()
    my_ugv.run()
