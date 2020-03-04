#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

position_msg = PoseStamped()
velocity_publisher = None
PI = 3.1415926535897

def callback(msg):
    position_msg = msg

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def get_msg_v_cmd(t):  
    current_positon_x = position_msg.pose.position.x
    current_positon_y = position_msg.pose.position.y
    print(current_positon_x)
    print(position_msg)

    # Get the yaw from quaternions
    x = position_msg.pose.orientation.x
    y = position_msg.pose.orientation.y
    z = position_msg.pose.orientation.z
    w = position_msg.pose.orientation.w
    #t1 = +2.0 * (qua_w * qua_z + qua_x * qua_y)
    #t2 = +1.0 - 2.0 * (qua_y * qua_y + qua_z * qua_z)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    
    print(x)
    print(y)
    print(z)
    print(w)
    # Define a reference trajectory
    ref_x = math.sin(t)
    ref_y = math.cos(t)

    x_dot = math.cos(t)
    y_dot = -math.sin(t)

    ref_w = 1
    ref_v = 1
    ref_theta = math.atan2(y_dot,x_dot)

    x_error = ref_x - current_positon_x
    y_error = ref_y - current_positon_y
    theta_error = ref_theta - yaw

    error_x = (ref_x - current_positon_x) * math.cos(yaw) + (ref_y - current_positon_y) * math.sin(yaw)
    error_y = -(ref_x - current_positon_x) * math.sin(yaw) +(ref_y -current_positon_y) * math.cos(yaw)
    error_theta = theta_error
    c1 = 0.5
    c2 = 10
    c3 = 5
    print('e_x=%f'%(x_error))
    print('e_y=%f'%(y_error))
    print('e_theta=%f'%(theta_error))
    

    # Input control rule
    w = ref_w + 100 * ref_v * error_y * (math.sin(error_theta)/error_theta) + 0.5 * error_theta
    v = ref_v * math.cos(error_theta) + 0.4 * error_theta
    #w = ref_w + c2 * ref_v * (error_y*math.cos(error_theta/2)-error_x*math.sin(error_theta/2)) / math.sqrt(1+math.pow(error_x,2)+math.pow(error_y,2)) + c3 * math.sin(error_theta/2)
    #v = ref_v + c1 * error_x / math.sqrt(1 + math.pow(error_x,2) + math.pow(error_y,2))
    # Publish the topic
    
    vel_msg = Twist()
    vel_msg.linear.x = 0.0 #abs(v)
    vel_msg.angular.z = 0.0#abs(w)
    return vel_msg


if __name__ == '__main__':
    # Start new node
    rospy.init_node('coordinates_listener', anonymous=True)
    rospy.Subscriber('vrpn_client_node/leo/pose', PoseStamped,callback)
    
    # Spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(30)
    velocity_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    #error_publisher = rospy.Publisher('/error_info', float, queue_size = 10)
    t1 = rospy.get_time()
    while not rospy.is_shutdown():
        t2 = rospy.get_time()
        t = t2 - t1
        #print (t)
        vel_msg = get_msg_v_cmd(t)
        velocity_publisher.publish(vel_msg)
        
        rate.sleep()
