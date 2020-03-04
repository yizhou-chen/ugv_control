#!/usr/bin/env python

import numpy as np
import rospy
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_marker_msg():
    m = Marker()
    m.header.frame_id = 'world'
    m.ns = 'trajectory'
    m.type = m.LINE_STRIP
    m.action = m.ADD
    m.id = 0

    # line width
    m.scale.x = 0.02

    # line color
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 0.0

    # trajectory
    z = 0.55
    L = 1.0
    mu = 0.1

    N = 1000
    t = np.linspace(0.0, 100.0, N)

    for i in range(N):
        ref_x = L*math.cos(mu*t[i])
        ref_y = 2*L*math.sin(mu*t[i])*math.cos(mu*t[i])

        m.points.append(Point(ref_x, ref_y, z))

    return m

def main():
    rospy.init_node('reference_traj_publiser', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub_traj_viz = rospy.Publisher(
            '/visualization/trajectory',
            Marker, 
            queue_size=1
        )

    traj_marker = create_marker_msg()
    
    while not rospy.is_shutdown():
        traj_marker.header.stamp = rospy.Time.now()
        pub_traj_viz.publish(traj_marker)

if __name__ == '__main__':
    main()
