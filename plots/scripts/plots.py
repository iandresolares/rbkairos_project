#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


l = []
lt = []
i = 0

def amcl_pose_callback(msg):
    global i
    l.append(msg.pose.covariance[1])
    lt.append(i)
    i += 1
    print l
    print lt
    #print msg.pose.covariance

if __name__ == "__main__":

    rospy.init_node("plot_tests")

    sub_amcl = rospy.Subscriber("/robot/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    rospy.Rate(1)

    plt.plot(lt, l)
    plt.show(block=True)

    #rospy.spin()