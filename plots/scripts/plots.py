#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from squaternion import quat2euler
from plot_functions import print_covariance_data


i = 0
l = []
c_xx = []
c_yy = []
c_tt = []
x = []
y = []
theta = []


def amcl_pose_callback(msg):
    global i, x, y, theta, c_xx, c_yy, c_tt
    #global l, matrix
    l = msg.pose.covariance

    c_xx.append(l[0])
    c_yy.append(l[7])
    c_tt.append(l[35])
    x.append(msg.pose.pose.position.x)
    y.append(msg.pose.pose.position.y)
    pt = quat2euler(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, degrees=True)
    theta.append(pt[2])
    print i
    if i == 30:
        print_covariance_data(x, y, theta, c_xx, c_yy, c_tt)
    i += 1

    
        


        #header = "x, y, theta, c_xx, c_yy, c_tt, c_xy, c_yx"
        #data = np.column_stack((c_xx, c_yy, c_tt, c_xy, c_yx))
        #np.savetxt("data2.dat", data, header=header)




if __name__ == "__main__":

    rospy.init_node("plot_tests")

    sub_amcl = rospy.Subscriber("/robot/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

        

    rospy.Rate(10)

    rospy.spin()