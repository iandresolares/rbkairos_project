#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from squaternion import quat2euler
from plot_functions import print_pointcloud_data
x = []
particles = []
y = []
t = []
i = -1
n = 0
theta = []

def amcl_particles_callback(msg):
    global particles, y, theta, x, n, t
    k = 0
    particles = msg.poses
    n = len(particles)

    if i == 0 or i == 6 or i == 12 or i == 18 or i == 24 or i == 30:
        for k in range(n):
            x.append(particles[k].position.x)
            y.append(particles[k].position.y)
            t = quat2euler(particles[k].orientation.w, particles[k].orientation.x, particles[k].orientation.y, particles[k].orientation.z, degrees=True)
            theta.append(t[2])
        print_pointcloud_data(x, y, theta, i)
        x = []
        y = []
        theta = []



def amcl_pose_callback(msg):
    global i
    i += 1
    print i


    # global i

    # i = i + 1
    # print i
    # if i == 18:
    #     print("printing done")
    #     with open("covariance_4_mas_angulos.csv", "w") as f:
    #         for j in range(len(x)):
    #             out_string = ""
    #             out_string += str(x[j])
    #             out_string += "," + str(y[j])
    #             out_string += "," + str(theta[j])
    #             out_string += "," + str(c_xx[j])
    #             out_string += "," + str(c_yy[j])
    #             out_string += "," + str(c_tt[j])
    #             out_string += "," + str(c_xy[j])
    #             out_string += "," + str(c_yx[j])
    #             out_string += "\n"
    #             f.write(out_string)


        #header = "x, y, theta, c_xx, c_yy, c_tt, c_xy, c_yx"
        #data = np.column_stack((c_xx, c_yy, c_tt, c_xy, c_yx))
        #np.savetxt("data2.dat", data, header=header)




if __name__ == "__main__":

    rospy.init_node("plot_particles")

    sub_amcl = rospy.Subscriber("/robot/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    sub_particles = rospy.Subscriber("/robot/particlecloud", PoseArray, amcl_particles_callback)

        

    rospy.Rate(10)

    rospy.spin()