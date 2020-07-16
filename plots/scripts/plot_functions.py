#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from squaternion import quat2euler


def print_pointcloud_data(x, y, theta, data_index):

    with open("pointcloud_data_prueba_reset_" + str(data_index) + ".csv", "w") as f:
        for j in range(len(x)):
                if x[j] < 0:
                    data_x = str(x[j])
                    data_x = data_x[:8]
                else:
                    data_x = str(x[j])
                    data_x = data_x[:7]
                
                if y[j] < 0:
                    data_y = str(y[j])
                    data_y = data_y[:8]
                else:
                    data_y = str(y[j])
                    data_y = data_y[:7]
                
                if theta[j] < 0:
                    data_t = str(theta[j])
                    data_t = data_t[:8]
                else:
                    data_t = str(theta[j])
                    data_t = data_t[:7]
                    
                out_string = ""
                out_string += data_x
                out_string += "," + data_y
                out_string += "," + data_t
                out_string += "\n"
                f.write(out_string)
    print("Archivo pointcloud " + str(data_index) + " generado \n")     




def print_covariance_data(x, y, theta, c_xx, c_yy, c_tt):

    with open("Varianza_prueba_reset.csv", "w") as f:
            for j in range(len(x)):
                if x[j] < 0:
                    data_x = str(x[j])
                    data_x = data_x[:8]
                else:
                    data_x = str(x[j])
                    data_x = data_x[:7]
                
                if y[j] < 0:
                    data_y = str(y[j])
                    data_y = data_y[:8]
                else:
                    data_y = str(y[j])
                    data_y = data_y[:7]
                
                if theta[j] < 0:
                    data_t = str(theta[j])
                    data_t = data_t[:8]
                else:
                    data_t = str(theta[j])
                    data_t = data_t[:7]

                if c_xx[j] < 0:
                    data_cxx = str(c_xx[j])
                    data_cxx = data_cxx[:8]
                else:
                    data_cxx = str(c_xx[j])
                    data_cxx = data_cxx[:7]
                
                if c_yy[j] < 0:
                    data_cyy = str(c_yy[j])
                    data_cyy = data_cyy[:8]
                else:
                    data_cyy = str(c_yy[j])
                    data_cyy = data_cyy[:7]
                
                if c_tt[j] < 0:
                    data_ctt = str(c_tt[j])
                    data_ctt = data_ctt[:8]
                else:
                    data_ctt = str(c_tt[j])
                    data_ctt = data_ctt[:7]

                out_string = ""
                out_string += data_x
                out_string += "," + data_y
                out_string += "," + data_t
                out_string += "," + data_cxx
                out_string += "," + data_cyy
                out_string += "," + data_ctt
                out_string += "\n"
                f.write(out_string)
    print("Archivo de covarianza generado \n")