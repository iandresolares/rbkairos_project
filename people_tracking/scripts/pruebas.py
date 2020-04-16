#!/usr/bin/env python

import rospy
import math
from custom_classes import *  # imports all classes
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from squaternion import euler2quat



PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

#global variables
robot = position()
person = position()
tracking = True


#Callback functions

var = False

def amcl_pose_callback(msg):
    rospy.loginfo("amcl_pose recieved")
    #global robot
    print var
    global var
    var = True




if __name__ == "__main__":

    rospy.init_node("pruebas")

    #initialize action client
    
    
    #initialize /amcl_pose subscriber
    # confirmar que el topic esta bien
    sub_amcl = rospy.Subscriber("/robot/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    #initialize /person_pose subscriber
    #sub_person = rospy.Subscriber("/person_pose", PoseStamped, person_pose_callback)
    
    if var:
        print("DENTRO DEL IF")

    rospy.spin()
    
    #robot = position(-14,-20)
    #person = position(-14,-10)
    #goal = calculate_goal_position(robot, person)
    #print("x_goal: " + str(goal.x) + "   y_goal: " + str(goal.y) + "   theta_goal: " + str(goal.theta))
    
