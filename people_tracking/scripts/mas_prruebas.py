#!/usr/bin/env python
from squaternion import euler2quat

import rospy
import math
from custom_classes import * #imports all classes
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


SAFE_RADIUS = 1.5 # [m]

#global variables
robot = position()
person = position()
stop_tracking = 0

def feedback_callback(feedback):
    print feedback

if __name__ == "__main__":

    rospy.init_node("prueba_action_server")

    action_server_name = "/robot/move_base"
    client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

    rospy.loginfo("Waiting for action server" + action_server_name)
    client.wait_for_server()
    rospy.loginfo("Action server found..." + action_server_name)

    gp = MoveBaseGoal()

    gp.target_pose.header.frame_id = "robot_map"
    gp.target_pose.header.stamp = rospy.Time.now()
    gp.target_pose.pose.position.x = 3
    gp.target_pose.pose.orientation.z = 0.79
    gp.target_pose.pose.orientation.w = 0.613
    client.send_goal(gp, feedback_cb=feedback_callback)


    client.wait_for_result()
    print(str(client.get_result()))


    