#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped



if __name__ == "__main__":

    rospy.init_node("person_pose_publisher")
    pub = rospy.Publisher("/person_pose", PoseStamped, queue_size=10)
    rate = rospy.Rate(0.3)
    i = 0
    while not rospy.is_shutdown():
        
        pose = PoseStamped()
        pose.header.frame_id = "robot_map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = i
        pose.pose.position.y = 0
        pub.publish(pose)
        i += 0.5
        if i == 5:
            i = -4.5
        rate.sleep()

        
        


    rospy.loginfo("Node was stopped")

