#!/usr/bin/env python
import math

SECURITY_RADIUS = 1.5 # [m]

class position:

    def __init__(self, x=0, y=0, z=0, theta=0):

        self.x = x
        self.y = y
        self.z = z
        self.theta = theta



#Function which calculates the position the robot should go to follow the tracked person
#works if robot and person coordinates are absolute, the output coordinates are also absolute

def calculate_goal_position(robot, person, security_radius=SECURITY_RADIUS): 
    

    goal = position()

    h = math.sqrt((abs(person.x-robot.x))**2 + (abs(person.y-robot.y))**2) - security_radius #[m]

    alpha = math.atan2((person.y-robot.y),(person.x-robot.x))  # angle from x-axis [rad]
    
    goal.theta = round(alpha, 3) #rad
    goal.x = round(robot.x + h*math.cos(alpha), 3)
    goal.y = round(robot.y + h*math.sin(alpha), 3)


    return goal

