#!/usr/bin/env python3
import math
from custom_classes import * #imports all classes

SAFE_RADIUS = 1.5 # [m]


#Function which calculates the position the robot should go to follow the tracked person
#works if robot and person coordinates are absolute, the output coordinates are also absolute

def calculate_goal_position(robot, person): 

    goal = position()

    h = math.sqrt((abs(person.x-robot.x))**2 + (abs(person.y-robot.y))**2) - SAFE_RADIUS #[m]

    alpha = math.atan2((person.y-robot.y),(person.x-robot.x))  # angle from x-axis [rad]
    
    goal.theta = alpha
    goal.x = robot.x + h*math.cos(alpha)
    goal.y = robot.y + h*math.sin(alpha)

    return goal
    





if __name__ == "__main__":

    robot = position(-14,-20)
    person = position(-14,-10)
    goal = calculate_goal_position(robot, person)
    print("x_goal: " + str(goal.x) + "   y_goal: " + str(goal.y) + "   theta_goal: " + str(goal.theta))
    
