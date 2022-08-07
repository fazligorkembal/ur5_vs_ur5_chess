#!/usr/bin/env python3
import rospy
import smach
from ur5_state_machine import Ur5_Mover



def main():
    rospy.init_node('smach_example_state_machine')
    
    robot_white = None
    robot_white_mover = Ur5_Mover(robot=robot_white, robot_name="Robot_White")
    robot_white_mover.move("ed45")


if __name__ == "__main__":
    main()
