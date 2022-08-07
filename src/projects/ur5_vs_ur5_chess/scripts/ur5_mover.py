#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from pyquaternion import Quaternion
from time import sleep
import math

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class Ur5_Move_Group(object):
    def __init__(self):
        super(Ur5_Move_Group, self).__init__()

        
        #moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface_tutorial',
        #                anonymous=True)

        
        robot = moveit_commander.RobotCommander()

        
        scene = moveit_commander.PlanningSceneInterface()

        
        group_name = "ur5_robot"
        gripper_name = "panda_gripper"
        
        group = moveit_commander.MoveGroupCommander(group_name)
        gripper = moveit_commander.MoveGroupCommander(gripper_name)

        
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    
        planning_frame = group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

    
        eef_link = group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

    
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

    
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
    
        
        self.robot = robot
        self.scene = scene
        self.group = group
        self.gripper = gripper
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.055  # above the panda_hand frame
        box_name = "test"
        scene.add_box(box_name, box_pose, size=(3.0, 3.0, 0.1))
        self.box_name = box_name


        
        self.safe_moving_box = geometry_msgs.msg.PoseStamped()
        self.safe_moving_box.header.frame_id = "world"
        self.safe_moving_box.pose.orientation.w = 1.0
        self.safe_moving_box.pose.position.x = -0.75
        self.safe_moving_box_name = "safe_moving_box"
        

        
        
        
        
        
    def map_open(self):
        self.scene.remove_world_object(self.safe_moving_box_name)

    def map_close(self):
        self.scene.add_box(self.safe_moving_box_name, self.safe_moving_box, size=(1.0, 1.0, 0.180))

    def go_to_pose_goal(self, x, y, z):
        self.map_close()
        group = self.group    
        q1 = Quaternion(axis=[1, 0,0], angle=-pi/2)
        q2 = Quaternion(axis=[0, 0,1], angle=0)
        q = q1*q2

        wpose = Pose()
        wpose.orientation.w = q.w
        wpose.orientation.x = q.x
        wpose.orientation.y = q.y
        wpose.orientation.z = q.z

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        group.set_pose_target(wpose)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose
        self.map_open()
        return all_close(wpose, current_pose, 0.01)

    
    def go_to_robot_start_pose(self):
        
        self.map_close()
        joint_goal = self.group.get_current_joint_values()
        gripper_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = -4.3011
        joint_goal[1] = -1.7585
        joint_goal[2] = 2.1694
        joint_goal[3] = 4.3017
        joint_goal[4] = 4.7133
        joint_goal[5] = -5.8711
        gripper_goal[0] = 0.0
        gripper_goal[1] = 0.0
        self.gripper.go(gripper_goal, wait=True)
        self.gripper.stop()
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        self.map_open()

    def cartesian_up(self, up=0.35):
        pose = self.group.get_current_pose().pose
        pose.position.z = up
        (plan, fraction) = self.group.compute_cartesian_path([pose], 0.01, 0.0)
        self.group.execute(plan, wait=True)

    def cartesian_down(self, down=0.25):
        pose = self.group.get_current_pose().pose
        pose.position.z = down
        (plan, fraction) = self.group.compute_cartesian_path([pose], 0.01, 0.0)
        self.group.execute(plan, wait=True)

    def cartesian_move(self, x, y):
        pose = self.group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        (plan, fraction) = self.group.compute_cartesian_path([pose], 0.01, 0.0)
        self.group.execute(plan, wait=True)

    def pick_piece(self, location_piece, location_destination):
        self.up = 0.35
        self.down = 0.25

        self.cartesian_up(self.up)
        self.map_close()
        self.cartesian_move(location_piece[0], location_piece[1])
        self.map_open()
        self.gripper_open()
        self.cartesian_down(self.down)
        self.gripper_close()
        self.cartesian_up(self.up)
        self.map_close()
        self.cartesian_move(location_destination[0], location_destination[1])
        self.map_open()
        self.cartesian_down(self.down)
        self.gripper_open()
        self.cartesian_up(self.up)
        print(self.robot.get_current_state().joint_state.position)



    def gripper_open(self):
        joint_states = self.gripper.get_current_joint_values()
        joint_states[0] = 0.0175
        joint_states[1] = 0.0175
        self.gripper.go(joint_states, wait=True)
        self.gripper.stop()

    def gripper_close(self):
        joint_states = self.gripper.get_current_joint_values()
        joint_states[0] = 0.01
        joint_states[1] = 0.01
        self.gripper.go(joint_states, wait=True)
        self.gripper.stop()



if __name__ == '__main__':
    

    """
    positions = {
        "rook_w_l": [-0.285, 0.215],
        "knight_w_l": [-0.285, 0.150],
        "king": [-0.285, 0.03],
        "pawn_3": [-0.350, 0.03],
        "rook_w_r": [-0.285, -0.215],
    }

    destinations = {
        "rook_w_l": [-0.715, -0.215],
        "knight_w_l": [-0.715, -0.150],
        "king": [-0.715, -0.03],
        "pawn_3": [-0.650, -0.03],
        "rook_w_r": [-0.715, 0.215],
    }
    """

    robot = Ur5_Move_Group()
    

    
    