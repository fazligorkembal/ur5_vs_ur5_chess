#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from ur5_mover import Ur5_Move_Group


class Ur5_Cartesian_Move(smach.State):
    def __init__(self, robot=None, position=None):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, userdata):
        if True:
            return 'success'
        else:
            return 'fail'

class Ur5_Gripper_Status(smach.State):
    def __init__(self, robot=None, gripper_status=None):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.gripper_status = gripper_status
    
    def execute(self, userdata):        
        if self.gripper_status == 'open' or self.gripper_status == 'close':
            return 'success'
        else:
            return 'fail'
        

class Ur5_Start_Pose(smach.State):
    def __init__(self, robot=None, error=None):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.error_count = 0
        self.robot = robot

    def execute(self, ud):
        #self.robot.go_to_pose_goal(-0.250, 0.300, 0.35)
        self.robot.go_to_robot_start_pose()
        if True:
            rospy.loginfo("Start pose reached")
            return 'success'
        else:
            return 'fail'
        

class Ur5:
    def __init__ (self):
        self.robot = Ur5_Move_Group()
        self.robot_name = rospy.get_namespace().replace("/", "")
        rospy.loginfo("Waiting for %s...", self.robot_name)
        self.move(move_code="test", capture=None)
    
    def move(self, move_code=None, capture=None):
        sm_without_capture=smach.StateMachine(outcomes=['success', 'fail'])
        
        with sm_without_capture:
            smach.StateMachine.add('{}_START_POSE'.format(self.robot_name), Ur5_Start_Pose(robot=self.robot), transitions={'success':'success', 'fail':'fail'})
        out = sm_without_capture.execute()
        print(out)


if __name__ == "__main__":
    rospy.init_node('ur5_state_machine')
    ur5 = Ur5()

        
    """
    def move(self, move_code):
        sm = smach.StateMachine(outcomes=['success', 'fail'])
        position = "TEMP"

        with sm:
            smach.StateMachine.add('{}_START_POSE'.format(self.robot_name), Ur5_Start_Pose(robot=self.robot), transitions={'success':'success', 'fail':'fail'})
            #smach.StateMachine.add('{}_START_POSE'.format(self.robot_name), Ur5_Start_Pose(robot=self.robot), transitions={'success':'{}_CARTESIAN_MOVE_FOR_CLOSING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_CARTESIAN_MOVE_FOR_CLOSING'.format(self.robot_name), Ur5_Cartesian_Move(robot=self.robot, position=position), transitions={'success':'{}_GRIPPER_OPENING_FOR_PICKING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_GRIPPER_OPENING_FOR_PICKING'.format(self.robot_name), Ur5_Gripper_Status(robot=self.robot, gripper_status='open'), transitions={'success':'{}_CARTESIAN_MOVE_FOR_PICKING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_CARTESIAN_MOVE_FOR_PICKING'.format(self.robot_name), Ur5_Cartesian_Move(robot=self.robot, position=position), transitions={'success':'{}_GRIPPER_CLOSING_FOR_PICKING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_GRIPPER_CLOSING_FOR_PICKING'.format(self.robot_name), Ur5_Gripper_Status(robot=self.robot, gripper_status='close'), transitions={'success':'{}_CARTESIAN_MOVE_FOR_RISING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_CARTESIAN_MOVE_FOR_RISING'.format(self.robot_name), Ur5_Cartesian_Move(robot=self.robot, position=position), transitions={'success':'{}_CARTESIAN_MOVE_FOR_PLACING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_CARTESIAN_MOVE_FOR_PLACING'.format(self.robot_name), Ur5_Cartesian_Move(robot=self.robot, position=position), transitions={'success':'{}_CARTESIAN_MOVE_FOR_RELEASING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_CARTESIAN_MOVE_FOR_RELEASING'.format(self.robot_name), Ur5_Cartesian_Move(robot=self.robot, position=position), transitions={'success':'{}_GRIPPER_OPENING_FOR_PLACING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_GRIPPER_OPENING_FOR_PLACING'.format(self.robot_name), Ur5_Gripper_Status(robot=self.robot, gripper_status='open'), transitions={'success':'{}_CARTESIAN_MOVE_FOR_WAITING'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_CARTESIAN_MOVE_FOR_WAITING'.format(self.robot_name), Ur5_Cartesian_Move(robot=self.robot, position=position), transitions={'success':'{}_RETURN_START_POSE'.format(self.robot_name), 'fail':'fail'})
            #smach.StateMachine.add('{}_RETURN_START_POSE'.format(self.robot_name), Ur5_Start_Pose(robot=self.robot), transitions={'success':'success', 'fail':'fail'})
        out = sm.execute()
        print(out)
    """