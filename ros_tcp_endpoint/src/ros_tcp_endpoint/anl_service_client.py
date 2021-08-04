#!/usr/bin/env python


import rospy
import sys
import moveit_commander
import math

from ur_msgs.srv import cmdVelJ1, cmdVelJ1Response, MoverService, MoverServiceResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

print("start of file")

joint_names = ['shoulder_lift_joint', 'shoulder_pan_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        

def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)

def handle_service(req):
    print("Got service %f"%req.cmd_vel)
    response = cmdVelJ1Response()
    response.success = 1
    return response

def PlanTrajectory(req):
    response = MoverServiceResponse()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]

    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)

    if not pre_grasp_pose.joint_trajectory.points:
        return response

    response.trajectories.append(pre_grasp_pose)

    move_group.clear_pose_targets()

    return response

def cmd_vel_server():
    print("hello")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('anl_server_node')
    print("node is up")

    s1 = rospy.Service('Cmd_Vel', cmdVelJ1, handle_service)
    s2 = rospy.Service('ur5_moveit', MoverService, PlanTrajectory)
    print("Ready to plan")
    rospy.spin()
    
if __name__ == "__main__":
    cmd_vel_server()