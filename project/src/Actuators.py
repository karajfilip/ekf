#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

from sensor_msgs.msg import JointState
from nist_gear.msg import RobotHealth, VacuumGripperState, ConveyorBeltState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from nist_gear.srv import VacuumGripperControl, ChangeGripper

import moveit_commander as mc
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from tf import TransformListener


class Actuators():

    def __init__(self):  #robot

        #self.tf = TransformListener()
        
        print("pocetak inita")

        robot_description = 'robot_description'
        kitting_name = 'kitting_arm'
        kitting_ns = '/ariac/kitting'

        mc.roscpp_initialize(sys.argv)
        self.kitting_robot = mc.RobotCommander(kitting_ns + '/' + robot_description, kitting_ns)
        self.kitting_scene = mc.PlanningSceneInterface(kitting_ns)
        self.kitting_group = mc.MoveGroupCommander(kitting_name, robot_description=kitting_ns + '/' + robot_description, ns=kitting_ns)
        
        gantry_name = 'gantry_full'
        gantry_ns = '/ariac/gantry'

        self.gantry_robot = mc.RobotCommander(gantry_ns + '/' + robot_description, gantry_ns)
        self.gantry_scene = mc.PlanningSceneInterface(gantry_ns)
        self.gantry_group = mc.MoveGroupCommander(gantry_name, robot_description=gantry_ns + '/' + robot_description, ns=gantry_ns)
        
        gantry_arm_name = 'gantry_arm'
        gantry_arm_ns = '/ariac/gantry'

        self.gantry_arm = mc.RobotCommander(gantry_arm_ns + '/' + robot_description, gantry_arm_ns)
        self.gantry_arm_scene = mc.PlanningSceneInterface(gantry_arm_ns)
        self.gantry_arm_group = mc.MoveGroupCommander(gantry_arm_name, robot_description=gantry_arm_ns + '/' + robot_description, ns=gantry_arm_ns)
        
        self.gantry_joint_state = JointState()
        self.kitting_joint_state = JointState()
        self.robot_health = RobotHealth()
        self.gantry_torso_state = JointTrajectoryControllerState()
        self.gantry_arm_state = JointTrajectoryControllerState()
        self.kitting_arm_state = JointTrajectoryControllerState()
        self.conveyor_belt_state = ConveyorBeltState()
        self.gripper_type = String()


        # Subscribers
        rospy.Subscriber('/ariac/gantry/joint_states', JointState, self.gantry_joint_state_callback)
        rospy.Subscriber('/ariac/kitting/joint_states', JointState, self.kitting_joint_state_callback)
        rospy.Subscriber('/ariac/robot_health', RobotHealth, self.robot_health_callback)
        rospy.Subscriber('/ariac/gantry/gantry_controller/state', JointTrajectoryControllerState, self.gantry_torso_state_callback)
        rospy.Subscriber('/ariac/gantry/gantry_arm_controller/state', JointTrajectoryControllerState, self.gantry_arm_state_callback)
        rospy.Subscriber('/ariac/kitting/kitting_arm_controller/state', JointTrajectoryControllerState, self.kitting_arm_state_callback)
        rospy.Subscriber('/ariac/conveyor/state', ConveyorBeltState, self.conveyor_belt_state_callback)
        rospy.Subscriber('/ariac/gantry/arm/gripper/type', String, self.gripper_type_callback)


        # Publishers
        self.gantry_arm_control = rospy.Publisher('/ariac/gantry/gantry_arm_controller/command', JointTrajectory, queue_size=10)
        self.gantry_torso_control = rospy.Publisher('/ariac/gantry/gantry_torso_controller/command', JointTrajectory, queue_size=10)
        self.kitting_arm_control = rospy.Publisher('/ariac/kitting/kitting_arm_controller/command', JointTrajectory, queue_size=10)
        
        print("kraj init")


    ### CALLBACKS ###
    def gantry_joint_state_callback(self, msg):
        self.gantry_joint_state = msg

    def kitting_joint_state_callback(self, msg):
        self.kitting_joint_state = msg

    def robot_health_callback(self, msg):
        self.robot_health = msg

    def gantry_torso_state_callback(self, msg):
        self.gantry_torso_state = msg

    def gantry_arm_state_callback(self, msg):
        self.gantry_arm_state = msg

    def kitting_arm_state_callback(self, msg):
        self.kitting_arm_state = msg

    def gripper_type_callback(self, msg):
        self.gripper_type = msg.data

    def conveyor_belt_state_callback(self, msg):
        # power -> power of belt
        # enabled -> true if the power of the belt can be modified; false if the belt is stopped
        self.conveyor_belt_state = msg

    ### GRIPPER ###

    def activate_kitting_gripper(self):
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        rospy.ServiceProxy('/ariac/kitting/arm/gripper/control', VacuumGripperControl)(True)

    def deactivate_kitting_gripper(self):
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        rospy.ServiceProxy('/ariac/kitting/arm/gripper/control', VacuumGripperControl)(False)


    def activate_gantry_gripper(self):
        rospy.wait_for_service('/ariac/gantry/arm/gripper/control')
        rospy.ServiceProxy('/ariac/gantry/arm/gripper/control', VacuumGripperControl)(True)

    def deactivate_gantry_gripper(self):
        rospy.wait_for_service('/ariac/gantry/arm/gripper/control')
        rospy.ServiceProxy('/ariac/gantry/arm/gripper/control', VacuumGripperControl)(False)


    def is_object_attached_kitting(self):
        return rospy.wait_for_message('/ariac/kitting/arm/gripper/state', VacuumGripperState)

    def is_object_attached_gantry(self):
        return rospy.wait_for_message('/ariac/kitting/arm/gripper/state', VacuumGripperState)
    
    def change_gripper(self, gripper_type):
        rospy.wait_for_service('/ariac/gantry/arm/gripper/change')
        change = rospy.ServiceProxy('/ariac/gantry/arm/gripper/change', ChangeGripper)

        try:
            resp = change(gripper_type)
            rospy.loginfo("Gripper changed.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    #def gripper_type(self):
    #    gripper_type = rospy.wait_for_message('/ariac/gantry/arm/gripper/type', String)
    #    return str(gripper_type.data)
   


    ### DIRECT & INVERSE KINEMATICS ###
    def direct_kinematics_kitting_arm(self):
        ee_pose = self.kitting_group.get_current_pose().pose

        direct_kin = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]

        return direct_kin

    def direct_kinematics_gantry_arm(self):
        ee_pose = self.gantry_group.get_current_pose().pose

        direct_kin = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]

        return direct_kin

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [qx, qy, qz, qw]

    # Start_joints = pocetna pozicija jointova iz koje se racuna inverzna. Ako je None, uzima se trenutna pozicija
    def inverse_kinematics_kitting_arm(self, target, start_joints = None):
        # Inverse kinematics for kitting arm
        # INPUT:
        #   - target: numpy 3x1
        # OUTPUT:
        #   - list 7x1
        print("usao u inverz")

        quaternion_ang = self.euler_to_quaternion(target[3], target[4], target[5])

        targetPose = PoseStamped()
        targetPose.header.stamp = rospy.Time.now()
        targetPose.pose.position.x = target[0]
        targetPose.pose.position.y = target[1]
        targetPose.pose.position.z = target[2]
        targetPose.pose.orientation.x = quaternion_ang[0]
        targetPose.pose.orientation.y = quaternion_ang[1]
        targetPose.pose.orientation.z = quaternion_ang[2]
        targetPose.pose.orientation.w = quaternion_ang[3]
        #print("TargetPose: ")
        #print(targetPose)

        if start_joints is None:
            robotTemp = RobotState()
            robotTemp.joint_state.name = self.kitting_joint_state.name
            robotTemp.joint_state.position = self.kitting_joint_state.position
        else:
            robotTemp = RobotState()
            robotTemp.joint_state.name = self.kitting_joint_state.name
            robotTemp.joint_state.position = start_joints
        #print("RobotTemp: ")
        #print(robotTemp)

        service_request = PositionIKRequest()
        service_request.group_name = "kitting_arm"
        service_request.pose_stamped = targetPose
        service_request.robot_state = robotTemp
        service_request.timeout.secs = 1
        service_request.avoid_collisions = True
        #print("service_request: ")
        #print(service_request)

        rospy.wait_for_service('/ariac/kitting/compute_ik')
        compute_ik = rospy.ServiceProxy('/ariac/kitting/compute_ik', GetPositionIK)
        
        try:
            resp = compute_ik(service_request)
            #rospy.loginfo(resp)
            print("gotov inverz")
        except rospy.ServiceException as exc:
            print(exc)
        #print("return:")
        #print(list(resp.solution.joint_state.position))
        return list(resp.solution.joint_state.position)

    # Target_group = grupa jointova koja se mice (0=gantry_full, 1=gantry_arm)
    def inverse_kinematics_gantry(self, target, target_group = 0, start_joints = None):
        # Inverse kinematics for kitting arm
        # INPUT:
        #   - target: numpy 3x1
        # OUTPUT:
        #   - list 7x1
        print("usao u inverz")

        quaternion_ang = self.euler_to_quaternion(target[3], target[4], target[5])

        targetPose = PoseStamped()
        targetPose.header.stamp = rospy.Time.now()
        targetPose.pose.position.x = target[0]
        targetPose.pose.position.y = target[1]
        targetPose.pose.position.z = target[2]
        targetPose.pose.orientation.x = quaternion_ang[0]
        targetPose.pose.orientation.y = quaternion_ang[1]
        targetPose.pose.orientation.z = quaternion_ang[2]
        targetPose.pose.orientation.w = quaternion_ang[3]

        robotTemp = RobotState()
        if start_joints is None:
            robotTemp.joint_state.position = self.gantry_joint_state.position
            robotTemp.joint_state.name = self.gantry_joint_state.name
        else:
            robotTemp.joint_state.position = start_joints
            #if target_group == 0:
            #    robotTemp.joint_state.name = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"#]
            #elif target_group == 1:
            robotTemp.joint_state.name = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint", "gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]

        service_request = PositionIKRequest()
        if target_group == 0:
            service_request.group_name = "gantry_full"
            service_request.pose_stamped = targetPose
            service_request.robot_state = robotTemp
            service_request.timeout.secs = 1
            service_request.avoid_collisions = True
        elif target_group == 1:
            service_request.group_name = "gantry_arm"
            service_request.pose_stamped = targetPose
            service_request.robot_state = robotTemp
            service_request.timeout.secs = 1
            service_request.avoid_collisions = True

        rospy.wait_for_service('/ariac/gantry/compute_ik')
        compute_ik = rospy.ServiceProxy('/ariac/gantry/compute_ik', GetPositionIK)
        
        try:
            resp = compute_ik(service_request)
            #rospy.loginfo(resp)
            print("gotov inverz")
        except rospy.ServiceException as exc:
            print(exc)
        print("return:")
        print(list(resp.solution.joint_state.position))
        return list(resp.solution.joint_state.position)



if __name__ == '__main__':
    #rospy.init_node('ActuatorsTest')
    try:
        node = Actuators()
    except Exception as e:
        print(e)