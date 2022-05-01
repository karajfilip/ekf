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

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from tf import TransformListener


class Actuators():

    def __init__(self):  #robot

        #self.tf = TransformListener()
        
        print("pocetak inita")

        moveGroupName_gantry = 'gantry'
        moveGroupName_kitting = 'kitting_arm'
        ns_kitting = 'ariac/kitting'
        robot_description_kitting = ns_kitting + '/robot_description'

        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_node', anonymous=True)

        kitting_robot = moveit_commander.RobotCommander(robot_description_kitting)
        scene = moveit_commander.PlanningSceneInterface("ariac/kitting")

        group = moveit_commander.MoveGroupCommander(moveGroupName_kitting, robot_description=robot_description_kitting, ns=ns_kitting)
        group.allow_replanning(True)
        group.allow_looking(True)

        #self.robot = kitting_robot
        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        print("EEF")
        print(eef_link)
        #group_names = robot.get_group_names()
        #print("group_names")
        

        
        self.gantry_joint_state = JointState()
        self.kitting_joint_state = JointState()
        self.robot_health = RobotHealth()
        self.gantry_torso_state = JointTrajectoryControllerState()
        self.gantry_arm_state = JointTrajectoryControllerState()
        self.kitting_arm_state = JointTrajectoryControllerState()
        self.conveyor_belt_state = ConveyorBeltState()
        #self.gripper_type = String()


        # Subscribers
        rospy.Subscriber('/ariac/gantry/joint_states', JointState, self.gantry_joint_state_callback)
        rospy.Subscriber('/ariac/kitting/joint_states', JointState, self.kitting_joint_state_callback)
        rospy.Subscriber('/ariac/robot_health', RobotHealth, self.robot_health_callback)
        rospy.Subscriber('/ariac/gantry/gantry_controller/state', JointTrajectoryControllerState, self.gantry_torso_state_callback)
        rospy.Subscriber('/ariac/gantry/gantry_arm_controller/state', JointTrajectoryControllerState, self.gantry_arm_state_callback)
        rospy.Subscriber('/ariac/kitting/kitting_arm_controller/state', JointTrajectoryControllerState, self.kitting_arm_state_callback)
        rospy.Subscriber('/ariac/conveyor/state', ConveyorBeltState, self.conveyor_belt_state_callback)
        #rospy.Subscriber('/ariac/gantry/arm/gripper/type', String, self.gripper_type_callback)


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

    #def gripper_type_callback(self, msg):
     #   self.gripper_type = msg

    def conveyor_belt_state_callback(self, msg):
        # power -> power of belt
        # enabled -> true if the power of the belt can be modified; false if the belt is stopped
        self.conveyor_belt_state = msg

    ### GRIPPER ###

    def activate_kitting_gripper(self):
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        act_gripp = rospy.ServiceProxy('/ariac/kitting/arm/gripper/control', VacuumGripperControl)(True)

        try:
            resp = act_gripp()
            rospy.loginfo("Gripper activated.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    def deactivate_kitting_gripper(self):
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        deact_gripp = rospy.ServiceProxy('/ariac/kitting/arm/gripper/control', VacuumGripperControl)(False)

        try:
            resp = deact_gripp()
            rospy.loginfo("Gripper deactivated.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    def activate_gantry_gripper(self):
        rospy.wait_for_service('/ariac/gantry/arm/gripper/control')
        rospy.ServiceProxy('/ariac/gantry/arm/gripper/control', VacuumGripperControl)(True)

        """try:
            resp = act_gripp(True)
            rospy.loginfo("Gantry gripper activated.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))"""

    def deactivate_gantry_gripper(self):
        rospy.wait_for_service('/ariac/gantry/arm/gripper/control')
        act_gripp = rospy.ServiceProxy('/ariac/gantry/arm/gripper/control', VacuumGripperControl)

        try:
            resp = act_gripp(False)
            rospy.loginfo("Gantry gripper deactivated.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    def is_object_attached_kitting(self):
        return rospy.wait_for_message('/ariac/kitting/arm/gripper/state', VacuumGripperState)

    def is_object_attached_gantry(self):
        return rospy.wait_for_message('/ariac/gantry/arm/gripper/state', VacuumGripperState)

    def change_gripper(self, gripper_type):
        rospy.wait_for_service('/ariac/gantry/arm/gripper/change')
        change = rospy.ServiceProxy('/ariac/gantry/arm/gripper/change', ChangeGripper)

        try:
            resp = change(gripper_type)
            rospy.loginfo("Gripper changed.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    def gripper_type(self):
        return rospy.wait_for_message('/ariac/gantry/arm/gripper/type', String)


    ### DIRECT & INVERSE KINEMATICS ###

    def transform_matrix(self, theta, a, d, alpha):
        """Function for calculating transfrom matrix."""
        return np.matrix([  [math.cos(theta), -math.cos(alpha)*math.sin(theta), math.sin(alpha)*math.sin(theta), a*math.cos(theta)],
                            [math.sin(theta), math.cos(alpha)*math.cos(theta), -math.sin(alpha)*math.cos(theta), a*math.sin(theta)],
                            [0,               math.sin(alpha),                  math.cos(alpha),                                 d],
                            [0,               0,                                0,                                               1] ])

    def direct_kinematics_kitting_arm(self, kitting_joint_state):
        """Function for calculating direct kinematics."""
        q1 = kitting_joint_state[0]         # 4) elbow joint
        q2 = kitting_joint_state[1]         # 1) linear_arm_actuator
        q3 = kitting_joint_state[2]         # 3) shoulder_lift_joint
        q4 = kitting_joint_state[3]         # 2) shoulder_pan
        q5 = kitting_joint_state[5]         # 6) wrist 1
        q6 = kitting_joint_state[6]         # 7) wrist 2
        q7 = kitting_joint_state[7]         # 8) wrist 3

        T1 = self.transform_matrix(q1, -0.5723, 0.0, 0.0)
        T2 = self.transform_matrix(0.0, 0.0, q2, 0.0)
        T3 = self.transform_matrix(q3, -0.612, 0.0, 0.0)
        T4 = self.transform_matrix(q4, 0.0, 0.1273, math.pi/2)
        T5 = self.transform_matrix(q5, 0.0, 0.163941, math.pi/2)
        T6 = self.transform_matrix(q6, 0.0, 0.1157, -math.pi/2)
        T7 = self.transform_matrix(q7, 0.0, 0.0922, 0.0)

        T = T2*T4*T3*T1*T5*T6*T7
        b = np.array([[T[0, 2]], [T[1, 2]], [T[2, 2]]])
        w2 = np.array(math.exp(q6/math.pi)*b)

        return np.array([[T[0, 3]], [T[1, 3]], [T[2, 3]], [w2[0, 0]], [w2[1, 0]], [w2[2, 0]]])

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [qx, qy, qz, qw]


    def inverse_kinematics_kitting_arm(self, target):
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
        print("TargetPose: ")
        print(targetPose)

        robotTemp = RobotState()
        robotTemp.joint_state.name = self.kitting_joint_state.name
        robotTemp.joint_state.position = self.kitting_joint_state.position
        print("RobotTemp: ")
        print(robotTemp)

        service_request = PositionIKRequest()
        service_request.group_name = "kitting_arm"
        service_request.pose_stamped = targetPose
        service_request.robot_state = robotTemp
        service_request.timeout.secs = 1
        print("service_request: ")
        print(service_request)

        rospy.wait_for_service('/ariac/kitting/compute_ik')
        compute_ik = rospy.ServiceProxy('/ariac/kitting/compute_ik', GetPositionIK)
        
        try:
            resp = compute_ik(service_request)
            rospy.loginfo(resp)
            print("gotov inverz")
        except rospy.ServiceException as exc:
            print(exc)
        print("return:")
        print(list(resp.solution.joint_state.position))
        return list(resp.solution.joint_state.position)

    def inverse_kinematics_gantry(self, target):
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
        print("TargetPose: ")
        print(targetPose)

        rospy.sleep(0.5)
        robotTemp = RobotState()
        robotTemp.joint_state.name = self.gantry_joint_state.name
        robotTemp.joint_state.position = self.gantry_joint_state.position

        print("RobotTemp: ")
        print(robotTemp)

        service_request = PositionIKRequest()
        service_request.group_name = "gantry_full"
        service_request.pose_stamped = targetPose
        service_request.robot_state = robotTemp
        service_request.timeout.secs = 1
        service_request.avoid_collisions = True
        print("service_request: ")
        print(service_request)

        rospy.wait_for_service('/ariac/gantry/compute_ik')
        compute_ik = rospy.ServiceProxy('/ariac/gantry/compute_ik', GetPositionIK)
        
        try:
            resp = compute_ik(service_request)
            rospy.loginfo(resp)
            print("gotov inverz")
        except rospy.ServiceException as exc:
            print(exc)
        print("return:")
        print(list(resp.solution.joint_state.position))
        return list(resp.solution.joint_state.position)

    def actuators_run(self):
        rospy.sleep(1.0)
        rospy.loginfo("123")
        #rospy.loginfo("FK za () = ", self.direct_kinematics_kitting_arm([1.7406034204577985, -1.512444221586856e-06, -1.2488649978462485, 0.00021580356888506458, -4.7316100815208983e-07, -2.0875959075821413, -1.5800032881696922, 5.921660003238571e-06]))
        print("IK za (-1.9, 2.963, 1) = ", self.inverse_kinematics_kitting_arm([-0.25, 0.7, 1.2], [[2.887680004303906, 0.7129823268720327, -2.5271091739946954, 4.361105553115772, -3.5243445088651564, -1.6358502975854903, 0.7084337129723515, 2.8318168610041994e-07]
]))
        print("kraj run-a")

if __name__ == '__main__':
    #rospy.init_node('ActuatorsTest')
    try:
        node = Actuators()
        node.actuators_run()
    except Exception as e:
        print(e)


