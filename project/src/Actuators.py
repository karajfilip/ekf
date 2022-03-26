#!/usr/bin/env python

import rospy
import np as numpy
import math

from sensor_msgs.msg import JointState
from nist_gear.msg import RobotHealth, VacuumGripperState, ConveyorBeltState
from trajectory_msgs.msg import JointTrajectory
from control_msgs import JointTrajectoryControllerState

from nist_gear.srv import VacuumGripperControl


class Actuators():

	def __init__(self, robot):

		self.robot = robot
		self.gantry_joint_state = JointState()
		self.kitting_joint_state = JointState()
		self.robot_health = RobotHealth()
		self.gantry_torso_state = JointTrajectoryControllerState()
		self.gantry_arm_state = JointTrajectoryControllerState()
		self.kitting_arm_state = JointTrajectoryControllerState()
		self.conveyor_belt_state = ConveyorBeltState()

		# Subscribers
		rospy.Subscriber('/ariac/gantry/joint_states', JointState, self.gantry_joint_state_callback)
		rospy.Subscriber('/ariac/kitting/joint_states', JointState, self.kitting_joint_state_callback)
		rospy.Subscriber('/ariac/robot_health', RobotHealth, self.robot_health_callback)
		rospy.Subscriber('/ariac/gantry/gantry_controller/state', JointTrajectoryControllerState, self.gantry_torso_state_callback)
		rospy.Subscriber('/ariac/gantry/gantry_arm_controller/state', JointTrajectoryControllerState, self.gantry_arm_state_callback)
		rospy.Subscriber('/ariac/kitting/kitting_arm_controller/state', JointTrajectoryControllerState, self.kitting_arm_state_callback)
		rospy.Subscriber('ariac/conveyor/state', ConveyorBeltState, self.conveyor_belt_state_callback)


		# Publishers
		self.gantry_arm_control = rospy.Publisher('/ariac/gantry/gantry_arm_controller/command', JointTrajectory, queue_size=10)
		self.gantry_torso_control = rospy.Publisher('/ariac/gantry/gantry_torso_controller/command', JointTrajectory, queue_size=10)
		self.kitting_arm_control = rospy.Publisher('/ariac/kitting/kitting_arm_controller/command', JointTrajectory, queue_size=10)


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

	def conveyor_belt_state_callback(self, msg)
		# power -> power of belt
		# enabled -> true if the power of the belt can be modified; false if the belt is stopped
		self.conveyor_belt_state = msg
	

	### GRIPPER ###

	def activate_gripper(self):
		rospy.wait_for_service('/ariac/' + self.robot'/arm/gripper/control')
		rospy.ServiceProxy('/ariac/' + self.robot + '/arm/gripper/control', VacuumGripperControl)(True) 

	def deactivate_gripper(self):
		rospy.wait_for_service('/ariac/' + self.robot + '/arm/gripper/control')
		rospy.ServiceProxy('/ariac/' + self.robot + '/arm/gripper/control', VacuumGripperControl)(False)

	def is_object_attached(self):
		return rospy.wait_for_message('/ariac/' + self.robot + '/arm/gripper/state', VacuumGripperState)

	
	### DIRECT & INVERSE KINEMATICS ###

	def transform_matrix(self, theta, d, alpha, a):
		"""Function for calculating transfrom matrix."""
		return np.matrix([ [math.cos(theta), -math.cos(alpha)*math.sin(theta), math.sin(alpha)*math.sin(theta), a*math.cos(theta)],
                       	   [math.sin(theta), math.cos(alpha)*math.cos(theta), -math.sin(alpha)*math.cos(theta), a*math.sin(theta)],
                           [0, 			     math.sin(alpha),                  math.cos(alpha),                                 d],
                           [0,               0,                                0,                                               1] ])

	def direct_kinematics_kitting_arm(self, joint_state):
		"""Function for calculating direct kinematics."""
		q1 = joint_state[0]
		q2 = joint_state[1]
		q3 = joint_state[2]
		q4 = joint_state[3]
		q5 = joint_state[4]
		q6 = joint_state[5]
		q7 = joint_state[6]

		T1 = self.transform_matrix()
		T2 = self.transform_matrix()
		T3 = self.transform_matrix()
		T4 = self.transform_matrix()
		T5 = self.transform_matrix()
		T6 = self.transform_matrix()
		T7 = self.transform_matrix()


        T = T1*T2*T3*T4*T5*T6*T7
        b = np.array([[T[0, 2]], [T[1, 2]], [T[2, 2]]])
        w2 = np.array(math.exp(q6/math.pi)*b)

        return np.array([[T[0, 3]], [T[1, 3]], [T[2, 3]], [1], [0], [0]])

    def inverse_kinematics_kitting_arm(self, target, temp_joint_state):
    	pass

	def actuators_run():
		pass
