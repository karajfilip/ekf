#!/usr/bin/env python

import rospy
import yaml
import math

from std_srvs.srv import Trigger
from nist_gear.srv import AGVControl, AGVToAssemblyStation, AGVToKittingStation, ConveyorBeltControl, AssemblyStationSubmitShipment, VacuumGripperControl

from geometry_msgs.msg import TransformStamped
from nist_gear.msg import Order, Model, LogicalCameraImage, Proximity, VacuumGripperState, RobotHealth
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


### Process management ###

def start_competition():
	"""ROS service: Start competition"""
	rospy.wait_for_service('/ariac/start_competition')
	rospy.ServiceProxy('/ariac/start_competition', Trigger)

def end_competition():
	"""ROS service: End competition"""
	rospy.wait_for_service('/ariac/end_competition')
	rospy.ServiceProxy('/ariac/end_competition', Trigger)

def get_order():
	"""ROS topic: Get order"""
	return rospy.wait_for_message('/ariac/orders', Order)

def get_time():
	"""ROS topic: Get simulation time"""
	return rospy.wait_for_message('/ariac/clock', Clock)

def get_competition_state():
	"""ROS topic: state of competition (init, ready, go, end_game, done)"""
	return rospy.wait_for_message('/ariac/competition_state', String)

def submit_kitting_shipment(agv, ast, shp_type):
	"""ROS service: Submit kitting shipment"""
	rospy.wait_for_service('/ariac/' + agv + '/submit_shipment')
	rospy.ServiceProxy('/ariac/' + agv + '/submit_shipment', AGVToAssemblyStation)(ast, shp_type)

def submit_assembly_shipment(ast, shp_type):
	"""ROS service: Submit assembly shipment"""
	rospy.wait_for_service('/ariac/' + ast + '/submit_shipment')
	rospy.ServiceProxy('/ariac/' + ast + '/submit_shipment', AssemblyStationSubmitShipment)(shp_type)


### Actuators ###

## TODO -> direktna i inverzna kinematika

def robot_health():
	"""ROS topic: return whether robot is enabled or disabled"""
	return rospy.wait_for_message('/ariac/robot_health', RobotHealth)

class GripperManager():
	def __init__(self, robot):
		self.robot = robot

	def activate_gripper(self):
		rospy.wait_for_service('/ariac/' + self.robot + '/arm/gripper/control')
		rospy.ServiceProxy('/ariac/' + self.robot + '/arm/gripper/control', VacuumGripperControl)(True) 

	def deactivate_gripper(self):
		rospy.wait_for_service('/ariac/' + self.robot + '/arm/gripper/control')
		rospy.ServiceProxy('/ariac/' + self.robot + '/arm/gripper/control', VacuumGripperControl)(False)

	def is_object_attached(self):
		return rospy.wait_for_message('/ariac/' + self.robot + '/arm/gripper/state', VacuumGripperState)


### Sensors ###

def run():

	#start_competition()
	#rospy.init_node('main_node', anonymous=True)

	order = get_order()
	rospy.loginfo(order.order_id)
	time = get_time()
	rospy.loginfo(time)
	
	#end_competition()


if __name__ == '__main__':
	
	# dok ima ordera (order.shipment)
	while not rospy.is_shutdown():
		run()
