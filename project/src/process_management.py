#!/usr/bin/env python

import rospy
import yaml
import math

from std_srvs.srv import Trigger
from nist_gear.srv import AGVControl,  ConveyorBeltControl, AssemblyStationSubmitShipment, SubmitKittingShipment

from geometry_msgs.msg import TransformStamped
from nist_gear.msg import Orders, Model
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

#from Actuators import Actuators
#from pick_and_place import RobotMover
from Sensors import Sensors_functions

class MainNode():

    def __init__(self):
        #rospy.init_node('main_node', anonymous=True)
        self.order = Orders()
        self.orders = []
        self.received_order = False
        self.order_id = Orders()
        self.kitting_info = Orders()
        self.assemlby_info = Orders()
        self.order_sub = rospy.Subscriber('/ariac/orders', Orders, self.get_order)

    def start_competition(self):
        """ROS service: Start competition"""
        rospy.loginfo("Waiting for competition to be ready...")
        rospy.wait_for_service('/ariac/start_competition')
        rospy.loginfo("Competition is ready.")
        start_competition = rospy.ServiceProxy('/ariac/start_competition', Trigger)

        try:
            resp = start_competition()
            rospy.loginfo("Competition started!")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    def end_competition(self):
        """ROS service: End competition"""
        rospy.wait_for_service('/ariac/end_competition')
        end_competition = rospy.ServiceProxy('/ariac/end_competition', Trigger)

        try:
            resp = end_competition()
            rospy.loginfo("Competition ended!")
        except rospy.ServiceException as exc:
            print(str(exc))

    def get_order(self, msg):
        """ROS topic: Get order"""
        self.order = msg
        self.orders.append(msg)
        self.received_order = True
        #self.order_id = msg.order_id
        #self.kitting_info = msg.kitting_shipments
        #self.assemlby_info = msg.assembly_shipments

    def kitting_agv(self):
        return self.order.kitting_shipments[0].agv

    def procces_kitting_shipment(self, kitting_products):
        
        self.agv = self.order.kitting_shipments[0].agv
        self.assemlby_station = self.order.kitting_shipments[0].assembly_station

        rospy.logerr(self.agv)
        
        parts = []

        for product in kitting_products.products:
            part = {
                "type": product.type,
                "gripper": product.gripper,
                "pose": product.pose
            }
            parts.append(part)

        return parts

    def procces_assembly_shipment(self, assembly_products):
        
        self.assemlby_station = self.order.assembly_shipments[0].station_id

        parts = []

        for product in assembly_products.products:
            part = {
                "type": product.type,
                "pose": product.pose
            }
            parts.append(part)

        return parts


    def submit_kitting_shipment(self, agv, assemlby_station, shipment_type):
        """ ROS Service for submit a kitting shipment."""
        rospy.wait_for_service('/ariac/' + agv + '/submit_kitting_shipment')
        submit_kitting = rospy.ServiceProxy('/ariac/' + agv + '/submit_kitting_shipment', SubmitKittingShipment)(assemlby_station, shipment_type)
        
        try:
            resp = submit_kitting()
            rospy.loginfo("Submitting a kitting shipment.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))
    
    def submit_assembly_shipment(self, station_id, shipment_type):
        """ ROS Service for submit an assembly shipment."""
        rospy.wait_for_service('/ariac/' + station_id + '/submit_assembly_shipment')
        submit_assembly = rospy.ServiceProxy('/ariac/' + station_id + 'submit_assembly_shipment', AssemblyStationSubmitShipment)

        try:
            resp = submit_assembly()
            rospy.loginfo("Submitting an assemlby shipment.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))


    def get_time(self):
        """ROS topic: Get simulation time"""
        return rospy.wait_for_message('/clock', Clock)

    def get_competition_state(self):
        """ROS topic: state of competition (init, ready, go, end_game, done)"""
        return rospy.wait_for_message('/ariac/competition_state', String)

    def run(self):
        
        #rospy.init_node('main_node', anonymous=True)
        self.start_competition()

        #while not rospy.is_shutdown():

        #r = rospy.Rate(10)

        while not self.received_order:
            rospy.loginfo("Waiting for order...")
            pass
            
        for order in self.orders:
            if order.kitting_shipments:
                rospy.loginfo("Number of KITTING shipments to do {0}".format(len(order.kitting_shipments)))
                var = self.procces_kitting_shipment(order.kitting_shipments[0])
                #print(var)
            elif order.assembly_shipments:
                rospy.loginfo("Number of ASSEMBLY shipments to do {0}".format(len(order.assembly_shipments)))
                
            
        #self.end_competition()

if __name__ == '__main__':
    sen = Sensors_functions()
    
    n = MainNode()
    print(sen.get_object_pose_in_workcell())
    n.run()
    #var = RobotMover()

