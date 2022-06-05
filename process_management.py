#!/usr/bin/env python

import rospy
import yaml
import math

from std_srvs.srv import Trigger
from nist_gear.srv import AGVControl,  ConveyorBeltControl, AssemblyStationSubmitShipment, SubmitKittingShipment, MoveToStation

from geometry_msgs.msg import TransformStamped
from nist_gear.msg import Orders, Model
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

#from Actuators import Actuators
#from pick_and_place import RobotMover
from Sensors import Sensors_functions
from nist_assembly import AssemblyPart

class process_management():

    def __init__(self):
        #rospy.init_node('main_node', anonymous=True)
        self.order = Orders()
        self.orders = []
        self.orders_served = 0
        self.received_order = False
        self.interrupted = None
        self.order_id = Orders()
        self.kitting_info = Orders()
        self.assembly_info = Orders()
        self.order_sub = rospy.Subscriber('/ariac/orders', Orders, self.get_order)
        self.placed = dict()
        self.remove = list()
        self.skip = list()
        self.agvtrays = dict()

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
        #self.assembly_info = msg.assembly_shipments

    def kitting_agv(self):
        return self.order.kitting_shipments[0].agv

    def procces_kitting_shipment(self, kitting_products):
        
        self.agv = self.order.kitting_shipments[0].agv
        self.assembly_station = self.order.kitting_shipments[0].assembly_station

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
        
        self.assembly_station = self.order.assembly_shipments[0].station_id

        parts = []

        for product in assembly_products.products:
            part = {
                "type": product.type,
                "pose_xyz": product.pose.xyz,
                "pose_rpy": product.pose.rpy
            }
            parts.append(part)

        return parts

    def filipov_process_assembly_shipment(self, assembly_shipment):
        # Eventually read from order but build dictionary to start
        parts = []
        name_str = ""
        for product in assembly_shipment.products:
            _, part_type, color = product.type.split("_")

            parts.append(AssemblyPart(part_type, color, product.pose))
            name_str += (color + "_" + part_type + ", ")

        rospy.loginfo("Received assembly order to deliver " + name_str +
                      "to assembly station: " + assembly_shipment.station_id)

        return parts

    def submit_kitting_shipment(self, agv, assembly_station, shipment_type):
        """ ROS Service for submit a kitting shipment."""
        rospy.wait_for_service('/ariac/' + agv + '/submit_kitting_shipment')
        submit_kitting = rospy.ServiceProxy('/ariac/' + agv + '/submit_kitting_shipment', SubmitKittingShipment)
        
        try:
            resp = submit_kitting(assembly_station, shipment_type)
            rospy.loginfo("Submitting a kitting shipment.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))
    
    def submit_assembly_shipment(self, station_id, shipment_type):
        """ ROS Service for submit an assembly shipment."""
        rospy.wait_for_service('/ariac/' + station_id + '/submit_assembly_shipment')
        submit_assembly = rospy.ServiceProxy('/ariac/' + station_id + '/submit_assembly_shipment', AssemblyStationSubmitShipment)

        try:
            resp = submit_assembly(shipment_type)
            rospy.loginfo("Submitting an assembly shipment.")
        except rospy.ServiceException as exc:
            rospy.logerr(str(exc))

    ### Moving AGV
    def get_position_AGV(self, agv):
        return rospy.wait_for_message('/ariac/' + agv + '/station', String).data

    def move_AGV(self, agv, station):
        rospy.wait_for_service('/ariac/' + agv + '/move_to_station')
        moveAGV = rospy.ServiceProxy('/ariac/' + agv + '/move_to_station', MoveToStation)

        try:
            resp = moveAGV(station)
            rospy.loginfo("{0} moved to {1}".format(agv, station))
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
        print("ovdje")
        #position = self.get_position_AGV("agv1")
        #print(position)
        #self.move_AGV("agv1", "as1")
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
        
        rospy.logwarn(self.orders[0].assembly_shipments[0])
        parts = self.filipov_process_assembly_shipment(self.orders[0].assembly_shipments[0])
        print(len(parts))
        rospy.logerr(parts)
        #self.end_competition()

if __name__ == '__main__':
    #sen = Sensors_functions()
    
    n = process_management()
    #print(sen.get_object_pose_in_workcell())
    n.run()
    #var = RobotMover()

