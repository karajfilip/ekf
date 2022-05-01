#!/usr/bin/env python

import rospy
import math
from node import MainNode
from Sensors import Sensors_functions
from Actuators import Actuators
from pick_and_place import RobotMover
from path_planning import GantryPlanner

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)
    
    pr_m = MainNode()
    sen = Sensors_functions()
    act = Actuators()
    rm = RobotMover()
    gp = GantryPlanner()
    #print(sen.get_object_pose_in_workcell()[1].type)
    
    objects = sen.get_object_pose_in_workcell()
    # Get the pose of tray from sensors
    for tray in objects:
        if (tray.type == "movable_tray_dark_wood"):
        	tray_pose_dark_wood = tray.pose
        elif(tray.type == "movable_tray_light_wood"):
        	tray_pose_light_wood = tray.pose
        elif(tray.type == "movable_tray_metal_rusty"):
        	tray_pose_metal_rusty = tray.pose
        elif(tray.type == "movable_tray_metal_shiny"):
        	tray_pose_metal_shiny = tray.pose



    pr_m.start_competition()
    
    while not pr_m.received_order:
        rospy.loginfo("Waiting for order...")
        pass

    for order in pr_m.orders:
        if order.kitting_shipments:
            rospy.loginfo("Number of KITTING shipments to do {0}".format(len(order.kitting_shipments)))
            var = pr_m.procces_kitting_shipment(order.kitting_shipments[0])
            print(order)
            # Namjesti tray
            for tray in objects:
                if (order.kitting_shipments[0].movable_tray.movable_tray_type == tray.type):
            	    
            	    #provjeri koji gripper ima gantry i po potrebi ga zamijeni
            	    if(act.gripper_type().data != order.kitting_shipments[0].movable_tray.gripper):
            	        print("Razlikuju se")
                        rm.move_directly_gantry([-4.04, 2, 1.8, 0, math.pi/2, 0])
                        rospy.sleep(2)

                        gp.move(5,0)
            	        #rm.move_directly_gantry([-4.3, 7.20, 1.33, 0.0, math.pi/2, 0.0])
                        rospy.sleep(5) # TODO pozicija i while
            	        print("gantry je iznad gripper stationa.")

                        rospy.sleep(1) #TODO rijesi ovo s pozicijom i whileom
                        
                        print(act.gripper_type().data)
                        try:
                            act.change_gripper(str(order.kitting_shipments[0].movable_tray.gripper))
                            rospy.logerr(act.gripper_type().data)
                        except rospy.ServiceException as exc:
                            print(str(exc))

                        while(act.gripper_type().data != order.kitting_shipments[0].movable_tray.gripper):
                            rospy.sleep(0.2)



            	    # posalji gantrija iznad traya kojeg treba pokupiti
                    tray_pose_x = tray.pose.position.x
                    tray_pose_y = tray.pose.position.y
                    tray_pose_z = tray.pose.position.z
                    tray_pose_z_offset = tray.pose.position.z + 0.2
                    rm.move_directly_gantry([tray_pose_x, tray_pose_y, tray_pose_z_offset, 0, math.pi/2, 0])
                    act.activate_gantry_gripper()
                    rospy.sleep(2)
                    rm.move_directly_gantry([tray_pose_x, tray_pose_y, tray_pose_z, 0, math.pi/2, 0])
                    rospy.sleep(2)

                    # pickup
                    #act.activate_gantry_gripper()
                    #rospy.sleep(2)
                    if (act.is_object_attached_gantry()):
                        rospy.loginfo("Predmet zakacen")





            	    # posalji gantrija u home/binL/binR tocku
                    rm.move_directly_gantry([-3.3, 2.96, 1.2, 0.0, math.pi/2, 0.0])
                    rospy.sleep(7)

                    rm.move_directly_gantry([-2.3, 2.96, 1.2, 0.0, math.pi/2, 0.0])
                    rospy.sleep(5.5)

            	    # stavi tray na agv
                    rm.move_directly_gantry([-2.26, 4.68, 1.2, 0.0, math.pi/2, 0.0])
                    rospy.sleep(5)
                    rm.move_directly_gantry([-2.26, 4.68, 0.9, 0.0, math.pi/2, 0.0])
                    rospy.sleep(5)
                    act.deactivate_gantry_gripper()

                    rm.move_directly_gantry([-2.26, 4.68, 1.5, 0.0, math.pi/2, 0.0])


            # Prebaci predmet iz bina na AGV
            #for i in pr_m.procces_kitting_shipment(order.kitting_shipments[0]):
            #	for j in sen.get_object_pose_in_workcell():

            #        if (i["type"] == j.type):
            #        	print("uspio")





        elif order.assembly_shipments:
            rospy.loginfo("Number of ASSEMBLY shipments to do {0}".format(len(order.assembly_shipments)))
                
            
    #self.end_competition()


