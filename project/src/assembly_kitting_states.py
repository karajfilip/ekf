from json.tool import main
import rospy
import smach
import pick_and_place
import process_management
import path_planning
import Actuators
from Sensors import Sensors_functions
from geometry_msgs.msg import Pose
from math import pi

class CheckPart(smach.State):
    def __init__(self, outcomes=['noParts', 'newPart'], input_keys=['task'], output_keys=['part']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.i = 0

    def execute(self, ud):
        if self.i < len(self.products):
            ud.part = self.products[self.i]
            self.i += 1
            return 'newPart'
        else:
           return 'noParts'

## ASSEMBLY STATES
class CheckGripper(smach.State):
    def __init__(self, outcomes=['changegripper', 'next'], input_keys=['task'], output_keys=['gripper']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.act = Actuators.Actuators()

    def execute(self, ud):
        gripper = self.act.gripper_type
        if str(gripper) !=  'gripper_part':
            ud.gripper = 'gripper_tray'
            return 'gripper_part'
        else:
            return 'next'

class SendGantry(smach.State):
    def __init__(self, outcomes=['arrived'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.gp = path_planning.GantryPlanner()
        self.act = Actuators.Actuators()
        self.rm = pick_and_place.RobotMover()

    def execute(self, ud):
        self.gp.move(ud.task.station_id)
        return 'arrived'

class SubmitAssemblyShipment(smach.State):
    def __init__(self, outcomes=['success'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = process_management.process_management()
    
    def execute(self, ud):
        self.node.submit_assembly_shipment(ud.task.station_id)
        return 'success'

class FindPartOnTray(smach.State):
    def __init__(self, outcomes=['found'], input_keys=['part', 'kittingtask'], output_keys=['partcurrentposition']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)

    def execute(self, ud):
        if ud.kittingtask:
            for product in ud.kittingtask.products:
                if product.type == ud.part.type:
                    ud.partcurrentposition = product.pose.position
                    return 'found'
    
class GantryMovePart(smach.State):
    def __init__(self, outcomes=['moved'], input_keys=['partcurrentposition', 'part', 'task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.rm = pick_and_place.RobotMover()
        self.sen = Sensors_functions()
    
    def execute(self, ud):
        pose_tray = self.sen.tf_transform(str("kit_tray_"+str((ud.task.agv)[-1])))
        part_curr_pose = Pose()
        part_curr_pose.position.x = ud.partcurrentposition.x + pose_tray.position.x
        part_curr_pose.position.y = ud.partcurrentposition.y + pose_tray.position.y
        part_curr_pose.position.z = ud.partcurrentposition.z + pose_tray.position.z
        self.rm.pickup_gantry([part_curr_pose.position.x, part_curr_pose.position.y, part_curr_pose.position.z, part_curr_pose.orientation.x, part_curr_pose.orientation.y, part_curr_pose.orientation.z])
        pose_briefase = self.sen.tf_transform(str("briefcase_"+str((ud.task.assembly_station)[-1])))
        part_pose = Pose()
        part_pose.position.x = ud.part.pose.position.x + pose_briefase.position.x
        part_pose.position.y = ud.part.pose.position.y + pose_briefase.position.y
        part_pose.position.z = ud.part.pose.position.z + pose_briefase.position.z
        self.rm.place_gantry([part_pose.position.x, part_pose.position.y, part_pose.position.z, part_pose.orientation.x, part_pose.orientation.y, part_pose.orientation.z])
        return 'moved'

## KITTING STATES
class CheckAGV(smach.State):
    def __init__(self, outcomes=['agvatks', 'agvnotatks'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = process_management.process_management()

    def execute(self, ud):
        if (str(self.node.get_position_AGV(ud.task.agv)))[:-1] != 'ks':
            return 'agvnotatks'
        else:
            return 'agvatks'

class SendAGV(smach.State):
    def __init__(self, outcomes=['agvatks'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.node = process_management.process_management()

    def execute(self, ud):
        self.node.move_AGV(ud.task.agv, 'ks')
        return super().execute(ud)

class CheckMoveableTray(smach.State):
    def __init__(self, outcomes=['changegripper', 'changetray'], input_keys=['task'], output_keys=['gripper']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.act = Actuators.Actuators()

    def execute(self, ud):
        gripper = self.act.gripper_type
        if str(gripper) !=  'gripper_tray':
            ud.gripper = 'gripper_tray'
            return 'changegripper'
        else:
            return 'changetray'

class GetGripper(smach.State):
    def __init__(self, outcomes=['gripperon'], input_keys=['gripper']):
        smach.State.__init__(self, outcomes, input_keys)
        self.rm = pick_and_place.RobotMover()
        self.gp = path_planning.GantryPlanner()
        self.act = Actuators.Actuators()
    
    def execute(self, ud):   ##################### poboljsati?     kopija iz main.py
        curr_pose = self.act.direct_kinematics_gantry_arm()
        self.rm.pickup_gantry([curr_pose[0], curr_pose[1], curr_pose[2]+0.2, 0, pi/2, 0], 1)
        self.gp.move('gripperstation')
        rospy.sleep(5)  # TODO pozicija i while
        print("gantry je iznad gripper stationa.")

        rospy.sleep(1)  # TODO rijesi ovo s pozicijom i whileom

        print(self.act.gripper_type)
        try:
            self.act.change_gripper(
                str(ud.gripper))
            rospy.logerr(self.act.gripper_type)
        except rospy.ServiceException as exc:
            print(str(exc))

        while(self.act.gripper_type != ud.gripper):
            rospy.sleep(0.2)
        return 'gripperon'

class GantryGetTray(smach.State):
    def __init__(self, outcomes=['trayon'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.gp = path_planning.GantryPlanner()
        self.rm = pick_and_place.RobotMover()
        self.sen = Sensors_functions()
        self.act = Actuators.Actuators()

    def execute(self, ud):
        self.gp.move('traystation')
        self.objects = self.sen.get_object_pose_in_workcell()
        for tray in self.objects:
            if tray.type == ud.task.movable_tray.movable_tray_type:
                self.rm.pickup_gantry([tray.pose.position.x, tray.pose.position.y, tray.pose.position.z, tray.pose.orientation.x, tray.pose.orientation.y, tray.pose.orientation.z])
                self.gp.move(ud.task.agv)
                agv_pose = self.sen.tf_transform(str("kit_tray_"+str((ud.task.agv)[-1])))
                self.rm.place_gantry([agv_pose.position.x, agv_pose.position.y, agv_pose.position.z, agv_pose.orientation.x, agv_pose.orientation.y, agv_pose.orientation.z])
                return 'trayon'

class FindPartInEnvironment(smach.State):
    def __init__(self, outcomes=['found'], input_keys=['part'], output_keys=['partposition', 'partcurrentposition']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.sen = Sensors_functions()

    def execute(self, ud):
        objects = self.sen.get_object_pose_in_workcell()
        ud.partposition = ud.part.pose.position
        for product in objects:
            if product.type == ud.part.type:
                ud.partcurrentposition = product.pose.position
                return 'found'

class KittingRobotPickAndPlace(smach.State):
    def __init__(self, outcomes=['success'], input_keys=['partposition', 'partcurrentposition']):
        smach.State.__init__(self, outcomes, input_keys)
        self.rm = pick_and_place.RobotMover()

    def execute(self, ud):
        self.rm.pickup_kitting(ud.partcurrentposition)
        self.rm.place_kitting(ud.partposition)
        return 'success'

class CheckFaulty(smach.State):
    def __init__(self, outcomes=['faulty', 'notfaulty'], input_keys=['part']):
        smach.State.__init__(self, outcomes, input_keys)

    def execute(self, ud):
        return super().execute(ud)

class SubmitKittingShipment(smach.State):
    def __init__(self, outcomes=['success'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = process_management.process_management()
    
    def execute(self, ud):
        self.node.submit_kitting_shipment(ud.task.agv, ud.task.assembly_station, ud.task.shipment_type)
        return 'success'