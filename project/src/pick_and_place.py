#!/usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectory
#from nist_gear.msg import VacuumGripperState
#from nist_gear.msg import VacuumGripperControl
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from Actuators import Actuators
#from nist_gear.msg import VacuumGripperState
from nist_gear.msg import *
from geometry_msgs.msg import Pose, PoseArray
import copy

# PROBAJ POSLATI NA OVU TOCKU
# [3.1886361434562973,
# 0.5410344923603474,
# -2.6265928652837984,
# 4.510746942464875,
# -5.025746736125955,
# -2.111830819017567,
# -1.6166671603447301e-09,
# 5.638986859679562e-07]

class RobotMover:
    def __init__(self):
        self.gantry_torso_cmd = rospy.Publisher("/ariac/gantry/gantry_controller/command", JointTrajectory, queue_size=2)
        self.gantry_cmd = rospy.Publisher("/ariac/gantry/gantry_arm_controller/command", JointTrajectory, queue_size=2)
        self.kitting_cmd = rospy.Publisher("/ariac/kitting/kitting_arm_controller/command", JointTrajectory, queue_size=2)

        self.inverse_kin = Actuators()
        self.kitting_grip_state = rospy.Subscriber("ariac/kitting/arm/gripper/state", VacuumGripperState, self.vacuum_kitting_state_cb, queue_size=1)
        self.gantry_grip_state = rospy.Subscriber("ariac/gantry/arm/gripper/state", VacuumGripperState, self.vacuum_gantry_state_cb, queue_size=1)
        self.track_sb = rospy.Subscriber("ariac/pose_on_track", PoseArray, self.track_pose_cb, queue_size=1)

        self.track_poses = PoseArray()
        self.watching_gantry = False
        self.watching_kitting = False
        self.kitting_pickedup = False
        self.gantry_pickedup = False

        self.flip_num = 0

    # Ako se radi pickup, gledaj state grippera. Kad uhvati objekt, prestani micat robota
    def vacuum_kitting_state_cb(self, data):
        global picked_up
        if self.watching_kitting:
            if data.attached:
                self.cancel_kitting()
                self.watching_kitting = False
                picked_up = True
                self.kitting_pickedup = True
        else:
            return

    def vacuum_gantry_state_cb(self, data):
        global picked_up
        if self.watching_gantry:
            if data.attached:
                self.cancel_gantry()
                self.watching_gantry = False
                picked_up = True
                self.gantry_pickedup = True
        else:
            return

    def track_pose_cb(self, data):
        self.track_poses = data
        return

    def cancel_gantry(self):
        print("GANTRY_MOVER:ENDED")
        trajectory_arm = JointTrajectory()
        trajectory_arm.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm.header.stamp = rospy.Time.now()

        trajectory_torso = JointTrajectory()
        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()
        
        self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        return

    def cancel_kitting(self):
        print("KITTING_MOVER:ENDED")
        trajectory = JointTrajectory()
        trajectory.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.gantry_cmd.publish(trajectory)
        self.gantry_cmd.publish(trajectory)
        self.gantry_cmd.publish(trajectory)


        trajectory = JointTrajectory()
        trajectory.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.gantry_torso_cmd.publish(trajectory)
        self.gantry_torso_cmd.publish(trajectory)
        self.gantry_torso_cmd.publish(trajectory)
        return

    # Je li robot dosao na poslanu poziciju? Tolerancija +- 0.05. Radi samo za xyz pozicije, nema orijentacije
    def check_kitting_position(self,position, tolerance=0.02):
        x = position[0]
        y = position[1]
        z = position[2]

        kpos = self.inverse_kin.direct_kinematics_kitting_arm()
        robx = kpos[0]
        roby = kpos[1]
        robz = kpos[2]

        if abs(x - robx) < tolerance:
            if abs(y - roby) < tolerance:
                if abs(z - robz) < tolerance:
                    return True

        return False

    def check_gantry_position(self,position, tolerance=0.04):
            x = position[0]
            y = position[1]
            z = position[2]

            kpos = self.inverse_kin.direct_kinematics_gantry_arm()
            robx = kpos[0]
            roby = kpos[1]
            robz = kpos[2]

            if abs(x - robx) < tolerance:
                if abs(y - roby) < tolerance:
                    if abs(z - robz) < tolerance:
                        return True

            return False

    # Dodavanje tocaka u trajektoriju
    # NE RACUNAJ TOCKE RUCNO JER CES NESTO ZEZNUT
    # KORISTI JOINTOVE SAMO IZ OVE FUNKCIJE, NE MIJENJAJ POREDAK!!!
    # used time - vrijeme potroseno do sad
    # point_time - vrijeme da se trenutni pokret izvrsi
    # VRACA: point_arm, point_torso (samo gantry), used_time, joint_point(za prosljedivati prev_joint gantrya)
    def add_point_kitting(self, position, used_time=0, point_time=1, prev_joints=None):
        joint_point = self.inverse_kin.inverse_kinematics_kitting_arm(position, prev_joints)
        joint_point = [joint_point[3], joint_point[0], joint_point[2], joint_point[1], joint_point[4], joint_point[5], joint_point[6]]
        joint_points = self.inverse_kin.fix_joints_kitting(joint_point, prev_joints)
        #TODO FIX JOINT POINTS

        point = JointTrajectoryPoint()
        point.positions = joint_point
        point.time_from_start = rospy.Duration(used_time + point_time)
        return point, used_time + point_time

    def add_point_gantry(self, position, joints=0, used_time=0, point_time=1, prev_joints=None):
        joint_point = self.inverse_kin.inverse_kinematics_gantry(position, target_group=joints, start_joints=prev_joints)
        #arm_joints = self.inverse_kin.fix_joints_gantry(joint_point, prev_joints)
        arm_joints = joint_point[3:-1]
        torso_joints = joint_point[0:3]
        del arm_joints[-1]

        point_arm = JointTrajectoryPoint()
        point_arm.positions = arm_joints
        point_arm.time_from_start = rospy.Duration(used_time + point_time)

        point_torso = JointTrajectoryPoint()
        point_torso.positions = torso_joints
        point_torso.time_from_start = rospy.Duration(used_time + point_time)

        return point_arm, point_torso, used_time + point_time, joint_point

    # Radi trajektoriju za robote
    # Gantry -- ako ne zelis micat torso, ne salji nista u points_torso
    def make_traj_kitting(self, points):
        trajectory = JointTrajectory()
        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()

        for point in points:
            trajectory.points.append(point)

        return trajectory

    def make_traj_gantry(self, points_arm, points_torso = None):
        trajectory_arm = JointTrajectory()
        trajectory_arm.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm.header.stamp = rospy.Time.now()

        trajectory_torso = JointTrajectory()
        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()

        for point in points_arm:
            trajectory_arm.points.append(point)

        if points_torso is None:
            trajectory_torso = None
        else:
            for point in points_torso:
                trajectory_torso.points.append(point)

        return trajectory_arm, trajectory_torso

    ### Moving torso
    def make_traj_torso(self, points_torso):
        trajectory_torso = JointTrajectory()
        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()
        for point in points_torso:
            trajectory_torso.points.append(point)
        return trajectory_torso

    def move_torso(self, position):

        self.torso_moved = False

        gantry_pose = []
        gantry_pose.append(position[0])
        gantry_pose.append(position[1])
        gantry_pose.append(position[2])
        point = JointTrajectoryPoint()
        point.positions = gantry_pose
        point.time_from_start = rospy.Duration(1.2)
        trajectory_torso = self.make_traj_torso([point])
        self.gantry_torso_cmd.publish(trajectory_torso)
        rospy.sleep(1.25)

    # Dohvaca trenutnu poziciju robota iz direktne kinematike
    # NE KORISTITI SAMO DIREKTNU JER POSTOJI SANSA DA CE FAILAT PA SE MORA POZVATI VISE PUTA DOK NE PRORADI
    def get_pos_kitting(self):
        pos = []
        to_break = 10
        while len(pos) == 0 or pos==[0.0, 0.0, 0.0]:
            try:
                pos = list(self.inverse_kin.direct_kinematics_kitting_arm())
            except:
                to_break -= 1
                if to_break == 0:
                    break
                continue

        return pos

    def get_pos_gantry(self):
        pos = []
        to_break = 10
        while len(pos) == 0 or pos==[0.0, 0.0, 0.0]:
            try:
                pos = list(self.inverse_kin.direct_kinematics_gantry_arm())
            except:
                to_break -= 1
                if to_break == 0:
                    break
                continue
        
        return pos

    # Pickup i place funkcije za gantry i kitting.
    # Pickup pomice robota pazljivo na poziciju, te zatim prima predmet
    # Place radi isto, samo spusta objekt na neku lokaciju. Baca warning ako robot nema nista gripanog.
    # Position = [x,y,z, roll, pitch, yaw]
    def pickup_kitting(self, position, object_name = "xsxsxs"):
        # Razvrsti putanju na tri tocke:
        # 1) Okreni elbow_joint za 0.3 rad, tako ce se ruka dici od trenutne pozicije
        # 2) Pomakni robota 0.5 iznad objekta
        # 3) Aktiviraj gripper, te pomici robota ispod dok ga ne uhvatis

        print("KITTING_MOVER: Pickup from:" + str(position))
        self.kitting_pickedup = False
        used_time = 0

        current_pos = self.get_pos_kitting()
        current_pos[2] += 0.2
        current_pos.append(position[3])
        current_pos.append(position[4])
        current_pos.append(position[5])
        
        above_end = position
        above_end[2] = above_end[2] + 0.25

        p1, used_time = self.add_point_kitting(current_pos, used_time, point_time=0.7, prev_joints=None)
        #if (current_pos[1] >= -0.3 and position[1] <= 0.3) or (current_pos[1] <= 0.3 and position[1] >= -0.3):
            #joint_point = [1.5462708704375743, -1.110562287222693, -1.5487328986142959, 3.2318798776308815, -1.5683342733005696, -1.5707963045250863, 3.2318798776894173]
            
            #p3 = JointTrajectoryPoint()
            #p3.positions = joint_point
            #p3.time_from_start = rospy.Duration(used_time + 1.5)
            #used_time = used_time + 1.5

            #p2, used_time = self.add_point_kitting(above_end, used_time, point_time=1.5, prev_joints=p3.positions)
            #trajectory = self.make_traj_kitting([p1, p2])
            #trajectory = self.make_traj_kitting([p1, p3, p2])
        #else:
        p2, used_time = self.add_point_kitting(above_end, used_time, point_time=1.5, prev_joints=p1.positions)
        trajectory = self.make_traj_kitting([p1, p2])
        self.kitting_cmd.publish(trajectory)

        print("KITTING_MOVER: Sent first")
        while not self.check_kitting_position(above_end):
            rospy.sleep(0.1)
        print("KITTING_MOVER: Calc second")

        # Posalji robota na pickup
        used_time = 0
        self.inverse_kin.activate_kitting_gripper()

        close_to_end = above_end
        close_to_end[2] = close_to_end[2] - 0.15
        p3, used_time = self.add_point_kitting(close_to_end, used_time=used_time, point_time=1, prev_joints=None)


        if "pump" in object_name:
            end_pos = close_to_end
            end_pos[2] = close_to_end[2] - 0.03
            p4, used_time = self.add_point_kitting(end_pos, used_time=used_time, point_time=4, prev_joints=p3.positions)
            trajectory2 = self.make_traj_kitting([p3, p4])

        else:
            end_pos = close_to_end
            end_pos[2] = close_to_end[2] - 0.05
            p4, used_time = self.add_point_kitting(end_pos, used_time=used_time, point_time=4, prev_joints=p3.positions)
            trajectory2 = self.make_traj_kitting([p3, p4])

        self.kitting_cmd.publish(trajectory2)
        print("KITTING_MOVER: Sent second")
        self.watching_kitting = True
        return  

    def place_kitting(self, position):
        print("KITTING_MOVER: Placing at" + str(position))
        # Isti princip kao i pickup, samo baci predment na kraju
        used_time = 0

        current_pos = self.get_pos_kitting()
        current_pos[2] += 0.25
        current_pos.append(position[3])
        current_pos.append(position[4])
        current_pos.append(position[5])

        above_end = position
        above_end[2] = above_end[2] + 0.3


        p1, used_time = self.add_point_kitting(current_pos, used_time, point_time=0.5, prev_joints=None)
        #if (current_pos[1] >= -0.3 and position[1] <= 0.3) or (current_pos[1] <= 0.3 and position[1] >= -0.3):
            #joint_point = [1.5462708704375743, -1.110562287222693, -1.5487328986142959, 3.2318798776308815, -1.5683342733005696, -1.5707963045250863, 3.2318798776894173]
            
            #p4 = JointTrajectoryPoint()
            #p4.positions = joint_point
            #p4.time_from_start = rospy.Duration(used_time + 1.5)
            #used_time = used_time + 1.5
        
            #p2, used_time = self.add_point_kitting(above_end, used_time, point_time=1.5, prev_joints=p4.positions)
            #p3, used_time = self.add_point_kitting(end_pos, used_time, point_time=1, prev_joints=p2.positions)
            #trajectory = self.make_traj_kitting([p1, p2, p3])
            #trajectory = self.make_traj_kitting([p1, p4, p2, p3])   
        #else:
        p2, used_time = self.add_point_kitting(above_end, used_time, point_time=1.5, prev_joints=p1.positions)

        end_pos = above_end
        end_pos[2] = end_pos[2] - 0.17
        p3, used_time = self.add_point_kitting(end_pos, used_time, point_time=1, prev_joints=p2.positions)

        trajectory = self.make_traj_kitting([p1, p2, p3])

        self.kitting_cmd.publish(trajectory)

        print("KITTING_MOVER: Sent trajectory")
        while not self.check_kitting_position(end_pos, tolerance=0.015):
            rospy.sleep(0.2)
            if not self.inverse_kin.is_object_attached_kitting().attached:
                return False
        self.inverse_kin.deactivate_kitting_gripper()
        print("KITTING_MOVER: Let go")
        return True

    # Gantry ima dodatnu varijablu joints, koja determinira koji dio gantry se treba pomaknuti. Za micanje samo baze koristi path_planner.py!!!
    # 0 - Cijeli gantry
    # 1 - samo ruka
    # Liftup odreduje treba li dici ruku prije nego se krene
    # 0 - Nemoj dici ruku
    # 1 - Digni ruku prije bilo cega drugog
    # Tray pickup - dizem li tray? 0 - ne
    def pickup_gantry(self, position, joints=0, liftup=0, tray_pickup = 0, object_name = None):
        print("GANTRY_MOVER: Pickup from: " + str(position))
        self.gantry_pickedup = False
        used_time = 0


        above_end = position
        above_end[2] = above_end[2] + 0.3

        if liftup == 1:
            current_pos = self.get_pos_gantry()
            current_pos[2] = current_pos[2] + 0.3
            current_pos.append(position[3])
            current_pos.append(position[4])
            current_pos.append(position[5])
            pa1, pt1, used_time, p1f = self.add_point_gantry(current_pos, joints, used_time, point_time=0.5, prev_joints=None)
            pa2, pt2, used_time, p2f = self.add_point_gantry(above_end, joints, used_time, point_time=1.5, prev_joints=p1f)
        elif liftup == 0:
            used_time -= 0.5
            pa2, pt2, used_time, p2f = self.add_point_gantry(above_end, joints, used_time, point_time=1.5, prev_joints=None)

        if joints == 0:
            if liftup == 0:
                ta, tt = self.make_traj_gantry([pa2], [pt2])
            elif liftup == 1:
                ta, tt = self.make_traj_gantry([pa1, pa2], [pt1, pt2])
        elif joints == 1:
            if liftup == 0:
                ta, tt = self.make_traj_gantry([pa2])
            elif liftup == 1:
                ta, tt = self.make_traj_gantry([pa1, pa2])

        # Reci robotu da dode iznad objekta
        if joints == 0:
            self.gantry_torso_cmd.publish(tt)
        self.gantry_cmd.publish(ta)
        print("GANTRY_MOVER: Sent first")

        i = 100 # wait for 10 seconds
        while not self.check_gantry_position(above_end):
            i -= 1
            rospy.sleep(0.1)
            if i == 0:
                break
        print("GANTRY_MOVER: Calc second")

        # Posalji robota na pickup
        self.inverse_kin.activate_gantry_gripper()
        used_time = 0

        if tray_pickup:
            above_end[2] -= 0.23
            pa3, pt3, used_time, p3f = self.add_point_gantry(above_end, joints, used_time, point_time=0.5, prev_joints=None)
            above_end[2] -= 0.07
            pa4, pt4, used_time, p4f = self.add_point_gantry(above_end, joints, used_time, point_time=3, prev_joints=p3f)

        elif object_name is not None:
            if 'pump' in object_name:
                above_end[2] -= 0.15
                pa3, pt3, used_time, p3f = self.add_point_gantry(above_end, joints, used_time, point_time=0.5, prev_joints=None)
                above_end[2] -= 0.085
                pa4, pt4, used_time, p4f = self.add_point_gantry(above_end, joints, used_time, point_time=5, prev_joints=p3f)

        else:
            above_end[2] -= 0.15
            pa3, pt3, used_time, p3f = self.add_point_gantry(above_end, joints, used_time, point_time=0.5, prev_joints=None)
            above_end[2] -= 0.12
            pa4, pt4, used_time, p4f = self.add_point_gantry(above_end, joints, used_time, point_time=5, prev_joints=p3f)


        if joints == 0:
            ta2, tt2 = self.make_traj_gantry([pa3, pa4], [pt3, pt4])
        elif joints == 1:
            ta2, tt2 = self.make_traj_gantry([pa3, pa4])

        if joints == 0:
            self.gantry_torso_cmd.publish(tt2)
        self.gantry_cmd.publish(ta2)

        print("GANTRY_MOVER: Sent second")
        self.watching_gantry = True

        return  

    def place_gantry(self, position, joints=0, liftup=1, tolerance=0.04):
        print("GANTRY_MOVER: Placing at: " + str(position))
        self.gantry_pickedup = False
        used_time = 0


        above_end = position
        above_end[2] = above_end[2] + 0.3

        if liftup == 1:
            print("lift1")
            current_pos = self.get_pos_gantry()
            current_pos[2] = current_pos[2] + 0.3
            current_pos.append(position[3])
            current_pos.append(position[4])
            current_pos.append(position[5])
            pa1, pt1, used_time, p1f = self.add_point_gantry(current_pos, joints, used_time, point_time=0.5, prev_joints=None)
            pa2, pt2, used_time, p2f = self.add_point_gantry(above_end, joints, used_time, point_time=1.5, prev_joints=p1f)
        else:
            print("lift0")
            used_time -= 0.5
            pa2, pt2, used_time, p2f = self.add_point_gantry(above_end, joints, used_time, point_time=1.5, prev_joints=None)


        above_end[2] = above_end[2] - 0.15
        pa3, pt3, used_time, p3f = self.add_point_gantry(above_end, joints, used_time, point_time=1.5, prev_joints=p2f)

        if joints == 0:
            if liftup == 0:
                ta, tt = self.make_traj_gantry([pa2, pa3], [pt2, pt3])
            elif liftup == 1:
                ta, tt = self.make_traj_gantry([pa1, pa2, pa3], [pt1, pt2, pt3])
        elif joints == 1:
            if liftup == 0:
                ta, tt = self.make_traj_gantry([pa2, pa3])
            elif liftup == 1:
                ta, tt = self.make_traj_gantry([pa1, pa2, pa3])

        # Reci robotu da dode iznad objekta
        if joints == 0:
            self.gantry_torso_cmd.publish(tt)
        self.gantry_cmd.publish(ta)
        print("GANTRY_MOVER: Sent first")

        i = 85 # wait for 8.5 seconds
        while not self.check_gantry_position(above_end, tolerance=tolerance):
            #print("trying to reach end position")
            rospy.sleep(0.1)
            i -= 1
            if i == 0:
                break
        self.inverse_kin.deactivate_gantry_gripper()
        print("GANTRY_MOVER: Let go")
        return

    def assemble_place_gantry(self, position, object_name, joints=0, liftup=1, tolerance=0.01):
        print("GANTRY_MOVER: Placing at: " + str(position))
        self.gantry_pickedup = False
        used_time = 0

        above_end = position
        if 'sensor' in object_name:
            above_end[0] = above_end[0] + 0.2
        else:
            above_end[2] = above_end[2] + 0.22

        if liftup == 1:
            current_pos = self.get_pos_gantry()
            current_pos[2] = current_pos[2] + 0.15
            current_pos.append(position[3])
            current_pos.append(position[4])
            current_pos.append(position[5])
            pa1, pt1, used_time, p1f = self.add_point_gantry(current_pos, joints, used_time, point_time=0.5,
                                                             prev_joints=None)
            pa2, pt2, used_time, p2f = self.add_point_gantry(above_end, joints, used_time, point_time=1.5,
                                                             prev_joints=p1f)
        else:
            used_time -= 0.5
            pa2, pt2, used_time, p2f = self.add_point_gantry(above_end, joints, used_time, point_time=2,
                                                             prev_joints=None)

        if 'sensor' in object_name:
            above_end[0] = above_end[0] - 0.1
        else:
            above_end[2] = above_end[2] - 0.1
        pa3, pt3, used_time, p3f = self.add_point_gantry(above_end, joints, used_time, point_time=2.5,
                                                         prev_joints=p2f)


        if 'sensor' in object_name:
            above_end[0] = above_end[0] - 0.1
        else:
            above_end[2] = above_end[2] - 0.12

        pa4, pt4, used_time, p3f = self.add_point_gantry(above_end, joints, used_time, point_time=3.5,
                                                         prev_joints=p3f)

        if joints == 0:
            if liftup == 0:
                ta, tt = self.make_traj_gantry([pa2, pa3, pa4], [pt2, pt3, pt4])
            elif liftup == 1:
                ta, tt = self.make_traj_gantry([pa1, pa2, pa3, pa4], [pt1, pt2, pt3, pt4])
        elif joints == 1:
            if liftup == 0:
                ta, tt = self.make_traj_gantry([pa2, pa3, pa4])
            elif liftup == 1:
                ta, tt = self.make_traj_gantry([pa1, pa2, pa3, pa4])

        # Reci robotu da dode iznad objekta
        if joints == 0:
            self.gantry_torso_cmd.publish(tt)
        self.gantry_cmd.publish(ta)
        print("GANTRY_MOVER: Sent first")

        i = 85 # wait for 8.5 seconds
        while not self.check_gantry_position(above_end, tolerance=tolerance):
            rospy.sleep(0.1)
            i -= 1
            if i == 0: # if not placed within 8.5 seconds, ABORT
                break
        self.inverse_kin.deactivate_gantry_gripper()

        if 'regulator' in object_name:
            self.move_directly_gantry([above_end[0] + 0.5, above_end[1], above_end[2] + 0.15, -math.pi/2, math.pi/2, 0])
        elif 'sensor' in object_name:
            self.move_directly_gantry([above_end[0], above_end[1] - 0.35, above_end[2], 0, 0, math.pi/2])
            self.move_directly_gantry([above_end[0], above_end[1] - 0.35, above_end[2] + 0.2, 0, math.pi/2, 0])
        elif 'pump' in object_name or 'battery' in object_name:
            self.move_directly_gantry([above_end[0] + 0.1, above_end[1], above_end[2] + 0.15, 0, math.pi/2, 0])

        print("GANTRY_MOVER: Let go")
        return

    # Funkcije koje direktno micu robota na neku poziciju. Ne uzimaju objekt niti pokusavaju raditi ikakav obstacle avoidance. Ne koristiti osim za priblizno pozicioniranje
    def move_directly_kitting(self, position):
        print("KITTING_MOVER: Moving to: " + str(position))
        inv_position = self.inverse_kin.inverse_kinematics_kitting_arm(position)
        inv_position = [inv_position[3], inv_position[0], inv_position[2], inv_position[1], inv_position[4], inv_position[5], inv_position[6], inv_position[7]]
        dir_pos = self.inverse_kin.direct_kinematics_kitting_arm()
        print(dir_pos)
        trajectory = JointTrajectory()

        # Napravi point
        point = JointTrajectoryPoint()
        point.positions = inv_position
        point.time_from_start = rospy.Duration(1)
        trajectory.points.append(point)

        # Ispuni trajectory
        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory)
        print("KITTING_MOVER: Sent")
        return

    def move_directly_gantry(self, position, joints = 0):
        print("GANTRY_MOVER: Moving to: " + str(position))
        pa1, pt1, used_time, p1f = self.add_point_gantry(position, joints, used_time=0, point_time=2, prev_joints = None)
        
        if joints == 0:
            ta, tt = self.make_traj_gantry([pa1], [pt1])
        elif joints == 1:
            ta, tt = self.make_traj_gantry([pa1])
        if joints == 0:
            self.gantry_torso_cmd.publish(tt)
        self.gantry_cmd.publish(ta)
        print("GANTRY_MOVER: Sent")
        return

    # Funkcija za assembleanje u briefcase
    def assemble_gantry(self, station_name, object_name, joints=0, z_rot = 0, offset = None, multiplier = 1):
        base_frame = []
        transf = []

        if station_name == 'as1':
            base_frame = [-7.22, 3.09, 1.31]
        elif station_name == 'as2':
            base_frame = [-12.22, 3.09, 1.31]
        elif station_name == 'as3':
            base_frame = [-7.22, -2.91, 1.31]
        elif station_name == 'as4':
            base_frame = [-12.22, -2.91, 1.31]

        rot = [0, math.pi/2, 0]
        if "sensor" in object_name:
            transf = [0.382, 0.12, -0.002]
            rot=[0, 0.02, math.pi/2]
        elif 'regulator' in object_name:
            transf = [-0.217 + 0.037, -0.1532 - 0.015, 0.103]
            if station_name == "as4" or station_name == "as2":
                transf[1] += 0.008
            rot = [-math.pi/2, math.pi, 0]
        elif 'battery' in object_name:
            transf = [-0.033465, 0.174845, 0.02]
            rot = [0, math.pi/2, math.pi/2]
        elif 'pump' in object_name:
            transf = [0.033085, -0.152835, 0.045]
            rot = [0, math.pi/2, math.pi/2]

        pos_place = []
        if offset is None:
            pos_place.append(base_frame[0] + transf[0])
            pos_place.append(base_frame[1] + transf[1])
            pos_place.append(base_frame[2] + transf[2])
        else:
            pos_place.append(base_frame[0] + transf[0] - offset[0])
            pos_place.append(base_frame[1] + transf[1] - offset[1])
            pos_place.append(base_frame[2] + transf[2])
        pos_place = pos_place + rot
        #return pos_place

        self.assemble_place_gantry(pos_place, joints=joints, tolerance=0.004, liftup=0, object_name=object_name)

        return

    # Funkcija za pokupljavanje sa trake pomocu kitting robota. Pozovi i on ide na predmet odredenog indexa (default 0)
    def pickup_from_track(self, pose_array_i=0):
        rospy.logwarn("Pickup from track")
        rospy.sleep(0.2)
        change_t = False
        offset_change_limit = 10

        i = 0
        for track_pos in self.track_poses.poses:
            if track_pos.position.y > 1.5:
                pose_array_i = i
            i += 1

        while not self.track_poses.poses:
            rospy.sleep(0.01)
        current_pose = self.track_poses.poses[pose_array_i].position
        while current_pose.y < -1:
            pose_array_i += 1
            current_pose = self.track_poses.poses[pose_array_i].position

        # 1) Dodi ispred predmeta
        current_position = [current_pose.x + 0.01, current_pose.y - 1, current_pose.z + 0.5, 0, math.pi / 2, 0]
        print(current_position)
        preposition = self.inverse_kin.inverse_kinematics_kitting_arm(current_position)
        print("prepostion" + str(preposition))
        preposition = [preposition[3], preposition[0], preposition[2], preposition[1], preposition[4], preposition[5],
                       preposition[6], preposition[7]]

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = preposition
        point.time_from_start = rospy.Duration(0.5)
        trajectory.points.append(point)

        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint",
                                  "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory)

        while not self.check_kitting_position(current_position):
            rospy.sleep(0.05)
        print("got to position")

        self.inverse_kin.activate_kitting_gripper()
        self.watching_kitting = True
        z_offset = 0.11
        y_offset = -0.035
        offset_change = 0
        while not self.kitting_pickedup:
            rospy.logwarn(z_offset)
            rospy.logwarn(y_offset)
            current_pose = self.track_poses.poses[pose_array_i].position
            kpos = self.inverse_kin.direct_kinematics_kitting_arm()

            current_position = [current_pose.x, current_pose.y + y_offset, current_pose.z + z_offset, 0, math.pi / 2, 0]

            if ((kpos[2] - current_position[2]) < z_offset + 0.02) and (abs(kpos[1] - current_position[1]) < 0.1):
                offset_change += 1
                if offset_change > offset_change_limit:
                    offset_change = 0
                    if z_offset == 0.11:
                        offset_change_limit = 20
                        change_t = True
                        z_offset = 0.0995
                        y_offset = -0.034
                    elif z_offset == 0.0995:
                        z_offset = 0.054
                        y_offset = -0.025
                    elif z_offset == 0.054:
                        z_offset = 0.0535
                        y_offset = -0.03
                    elif z_offset == 0.0535:
                        z_offset = 0.043
                        y_offset = -0.035
            # print("offset=" + str(z_offset))

            diff = math.sqrt((kpos[1] - current_position[1]) ** 2 + (kpos[2] - current_position[2]) ** 2)
            t = diff / 0.4
            if t > 0.8:
                t = 0.5
            if change_t:
                t = 0.1
            # print(diff)
            # print(t)

            preposition = self.inverse_kin.inverse_kinematics_kitting_arm(current_position)
            preposition = [preposition[3], preposition[0], preposition[2], preposition[1], preposition[4],
                           preposition[5], preposition[6], preposition[7]]
            point = JointTrajectoryPoint()
            trajectory = JointTrajectory()
            point.positions = preposition
            point.time_from_start = rospy.Duration(t)
            trajectory.points.append(point)
            trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint",
                                      "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            trajectory.header.stamp = rospy.Time.now()
            self.kitting_cmd.publish(trajectory)
            rospy.sleep(0.04)

        current_position = [current_pose.x, current_pose.y, current_pose.z + 0.5, 0, math.pi / 2, 0]
        print(current_position)
        preposition = self.inverse_kin.inverse_kinematics_kitting_arm(current_position)
        preposition = [preposition[3], preposition[0], preposition[2], preposition[1], preposition[4], preposition[5],
                       preposition[6], preposition[7]]

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = preposition
        point.time_from_start = rospy.Duration(1.5)
        trajectory.points.append(point)

        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint",
                                  "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory)

        return

    # Zadajes poziciju predmeta kojeg treba flipat i funkcija napravi proces
    def flip_part_kitting(self, position):
        rospy.logwarn("Flip part called")
        rospy.logwarn(position)
        self.flip_num += 1
        x_offset = 0
        y_offset = 0
        position[2] += 0.04
        if self.flip_num == 1:
            rospy.logwarn("New pick up")
            rospy.logwarn(position)
            self.pickup_kitting(copy.deepcopy(position))
            while not self.kitting_pickedup:
                rospy.sleep(0.2)
            if self.flip_num == 2 or self.flip_num == 3:
                x_offset = 0.065
            #elif self.flip_num == 4:
             #   x_offset = -0.065
            print("KITTING_MOVER: Pickup from:" + str(position))
            self.kitting_pickedup = False
            used_time = 0

            current_pos = self.get_pos_kitting()
            current_pos[2] += 0.3
            current_pos.append(position[3])
            current_pos.append(position[4])
            current_pos.append(position[5])

            # pi/2 u y, pi/2 u z, pi/2 u z, -pi/2 u y
            above_end = copy.deepcopy(position)
            above_end[2] = above_end[2] + 0.3
            if self.flip_num == 1:
                above_end[5] = above_end[5] + math.pi / 2
            elif self.flip_num == 2:
                above_end[4] = above_end[4] + math.pi / 2
            elif self.flip_num == 3:
                above_end[4] = above_end[4] + math.pi / 2
            elif self.flip_num == 4:
                above_end[5] = above_end[5] - math.pi / 2

            p1, used_time = self.add_point_kitting(current_pos, used_time, point_time=2.2, prev_joints=None)
            p2, used_time = self.add_point_kitting(above_end, used_time, point_time=2.3, prev_joints=p1.positions)

            close_to_end = above_end
            close_to_end[0] = close_to_end[0] + x_offset
            close_to_end[1] = close_to_end[1] + y_offset
            close_to_end[2] = close_to_end[2] - 0.2
            # if self.flip_num == 1 or self.flip_num == 2:
            # close_to_end[2] = close_to_end[2] - 0.08
            p3, used_time = self.add_point_kitting(close_to_end, used_time=used_time, point_time=1.2, prev_joints=None)

            trajectory = self.make_traj_kitting([p1, p2, p3])
            self.kitting_cmd.publish(trajectory)

            while (not self.check_kitting_position(close_to_end, tolerance=0.01)) or (
            not self.check_kitting_rotation(close_to_end, tolerance=0.01)):
                rospy.sleep(0.1)
            rospy.sleep(0.2)
            rospy.logwarn(self.flip_num)
            self.inverse_kin.deactivate_kitting_gripper()
            rospy.sleep(0.5)
            print("KITTING_MOVER: Let go (flip)")
            self.move_directly_kitting(position+[0, 0, 0.3, 0, 0, 0])
            if self.flip_num < 4:
                rospy.logwarn("New pick up")
                rospy.logwarn(position)
                self.pickup_kitting(copy.deepcopy(position))
            while not self.kitting_pickedup:
                rospy.sleep(0.2)
            #self.kitting_pickedup = False
            rospy.logwarn(position)
            self.flip_part_kitting(position)
        rospy.logwarn("flipping done")
        return
    
    def check_kitting_rotation(self, position, tolerance=0.05):
        position = self.inverse_kin.euler_to_quaternion(position[3], position[4], position[5])
        x = position[0]
        y = position[1]
        z = position[2]
        w = position[3]

        kpos = self.inverse_kin.rotation_kitting_arm()
        robx = kpos[0]
        roby = kpos[1]
        robz = kpos[2]
        robw = kpos[3]
        print("check")
        print(position)
        print(kpos)
        print("check")

        if abs(x - robx) < tolerance:
            if abs(y - roby) < tolerance:
                if abs(z - robz) < tolerance:
                    if abs(w - robw) < tolerance:
                        return True

        return False

#rospy.init_node("roboter")
#rm = RobotMover()
#pi = math.pi
#rospy.sleep(0.5)
#rm.move_directly_gantry([-5.601358644426888, -1.1331032844026437, 1.3889896286826781, 0, 1.5707963267948966, 0], joints=1)

#GANTRY_MOVER: Moving to: [-5.601358644426888, -1.4331032844026437, 1.5889896286826781, 0, 1.5707963267948966, 0]
#GANTRY_MOVER: Moving to: [-5.401358644426888, -1.1331032844026436, 1.538989628682678, -1.5707963267948966, 0, -1.5707963267948966]
#GANTRY_MOVER: Moving to: [-5.401358644426888, -0.9331032844026437, 1.538989628682678, -3.141592653589793, 0, -3.141592653589793]

#self.rm.move_directly_gantry([ud.partcurrentpose.position.x + 0.2, ud.partcurrentpose.position.y - (0.3 * multiplier), ud.partcurrentpose.position.z + 0.75, pi / 2 * multiplier, 0, pi / 2 * multiplier], 1)

#self.rm.move_directly_gantry([ud.partcurrentpose.position.x + 0.2, ud.partcurrentpose.position.y - (0.5 * multiplier), ud.partcurrentpose.position.z + 0.75, pi / 2 * (multiplier - 1), 0, pi * multiplier], 1)

# rospy.sleep(1.5)
# self.rm.move_directly_gantry([ud.partcurrentpose.position.x + 0.2, ud.partcurrentpose.position.y - (0.3 * multiplier), ud.partcurrentpose.position.z + 0.75, 0, pi/2 * (multiplier - 1), pi * multiplier], 1)


#print(rm.assemble_gantry('as2', 'battery', joints=0))


#rm.assemble_gantry('as2', 'pump', joints=0)
#while not rm.gantry_pickedup:
#    rospy.sleep(0.2)
#rm.move_directly_gantry([-3.415, 0.607, 0.9, 0, pi/2, 0], joints=1)
#rm.place_kitting([-2.265,1.37 , 0.8 , 0 , math.pi/2 , 0])

#while rm.watching_kitting:
#    rospy.sleep(0.2)
#rm.pickup_kitting([-1.79, 2.6655014024893363, 0.77965688566175, 0, 1.5707963267948966, 0])
#while not rm.kitting_pickedup:
#    rospy.sleep(0.2)
#rm.place_kitting([-2.265,1.37 , 0.8 , 0 , math.pi/2 , 0])

#rm.flip_part_kitting([-1.79, 2.465, 0.774, 0, math.pi/2, 0])
#rospy.spin()

