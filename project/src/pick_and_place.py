import rospy
import math
from trajectory_msgs.msg import JointTrajectory
#from nist_gear.msg import VacuumGripperState
#from nist_gear.msg import VacuumGripperControl
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from Actuators import Actuators
from nist_gear.msg import VacuumGripperState
from geometry_msgs.msg import Pose, PoseArray

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
        print("ENDED")
        trajectory = JointTrajectory()
        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory)
        self.kitting_cmd.publish(trajectory)
        self.kitting_cmd.publish(trajectory)
        return

    def cancel_kitting(self):
        print("ENDED")
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
    def check_kitting_position(self,position, tolerance=0.05):
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

    def check_gantry_position(self,position, tolerance=0.05):
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

    # Pickup i place funkcije za gantry i kitting.
    # Pickup pomice robota pazljivo na poziciju, te zatim prima predmet
    # Place radi isto, samo spusta objekt na neku lokaciju. Baca warning ako robot nema nista gripanog.
    # Position = [x,y,z, roll, pitch, yaw]
    def pickup_kitting(self, position):
        # Razvrsti putanju na tri tocke:
        # 1) Okreni elbow_joint za 0.3 rad, tako ce se ruka dici od trenutne pozicije
        # 2) Pomakni robota 0.5 iznad objekta
        # 3) Aktiviraj gripper, te pomici robota ispod dok ga ne uhvatis

        self.kitting_pickedup = False
        inv_start = list(self.inverse_kin.kitting_joint_state.position)
        inv_start[2] = inv_start[2] + 0.3
        del inv_start[4]

        end_position = position
        end_position[2] += 0.3

        inv_end = self.inverse_kin.inverse_kinematics_kitting_arm(end_position, inv_start)
        inv_end = [inv_end[3], inv_end[0], inv_end[2], inv_end[1], inv_end[4], inv_end[5], inv_end[6], inv_end[7]]

        # Napravi trajektoriju
        trajectory = JointTrajectory()

        # Napravi pointove
        point1 = JointTrajectoryPoint()
        point1.positions = inv_start
        point1.time_from_start = rospy.Duration(0.6)
        trajectory.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = inv_end
        point2.time_from_start = rospy.Duration(1.6)
        trajectory.points.append(point2)

        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()

        # Reci robotu da dode iznad objekta
        self.kitting_cmd.publish(trajectory)

        print("sent first")
        while not self.check_kitting_position(end_position):
            rospy.sleep(0.2)
        print("calc second")


        # Posalji robota na pickup
        self.inverse_kin.activate_kitting_gripper()
        trajectory2 = JointTrajectory()

        end_position[2] -= 0.1
        inv_pickup = self.inverse_kin.inverse_kinematics_kitting_arm(end_position)
        inv_pickup = [inv_pickup[3], inv_pickup[0], inv_pickup[2], inv_pickup[1], inv_pickup[4], inv_pickup[5], inv_pickup[6], inv_pickup[7]]
        point3 = JointTrajectoryPoint()
        point3.positions = inv_pickup
        point3.time_from_start = rospy.Duration(1)
        trajectory2.points.append(point3)


        end_position[2] -= 0.2
        inv_pickup = self.inverse_kin.inverse_kinematics_kitting_arm(end_position)
        inv_pickup = [inv_pickup[3], inv_pickup[0], inv_pickup[2], inv_pickup[1], inv_pickup[4], inv_pickup[5], inv_pickup[6], inv_pickup[7]]
        point4 = JointTrajectoryPoint()
        point4.positions = inv_pickup
        point4.time_from_start = rospy.Duration(7)
        trajectory2.points.append(point4)

        trajectory2.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory2.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory2)
        print("sent second")
        self.watching_kitting = True

        return  

    def place_kitting(self, position):
        # Isti princip kao i pickup, samo baci predment na kraju
        inv_start = list(self.inverse_kin.direct_kinematics_kitting_arm())
        inv_start[2] = inv_start[2] + 0.3
        inv_start = self.inverse_kin.inverse_kinematics_kitting(inv_start)
        inv_start = [inv_start[3], inv_start[0], inv_start[2], inv_start[1], inv_start[4], inv_start[5], inv_start[6], inv_start[7]]

        end_position = position
        end_position[2] += 0.3

        inv_end = self.inverse_kin.inverse_kinematics_kitting_arm(end_position, inv_start)
        inv_end = [inv_end[3], inv_end[0], inv_end[2], inv_end[1], inv_end[4], inv_end[5], inv_end[6], inv_end[7]]

        # Napravi trajektoriju
        trajectory = JointTrajectory()

        # Napravi pointove
        point1 = JointTrajectoryPoint()
        point1.positions = inv_start
        point1.time_from_start = rospy.Duration(0.6)
        trajectory.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = inv_end
        point2.time_from_start = rospy.Duration(1.6)
        trajectory.points.append(point2)

        end_position[2] -= 0.22
        inv_pickup = self.inverse_kin.inverse_kinematics_kitting_arm(end_position)
        inv_pickup = [inv_pickup[3], inv_pickup[0], inv_pickup[2], inv_pickup[1], inv_pickup[4], inv_pickup[5], inv_pickup[6], inv_pickup[7]]
        point3 = JointTrajectoryPoint()
        point3.positions = inv_pickup
        point3.time_from_start = rospy.Duration(3)
        trajectory.points.append(point3)


        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()

        self.kitting_cmd.publish(trajectory)
        print("Sent trajectory")

        while not self.check_kitting_position(end_position):
            rospy.sleep(0.2)
        self.inverse_kin.deactivate_kitting_gripper()
        print("Let go")
        return

    # Gantry ima dodatnu varijablu joints, koja determinira koji dio gantry se treba pomaknuti. Za micanje samo baze koristi path_planner.py!!!
    # 0 - Cijeli gantry
    # 1 - samo ruka
    # Liftup odreduje treba li dici ruku prije nego se krene
    # 0 - Nemoj dici ruku
    # 1 - Digni ruku prije bilo cega drugog
    def pickup_gantry(self, position, joints=0, liftup=0):
        self.gantry_pickedup = False
        inv_start = self.inverse_kin.direct_kinematics_gantry_arm()
        print ("START:" + str(inv_start))
        inv_start[2] = inv_start[2] + 0.1
        inv_start.append(position[3])
        inv_start.append(position[4])
        inv_start.append(position[5])
        inv_start = self.inverse_kin.inverse_kinematics_gantry(inv_start, joints)
        start_position_arm = inv_start[3:-1]
        start_position_torso = inv_start[0:3]
        print("1:" + str(inv_start))
        
        end_position = position
        end_position[2] += 0.3
        inv_end = self.inverse_kin.inverse_kinematics_gantry(end_position, joints, inv_start)
        end_position_arm = inv_end[3:-1]
        end_position_torso = inv_end[0:3]
        print("2:" + str(end_position))

        # Napravi trajektoriju
        trajectory_arm = JointTrajectory()
        trajectory_torso = JointTrajectory()


        # Napravi pointove
        if joints == 0:
            if liftup == 1:
                point_arm = JointTrajectoryPoint()
                point_arm.positions = start_position_arm
                point_arm.time_from_start = rospy.Duration(1)
                trajectory_arm.points.append(point_arm)

                point_torso = JointTrajectoryPoint()
                point_torso.positions = start_position_torso
                point_torso.time_from_start = rospy.Duration(1)
                trajectory_torso.points.append(point_torso)

            point_arm = JointTrajectoryPoint()
            point_arm.positions = end_position_arm
            point_arm.time_from_start = rospy.Duration(1.5)
            trajectory_arm.points.append(point_arm)

            point_torso = JointTrajectoryPoint()
            point_torso.positions = end_position_torso
            point_torso.time_from_start = rospy.Duration(1.5)
            trajectory_torso.points.append(point_torso)
        elif joints == 1:
            if liftup == 1:
                point_arm = JointTrajectoryPoint()
                point_arm.positions = start_position_arm
                point_arm.time_from_start = rospy.Duration(1)
                trajectory_arm.points.append(point_arm)

            point_arm = JointTrajectoryPoint()
            point_arm.positions = end_position_arm
            point_arm.time_from_start = rospy.Duration(1.5)
            trajectory_arm.points.append(point_arm)


        trajectory_arm.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm.header.stamp = rospy.Time.now()
        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()

        # Reci robotu da dode iznad objekta
        if joints == 0:
            self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        print("sent first")

        while not self.check_gantry_position(end_position):
            rospy.sleep(0.2)
        print("calc second")


        # Posalji robota na pickup
        self.inverse_kin.activate_gantry_gripper()
        trajectory_arm2 = JointTrajectory()
        trajectory_torso2 = JointTrajectory()

        end_position[2] -= 0.1
        print("3:" + str(end_position))
        inv_pickup = self.inverse_kin.inverse_kinematics_gantry(end_position, joints)
        end_position_arm1   = inv_pickup[3:-1]
        end_position_torso1 = inv_pickup[0:3]

        point_arm2 = JointTrajectoryPoint()
        point_arm2.positions = end_position_arm1
        point_arm2.time_from_start = rospy.Duration(1)
        trajectory_arm2.points.append(point_arm2)

        if joints == 0:
            point_torso2 = JointTrajectoryPoint()
            point_torso2.positions = end_position_torso1
            point_torso2.time_from_start = rospy.Duration(1)
            trajectory_torso2.points.append(point_torso2)

        end_position[2] -= 0.2
        print("4:" + str(end_position))
        inv_pickup = self.inverse_kin.inverse_kinematics_gantry(end_position, joints)
        end_position_arm1   = inv_pickup[3:-1]
        end_position_torso1 = inv_pickup[0:3]

        point_arm2 = JointTrajectoryPoint()
        point_arm2.positions = end_position_arm1
        point_arm2.time_from_start = rospy.Duration(7)
        trajectory_arm2.points.append(point_arm2)

        if joints == 0:
            point_torso2 = JointTrajectoryPoint()
            point_torso2.positions = end_position_torso1
            point_torso2.time_from_start = rospy.Duration(7)
            trajectory_torso2.points.append(point_torso2)

        trajectory_arm2.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm2.header.stamp = rospy.Time.now()
        trajectory_torso2.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso2.header.stamp = rospy.Time.now()

        if joints == 0:
            self.gantry_torso_cmd.publish(trajectory_torso2)
        self.gantry_cmd.publish(trajectory_arm2)

        print("sent second")
        self.watching_gantry = True

        return  

    def place_gantry(self, position, joints=0):
        inv_start = self.inverse_kin.direct_kinematics_gantry_arm()
        inv_start[2] = inv_start[2] + 0.3
        inv_start.append(position[3])
        inv_start.append(position[4])
        inv_start.append(position[5])
        inv_start = self.inverse_kin.inverse_kinematics_gantry(inv_start, joints)
        del inv_start[-2]
        del inv_start[-1]
        start_position_arm = inv_start[3:-1]
        start_position_torso = inv_start[0:3]

        end_position = position
        end_position[2] += 0.3
        inv_end = self.inverse_kin.inverse_kinematics_gantry(end_position, joints, inv_start)
        del inv_end[-2]
        del inv_end[-1]
        end_position_arm = inv_end[3:-1]
        end_position_torso = inv_end[0:3]

        end_position[2] -= 0.2
        inv_pickup = self.inverse_kin.inverse_kinematics_gantry(end_position, joints, inv_end)
        del inv_pickup[-2]
        del inv_pickup[-1]
        place_position_arm = inv_pickup[3:-1]
        place_position_torso = inv_pickup[0:3]

        # Napravi trajektoriju
        trajectory_arm = JointTrajectory()
        trajectory_torso = JointTrajectory()


        # Napravi pointove
        if joints == 0:
            point_torso = JointTrajectoryPoint()
            point_torso.positions = start_position_torso
            point_torso.time_from_start = rospy.Duration(1)
            trajectory_torso.points.append(point_torso)

            point_torso = JointTrajectoryPoint()
            point_torso.positions = end_position_torso
            point_torso.time_from_start = rospy.Duration(2.5)
            trajectory_torso.points.append(point_torso)

            point_torso = JointTrajectoryPoint()
            point_torso.positions = place_position_torso
            point_torso.time_from_start = rospy.Duration(3.5)
            trajectory_torso.points.append(point_torso)

        point_arm = JointTrajectoryPoint()
        point_arm.positions = start_position_arm
        point_arm.time_from_start = rospy.Duration(1)
        trajectory_arm.points.append(point_arm)

        point_arm = JointTrajectoryPoint()
        point_arm.positions = end_position_arm
        point_arm.time_from_start = rospy.Duration(2.5)
        trajectory_arm.points.append(point_arm)

        point_arm = JointTrajectoryPoint()
        point_arm.positions = place_position_arm
        point_arm.time_from_start = rospy.Duration(3.5)
        trajectory_arm.points.append(point_arm)


        trajectory_arm.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm.header.stamp = rospy.Time.now()
        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()

        print("TORSO TRAJ")
        print(trajectory_torso)
        print()
        print()
        print("ARM TRAJ")
        print(trajectory_arm)


        # Reci robotu da dode iznad objekta
        if joints == 0:
            self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        print("sent first")


        while not self.check_gantry_position(end_position):
            print("trying to reach end position")
            rospy.sleep(0.2)
        self.inverse_kin.deactivate_gantry_gripper()
        print("Let go")
        return

    # Funkcije koje direktno micu robota na neku poziciju. Ne uzimaju objekt niti pokusavaju raditi ikakav obstacle avoidance. Ne koristiti osim za priblizno pozicioniranje
    def move_directly_kitting(self, position):
        rospy.sleep(0.5)
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
        print("sent")
        return

    def move_directly_gantry(self, position, joints = 0):
        rospy.sleep(0.5)
        inv_position = self.inverse_kin.inverse_kinematics_gantry(position, joints)
        inv_position_arm = inv_position[3:-1]
        inv_position_torso = inv_position[0:3]

        trajectory_arm = JointTrajectory()
        trajectory_torso = JointTrajectory()

        # Napravi point
        point_arm = JointTrajectoryPoint()
        point_arm.positions = inv_position_arm
        point_arm.time_from_start = rospy.Duration(1.5)
        trajectory_arm.points.append(point_arm)


        point_torso = JointTrajectoryPoint()
        point_torso.positions = inv_position_torso
        point_torso.time_from_start = rospy.Duration(1.5)
        trajectory_torso.points.append(point_torso)

        # Ispuni trajectory
        trajectory_arm.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm.header.stamp = rospy.Time.now()

        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()

        if joints == 0:
            self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        print("sent")
        return

    # Funkcija za pokupljavanje sa trake pomocu kitting robota. Pozovi i on ide na predmet odredenog indexa (default 0)
    def pickup_from_track(self, pose_array_i=0):
        rospy.sleep(0.2)
        current_pose = self.track_poses.poses[pose_array_i].position

        # 1) Dodi ispred predmeta
        current_position = [current_pose.x, current_pose.y - 1, current_pose.z + 0.5, 0, math.pi/2, 0]
        print(current_position)
        preposition = self.inverse_kin.inverse_kinematics_kitting_arm(current_position)
        preposition = [preposition[3], preposition[0], preposition[2], preposition[1], preposition[4], preposition[5], preposition[6], preposition[7]]

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = preposition
        point.time_from_start = rospy.Duration(1.5)
        trajectory.points.append(point)

        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory)

        while not self.check_kitting_position(current_position):
            rospy.sleep(0.05)
        print("got to position")


        self.inverse_kin.activate_kitting_gripper()
        self.watching_pickup = True
        while not self.picked_from_tray:
            current_pose = self.track_poses.poses[pose_array_i].position

            current_position = [current_pose.x, current_pose.y - 0.05, current_pose.z, 0, math.pi/2, 0]
            preposition = self.inverse_kin.inverse_kinematics_kitting_arm(current_position)
            preposition = [preposition[3], preposition[0], preposition[2], preposition[1], preposition[4], preposition[5], preposition[6], preposition[7]]
            point = JointTrajectoryPoint()
            trajectory = JointTrajectory()
            point.positions = preposition
            point.time_from_start = rospy.Duration(0.8)
            trajectory.points.append(point)
            trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            trajectory.header.stamp = rospy.Time.now()
            self.kitting_cmd.publish(trajectory)
            rospy.sleep(0.04)

        current_position = [current_pose.x, current_pose.y, current_pose.z + 0.5, 0, math.pi/2, 0]
        print(current_position)
        preposition = self.inverse_kin.inverse_kinematics_kitting_arm(current_position)
        preposition = [preposition[3], preposition[0], preposition[2], preposition[1], preposition[4], preposition[5], preposition[6], preposition[7]]

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = preposition
        point.time_from_start = rospy.Duration(1.5)
        trajectory.points.append(point)

        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()
        self.kitting_cmd.publish(trajectory)


        return

