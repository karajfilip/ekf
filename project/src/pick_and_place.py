import rospy
import math
from trajectory_msgs.msg import JointTrajectory
#from nist_gear.msg import VacuumGripperState
#from nist_gear.msg import VacuumGripperControl
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from Actuators import Actuators
from nist_gear.msg import VacuumGripperState

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
        self.gantry_torso_cmd = rospy.Publisher("/ariac/gantry/gantry_controller/command", JointTrajectory, queue_size=3)
        self.gantry_cmd = rospy.Publisher("/ariac/gantry/gantry_arm_controller/command", JointTrajectory, queue_size=3)
        self.kitting_cmd = rospy.Publisher("/ariac/kitting/kitting_arm_controller/command", JointTrajectory, queue_size=3)

        rospy.sleep(0.5)
        self.inverse_kin = Actuators()
        #self.gantry_grip = rospy.ServiceProxy("ariac/kitting/arm/gripper/control", VacuumGripperControl)   ### enbale:true ili enable:false
        #self.kitting_grip = rospy.ServiceProxy("/ariac/gantry/arm/gripper/control", VacuumGripperControl)

        #self.kitting_grip_state = rospy.Subscriber("ariac/kitting/arm/gripper/state", VacuumGripperState, queue_size=1)
        #self.gantry_grip_state = rospy.Subscriber("/ariac/gantry/arm/gripper/state", VacuumGripperState,queue_size=1)


    def is_object_attached_kitting(self):
        return rospy.wait_for_message('/ariac/kitting/arm/gripper/state', VacuumGripperState)

    # Pickup i place funkcije za gantry i kitting.
    # Pickup pomice robota pazljivo na poziciju, te zatim prima predmet
    # Place radi isto, samo spusta objekt na neku lokaciju. Baca warning ako robot nema nista gripanog.
    # Position = [x,y,z, roll, pitch, yaw]
    def pickup_kitting(self, position):
        rospy.sleep(0.2)

        # Razvrsti putanju na tri tocke:
        # 1) Okreni elbow_joint za 0.3 rad, tako ce se ruka dici od trenutne pozicije
        # 2) Pomakni robota 0.5 iznad objekta
        # 3) Aktiviraj gripper, te pomici robota ispod dok ga ne uhvatis

        inv_start = list(self.inverse_kin.kitting_joint_state.position)
        inv_start[2] = inv_start[2] - 0.3

        end_position = position
        end_position[2] += 0.3

        inv_end = self.inverse_kin.inverse_kinematics_kitting_arm(end_position)
        inv_end = [inv_end[3], inv_end[0], inv_end[2], inv_end[1], inv_end[4], inv_end[5], inv_end[6], inv_end[7]]

        # Napravi trajektoriju
        trajectory = JointTrajectory()

        # Napravi point
        point = JointTrajectoryPoint()
        point.positions = inv_start
        point.time_from_start = rospy.Duration(1)
        trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = inv_end
        point.time_from_start = rospy.Duration(4)
        trajectory.points.append(point)

        trajectory.joint_names = ["elbow_joint", "linear_arm_actuator_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        trajectory.header.stamp = rospy.Time.now()

        self.kitting_cmd.publish(trajectory)
        print("sent")

        return  

    def place_kitting(self, postition):
        return

    def pickup_gantry(self, position):
        return

    def place_gantry(self, position):
        return

    # Funkcije koje direktno micu robota na neku poziciju. Ne uzimaju objekt niti pokusavaju raditi ikakav obstacle avoidance. Ne koristiti osim za priblizno pozicioniranje
    def move_directly_kitting(self, position):
        inv_position = self.inverse_kin.inverse_kinematics_kitting_arm(position)
        inv_position = [inv_position[3], inv_position[0], inv_position[2], inv_position[1], inv_position[4], inv_position[5], inv_position[6], inv_position[7]]
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

    def move_directly_gantry(self, position):
        inv_position = self.inverse_kin.inverse_kinematics_gantry(position)
        inv_position_arm = inv_position[3:-1]
        inv_position_torso = inv_position[0:3]

        print inv_position_arm

        trajectory_arm = JointTrajectory()
        trajectory_torso = JointTrajectory()

        # Napravi point
        point_arm = JointTrajectoryPoint()
        point_arm.positions = inv_position_arm
        point_arm.time_from_start = rospy.Duration(5)
        trajectory_arm.points.append(point_arm)


        point_torso = JointTrajectoryPoint()
        point_torso.positions = inv_position_torso
        point_torso.time_from_start = rospy.Duration(5)
        trajectory_torso.points.append(point_torso)

        # Ispuni trajectory
        trajectory_arm.joint_names = ["gantry_arm_shoulder_pan_joint", "gantry_arm_shoulder_lift_joint","gantry_arm_elbow_joint","gantry_arm_wrist_1_joint","gantry_arm_wrist_2_joint","gantry_arm_wrist_3_joint"]
        trajectory_arm.header.stamp = rospy.Time.now()

        trajectory_torso.joint_names = ["small_long_joint", "torso_rail_joint", "torso_base_main_joint"]
        trajectory_torso.header.stamp = rospy.Time.now()

        self.gantry_torso_cmd.publish(trajectory_torso)
        self.gantry_cmd.publish(trajectory_arm)
        print("sent")
        return


#rospy.init_node("robot_mover")
#robot_move = RobotMover()
#robot_move.pickup_kitting([-1.79899, 2.665, 0.7796, 0, math.pi/2, 0])
#robot_move.move_gantry([ -5, 0.77, 1.5, 0, math.pi/2, 0 ])
#rospy.spin()