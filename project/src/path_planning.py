#!/usr/bin/env python

from __future__ import print_function
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


# TOCKE NA KOJE TREBA DOC
#								(small_long_joint, torso_base_main_joint,torso_rail_joint)
# 1 - STATION DESNI PREDNJI  - (  -3.7 ,  1   , -3.4   )
# 2 - STATION LIJEVI PREDNJI - (  -3.9 , 2.44 ,  3.2  )
# 3 - STATION DESNI STRAZNJI - (  -8.7 ,  1   , -3.4  )
# 4 - STATION LIJEVI STRAZNJI- (  -8.8 , 2.44 ,  3.2  )
# 5 - STATION GRIPPER CHANGE - (-1.75, 0, 1.48)

# STA KOJI JOINT RADI?
# small_long_joint - 		mice robota naprijed-nazad (mijenja prednji i straznji station)
# torso_base_main_joint - 	rotacija robota lijevo/desno (locked na 1/2.44 rad odns. otpr. 60/140 stupnjeva trenutno)
# torso_rail_joint -		mice robota lijevo/desno (mijenja izmedu lijevog i desnog stationa)



class GantryPlanner:
	# Inicijalizacija klase
	def __init__(self):
		# TODO: Definiraj pub i sub za pozicije gantrya
		self.pos_subscriber 	  = rospy.Subscriber("/ariac/gantry/gantry_controller/state", JointTrajectoryControllerState, self.position_cb, queue_size=1)
		self.trajectory_publisher = rospy.Publisher("/ariac/gantry/gantry_controller/command", JointTrajectory, queue_size=1)

		self.wanted_pos = [0,0,0];
		self.received_pos = [0,0,0];
		self.current_station = -1;

		# TODO: Definiraj subscribere za breakbeam kamere koje detetiraju ljude
		self.blocked_1 = False
		self.blocked_2 = False
		self.blocked_3 = False
		self.blocked_4 = False
		rospy.sleep(0.1)

		return

	# Pomakne robota na odredeni station. Brojevi stationa su na pocetku filea
	# Rotacije: 1 - okrenut na lijevo (2.44 rad) 
	#			2 - okrenut na desno  (1 rad)
	def move(self, station_number, rotation = 1):
		print("PATH_PLANNER: MICEM GANTRY NA STATION " + str(station_number) + " ROTIRAN ", end="")
		if rotation == 1:
			print("LIJEVO (2.44 rad)")
		elif rotation == 2:
			print("DESNO (2.44 rad)")

		
		move_to = JointTrajectory();
		move_to.header.stamp = rospy.Time.now()
		move_to.joint_names = ["small_long_joint", "torso_base_main_joint", "torso_rail_joint"]
		points = JointTrajectoryPoint()
		used_time = rospy.Duration(0) 	# Koliko dugo traju trenutni pokreti? Koristi za time_from_start

		if rotation == 1:
			wanted_rot = 2.44
		elif rotation == 2:
			wanted_rot = 1


		# Provjeri gdje se robot trenutno nalazi. Ako je u krivom redu, pomakni ga na centar, onda promjeni red i tek onda odi na station
		# Ako je robot u dobrom redu, onda kreni prema odmah prema stationu.
		# Prije micanja OBAVEZNO se robot pomakne iza od stationa tako da ne udari station/assembly
		# Ako je current_station = -1, onda se smatra da je robot na pocetnoj poziciji (prednji red, centar)
		if self.current_station == -1:
			if station_number == 1:
				points.positions = [-3.7, wanted_rot, -3.4]
				points.time_from_start = rospy.Duration(1)
				used_time = points.time_from_start 
			elif station_number == 2:
				points.positions = [-3.9, wanted_rot, -3.2]
				points.time_from_start = rospy.Duration(1)
				used_time = points.time_from_start
			elif station_number == 5:
				points.positions = [-1.75, 0, -6.30]
				points.time_from_start = rospy.Duration(4)
				used_time = points.time_from_start
			else:
				points_change_row = JointTrajectoryPoint()
				points_change_row.positions = [-8, 0, 0]
				points_change_row.time_from_start = rospy.Duration(2.5)
				move_to.points.append(points_change_row)
				used_time = points_change_row.time_from_start
				if station_number == 3:
					points.positions = [-8.7, wanted_rot, -3.4]
					points.time_from_start = used_time + rospy.Duration(1)
					used_time = points.time_from_start

				elif station_number == 4:
					points.positions = [-8.8, wanted_rot, 3.2]
					points.time_from_start = used_time + rospy.Duration(1)
					used_time = points.time_from_start



		
		# -- Automatski dodaj jedan point koji ce robota pomaknuti iza od trenutnog stationa, isto napravi na kraju kad se priblizava stationu
		if self.current_station != -1 and station_number <= 4:
			point_move_away = JointTrajectoryPoint()
			point_move_away.positions = self.received_pos
			point_move_away.positions[0] += 0.5
			point_move_away.time_from_start = used_time + rospy.Duration(0.2)
			used_time = point_move_away.time_from_start
			move_to.points.append(point_move_away)

		if self.current_station <= 4:
			point_move_close = JointTrajectoryPoint()
			point_move_close.positions = list(points.positions)
			point_move_close.positions[0] = point_move_close.positions[0] + 0.5
			point_move_close.time_from_start = points.time_from_start
			points.time_from_start = used_time + rospy.Duration(0.5)
			move_to.points.append(point_move_close)



		# -- Logika gotova. Salji robotu pomak --
		move_to.points.append(points)
		self.wanted_pos = points.positions
		self.current_station = station_number
		self.trajectory_publisher.publish(move_to)
		print(points)
		return

	def position_cb(self, data):
		self.received_pos = [round(data.actual.positions[0], 2), round(data.actual.positions[1], 2), round(data.actual.positions[2], 2)]
		#print("PATH_PLANNER: GOT POSTION : " + str(self.received_pos))

	def check_position(self):
		if self.received_pos > self.wanted_pos - 0.02 and self.received_pos < self.wanted_pos + 0.02:
			print("PATH_PLANNER: ROBOT REACHED POSITION " + str(self.wanted_pos))
			return True
		else:
			return False

# --- TESTNI KOD -- MAKNI KASNIJE ---
#rospy.init_node("path_planner")
#gantry_planner = GantryPlanner()

#gantry_planner.move(5, 1) 

#rospy.spin()
