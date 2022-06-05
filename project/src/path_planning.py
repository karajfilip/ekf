from __future__ import print_function
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
import math
from copy import deepcopy


# TOCKE NA KOJE TREBA DOC
#                               (small_long_joint, torso_base_main_joint,torso_rail_joint)
# 1 - STATION DESNI PREDNJI  - (  -3.7 ,  1   , -3.4   )
# 2 - STATION LIJEVI PREDNJI - (  -3.9 ,  2.44 ,  3.2  )
# 3 - STATION DESNI STRAZNJI - (  -8.7 ,  1   , -3.4   )
# 4 - STATION LIJEVI STRAZNJI- (  -8.8 ,  2.44 ,  3.2  )
# 5 - STATION GRIPPER CHANGE - (  -1.75,  0   ,  -6.30 )
# 6 - STATION TRAYS          - (  -3.95,  0   ,  -4.9  )
# 7 - STATION KITTING AGV1   - (  -0.71, -2.35,  3.4   )
# 8 - STATION KITTING AGV2   - (  -0.71, -0.8,   2.38  )
# 9 - STATION KITTING AGV3   - (  -0.71, -2.35,  -2.38 )
# 10 - STATION KITTING AGV4   -(  -0.71, -0.8,   -3.4  )
# 11 - HOME                  - (  -1.46,  0   ,   0    )

# STA KOJI JOINT RADI?
# small_long_joint -        mice robota naprijed-nazad (mijenja prednji i straznji station)
# torso_base_main_joint -   rotacija robota lijevo/desno (locked na 1/2.44 rad odns. otpr. 60/140 stupnjeva trenutno)
# torso_rail_joint -        mice robota lijevo/desno (mijenja izmedu lijevog i desnog stationa)



class GantryPlanner:
    # Inicijalizacija klase
    def __init__(self):
        # TODO: Definiraj pub i sub za pozicije gantrya
        self.pos_subscriber       = rospy.Subscriber("/ariac/gantry/gantry_controller/state", JointTrajectoryControllerState, self.position_cb, queue_size=1)
        self.trajectory_publisher = rospy.Publisher("/ariac/gantry/gantry_controller/command", JointTrajectory, queue_size=1)
        
        self.wanted_pos = [0,0,0];
        self.received_pos = [0,0,0];
        self.current_station = 11;

        self.stations = [ [],
                         [  -3.7 , math.pi/2, -3],
                         [  -3.7 , math.pi/2,  3],
                         [  -8.7 , math.pi/2, -3],
                         [  -8.7 , math.pi/2,  3],
                         [  -1.75,  0   , -6.30  ],
                         [  -4.15,  0   , -5.0   ],
                         [    0.35  ,math.pi, 3.8],
                         [    0.35  ,math.pi, 0.4],
                         [    0  ,  0   , -0.40  ],
                         [    0  ,  0   , -4.0   ],
                         [  -1.46,  0   ,   0    ]]

        # TODO: Definiraj subscribere za breakbeam kamere koje detetiraju ljude
        self.blocked_1 = False
        self.blocked_2 = False
        self.blocked_3 = False
        self.blocked_4 = False
        self.checking_position = False
        rospy.sleep(0.1)

        return

    # Pomakne robota na odredeni station. Brojevi stationa su na pocetku filea
    def move(self, station):
        if (station == 'as1'):
            station_number = 1
        elif (station == 'as2'):
            station_number = 3
        elif (station == 'as3'):
            station_number = 2
        elif (station == 'as4'):
            station_number = 4
        elif (station == 'gripperstation'):
            station_number = 5
        elif (station == 'traystation'):
            station_number = 6
        elif (station == 'agv4'):
            station_number = 7
        elif (station == 'agv3'):
            station_number = 8
        elif (station == 'agv2'):
            station_number = 9
        elif (station == 'agv1'):
            station_number = 10
        elif (station == 'home'):
            station_number = 11

        print("PATH_PLANNER: MICEM GANTRY NA STATION " + str(station_number) + " SA STATIONA " + str(self.current_station))

        if self.current_station == station_number:
            print("PATH_PLANNER: GANTRY VEC NA STATIONU " + str(self.current_station))

        move_to = JointTrajectory();
        move_to.header.stamp = rospy.Time.now()
        move_to.joint_names = ["small_long_joint", "torso_base_main_joint", "torso_rail_joint"]
        points = JointTrajectoryPoint()
        used_time = rospy.Duration(0)   # Koliko dugo traju trenutni pokreti? Koristi za time_from_start

        
        # -- Automatski dodaj jedan point koji ce robota pomaknuti iza od trenutnog stationa, isto napravi na kraju kad se priblizava stationu
        if self.current_station != 11:
            if self.current_station not in [5,6,7,8,9,10]:
                point_move_away = JointTrajectoryPoint()
                point_move_away.positions = deepcopy(self.stations[self.current_station])
                point_move_away.positions[0] += 0.5
                point_move_away.time_from_start = used_time + rospy.Duration(0.8)
                used_time = point_move_away.time_from_start
                move_to.points.append(point_move_away)
            elif self.current_station in [5,6]:
                point_move_away = JointTrajectoryPoint()
                point_move_away.positions = deepcopy(self.stations[self.current_station])
                point_move_away.positions[2] += 1.5
                point_move_away.time_from_start = used_time + rospy.Duration(0.8)
                used_time = point_move_away.time_from_start
                move_to.points.append(point_move_away)
            else:
                point_move_away = JointTrajectoryPoint()
                point_move_away.positions = deepcopy(self.stations[self.current_station])
                point_move_away.positions[0] -= 1.5
                point_move_away.time_from_start = used_time + rospy.Duration(1.0)
                used_time = point_move_away.time_from_start
                move_to.points.append(point_move_away)



        # Provjeri gdje se robot trenutno nalazi. Ako je u krivom redu, pomakni ga na centar, onda promjeni red i tek onda odi na station
        # Ako je robot u dobrom redu, onda kreni prema odmah prema stationu.
        # Prije micanja OBAVEZNO se robot pomakne iza od stationa tako da ne udari station/assembly
        # Ako je current_station = -1, onda se smatra da je robot na pocetnoj poziciji (prednji red, centar)
        if self.current_station in [1,2,5,6,7,8,9,10,11]:
            if station_number in [1,2,5,6,7,8,9,10,11]:
                points.positions = self.stations[station_number]
                points.time_from_start = used_time + rospy.Duration(4)
                used_time = points.time_from_start 
            else:
                # Makni na centar
                if station_number != 11:
                    points_change_row = JointTrajectoryPoint()
                    points_change_row.positions = [-3, self.stations[self.current_station][1], 0]
                    points_change_row.time_from_start = used_time + rospy.Duration(1.5)
                    move_to.points.append(points_change_row)
                    used_time = points_change_row.time_from_start

                points_change_row = JointTrajectoryPoint()
                points_change_row.positions = [-8, self.stations[self.current_station][1], 0]
                points_change_row.time_from_start = used_time + rospy.Duration(2.5)
                move_to.points.append(points_change_row)
                used_time = points_change_row.time_from_start

                points.positions = self.stations[station_number]
                points.time_from_start = used_time + rospy.Duration(2)
                used_time = points.time_from_start

        else:
            if station_number in [3,4]:
                points.positions = self.stations[station_number]
                points.time_from_start = used_time +  rospy.Duration(3)
                used_time = points.time_from_start 
            else:
                points_change_row = JointTrajectoryPoint()
                points_change_row.positions = [-8, self.stations[self.current_station][1], 0]
                points_change_row.time_from_start = used_time + rospy.Duration(1.5)
                move_to.points.append(points_change_row)
                used_time = points_change_row.time_from_start

                points_change_row = JointTrajectoryPoint()
                points_change_row.positions = [-3, self.stations[self.current_station][1], 0]
                points_change_row.time_from_start = used_time +  rospy.Duration(2.5)
                move_to.points.append(points_change_row)
                used_time = points_change_row.time_from_start

                points.positions = self.stations[station_number]
                points.time_from_start = used_time + rospy.Duration(2)
                used_time = points.time_from_start

        if station_number in [1,2,3,4,7,8,9,10]:
            if station_number in[1,2,3,4]:
                point_move_close = JointTrajectoryPoint()
                point_move_close.positions = list(points.positions)
                point_move_close.positions[0] = point_move_close.positions[0] + 0.5
                point_move_close.time_from_start = points.time_from_start
                points.time_from_start = used_time + rospy.Duration(0.5)
                move_to.points.append(point_move_close)
            else:
                point_move_close = JointTrajectoryPoint()
                point_move_close.positions = list(points.positions)
                point_move_close.positions[0] = point_move_close.positions[0] - 0.5
                if station_number in [10,9]:
                    point_move_close.positions[2] = point_move_close.positions[2] + 0.8
                else:
                    point_move_close.positions[2] = point_move_close.positions[2] - 0.6
                point_move_close.time_from_start = points.time_from_start
                points.time_from_start = used_time + rospy.Duration(2.5)
                move_to.points.append(point_move_close)


        move_to.points.append(points)



        # -- Logika gotova. Salji robotu pomak --
        self.wanted_pos = points.positions
        self.current_station = station_number
        self.trajectory_publisher.publish(move_to)
        self.checking_position = True
        return

    def position_cb(self, data, tolerance = 0.1):
        if self.checking_position:
            x = self.stations[self.current_station][0]
            y = self.stations[self.current_station][1]
            z = self.stations[self.current_station][2]

            robx = data.actual.positions[0]
            roby = data.actual.positions[2]
            robz = data.actual.positions[1]

            if abs(x - robx) < tolerance:
                if abs(y - roby) < tolerance:
                    if abs(z - robz) < tolerance:
                        print(x,y,z)
                        print(data.actual)
                        self.checking_position = False
                        print("PATH_PLANNER: ARRIVED")
        return

    def check_position(self):
        if self.received_pos > self.wanted_pos - 0.02 and self.received_pos < self.wanted_pos + 0.02:
            print("PATH_PLANNER: ROBOT REACHED POSITION " + str(self.wanted_pos))
            return True
        else:
            return False



