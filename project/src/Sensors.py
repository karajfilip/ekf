#!/usr/bin/env python

import numpy as np
import rospy
import math
import tf2_ros
import yaml
import re

# Break Beam Sensor, Quality Control Sensor and Logical Camera messages
#from nist_gear.msg import Proximity, LogicalCameraImage, Model
from nist_gear.msg import *

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header


class Sensors_functions:

    def __init__(self):
        self.bb1 = False
        self.bb2 = False
        self.objects = list()

        rospy.Subscriber('/ariac/breakbeam_1_change', Proximity, self.break_beam_callback_1)
        rospy.Subscriber('/ariac/breakbeam_2_change', Proximity, self.break_beam_callback_2)


    def tf_transform(self, frame):
        '''
        Get the world pose of object
        Returns:
        pose: A pose of the object
        '''
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        success = False

        # wait for all cameras to be broadcasting
        while(not success):
            try:
                success = True
                world_tf = tf_buffer.lookup_transform(
                    'world',
                    str(frame),
                    rospy.Time(),
                    rospy.Duration(0.1)
                )
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                print(frame)
                success = False
                continue

        # remove stale transforms
        tf_time = rospy.Time(
            world_tf.header.stamp.secs,
            world_tf.header.stamp.nsecs
        )
        # if rospy.Time.now() - tf_time > rospy.Duration(1.0):
        #     continue

        pose = Pose()
        pose.position = world_tf.transform.translation
        pose.orientation = world_tf.transform.rotation
        return pose

    def get_object_pose_in_workcell(self, camera_num="[0-9]"):
        '''
        Get the world pose of each object found by cameras,
        including parts and movable trays
        Note, logical cameras must be named using the convention:
        logical_camera_x
        Returns:
        list: A list of all the objects found
        '''
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # check if there is a sensor blackout
        try:
            print(rospy.wait_for_message("/ariac/breakbeam_0", Proximity, 1))
        except:
            return self.objects

        # wait for all cameras to be broadcasting
        all_topics = rospy.get_published_topics()
        #  NOTE: This will not work if your logical cameras are named differently
        camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
        for topic in camera_topics:
            rospy.wait_for_message(topic, LogicalCameraImage)

        # e.g., logical_camera_1_assembly_pump_red_1
        camera_frame_format = r"logical_camera_" + str(camera_num) + "+_(\w+)_[0-9]+_frame"
        all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
        part_frames = [f for f in all_frames if re.match(camera_frame_format, f)]

        self.objects = []
        for frame in part_frames:
            try:
                world_tf = tf_buffer.lookup_transform(
                    'world',
                    frame,
                    rospy.Time(),
                    rospy.Duration(0.1)
                )
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                continue

            # remove stale transforms
            tf_time = rospy.Time(
                world_tf.header.stamp.secs,
                world_tf.header.stamp.nsecs
            )
            if rospy.Time.now() - tf_time > rospy.Duration(1.0):
                continue

            model = Model()
            model.type = re.match(camera_frame_format, frame).group(1)
            model.pose.position = world_tf.transform.translation
            model.pose.orientation = world_tf.transform.rotation
            self.objects.append(model)
        return self.objects

    def break_beam_callback_1(self, msg):
        ''' For human obstacle at as 2 '''
        if msg.object_detected:
            self.bb1 = True
        else:
            self.bb1 = False

    def break_beam_callback_2(self, msg):
        ''' For human obstacle at as 4 '''
        if msg.object_detected:
            self.bb2 = True
        else:
            self.bb2 = False

class Sensors_subscribers:

    def __init__(self):

        rospy.wait_for_message('/ariac/logical_camera_8', LogicalCameraImage)
        # Data from sensors
        # self.data = {}
        self.breakbeam_detection = {}
        self.logical_camera_detection = {}
        self.i = 0
        self.track_items_pose = PoseArray()
        self.faulty = [False]*4

        # Velocity of track in m/s
        self.v = 0.2
        # The beggining of the track
        self.p_start = [-0.573, 4.3, 0.9]
        # The end of track
        # self.p_end = [p1_x, p1_y, p1_z];

        # SUBSCRIBERS
        # Each sensor has its own subscriber
        # Break Beam Sensor
        rospy.Subscriber('/ariac/breakbeam_0_change', Proximity, self.break_beam_callback)

        # Logical Camera
        rospy.Subscriber('/ariac/logical_camera_1', LogicalCameraImage, self.logical_camera_1_callback)
        rospy.Subscriber('/ariac/logical_camera_2', LogicalCameraImage, self.logical_camera_2_callback)
        rospy.Subscriber('/ariac/logical_camera_3', LogicalCameraImage, self.logical_camera_3_callback)
        rospy.Subscriber('/ariac/logical_camera_4', LogicalCameraImage, self.logical_camera_4_callback)

        # Quality Control Sensor
        rospy.Subscriber('/ariac/quality_control_sensor_1', LogicalCameraImage, self.quality_control_sensor_1_callback)
        rospy.Subscriber('/ariac/quality_control_sensor_2', LogicalCameraImage, self.quality_control_sensor_2_callback)
        rospy.Subscriber('/ariac/quality_control_sensor_3', LogicalCameraImage, self.quality_control_sensor_3_callback)
        rospy.Subscriber('/ariac/quality_control_sensor_4', LogicalCameraImage, self.quality_control_sensor_4_callback)

        # PUBLISHERS
        # Position on track
        self.pub_pose_on_track = rospy.Publisher('ariac/pose_on_track', PoseArray, queue_size=1)
        while not rospy.is_shutdown():
            del self.track_items_pose.poses[:]
            for key, value in self.breakbeam_detection.items():
                # item_pose = Pose()
                item_pose = self.position_on_track(value.to_nsec(), rospy.get_rostime().to_nsec())
                if item_pose.position.y < -4.4:
                    continue
                # print(key)
                self.track_items_pose.header.frame_id = str(key)
                #   print(self.track_items_pose)
                self.track_items_pose.poses.append(
                    self.position_on_track(value.to_nsec(), rospy.get_rostime().to_nsec()))
            self.pub_pose_on_track.publish(self.track_items_pose)
            #print(self.track_items_pose)
            rospy.sleep(0.01)

    ### CALLBACKS ###
    def break_beam_callback(self, msg):
        ''' Callback function for break beam sensor
        @param msg, Proximity, data from sensor'''
        if msg.object_detected:
            self.i += 1
            self.breakbeam_detection.update({self.i: msg.header.stamp})

    def logical_camera_1_callback(self, msg):
        ''' Callback function for logical camera
        @param msg, LogicalCameraImage, data logical camera'''
        self.logical_camera_detection.update({1: msg.models})

    def logical_camera_2_callback(self, msg):
        ''' Callback function for logical camera
        @param msg, LogicalCameraImage, data logical camera'''
        self.logical_camera_detection.update({2: msg.models})

    def logical_camera_3_callback(self, msg):
        ''' Callback function for logical camera
        @param msg, LogicalCameraImage, data logical camera'''
        self.logical_camera_detection.update({3: msg.models})

    def logical_camera_4_callback(self, msg):
        ''' Callback function for logical camera
        @param msg, LogicalCameraImage, data logical camera'''
        self.logical_camera_detection.update({4: msg.models})



    def quality_control_sensor_1_callback(self, msg):
        ''' Callback function for quality control sensor
        @param msg, LogicalCameraImage, quality control sensor'''
        if not msg.models:
            self.faulty[0] = False
        else:
            self.faulty[0] = True

    def quality_control_sensor_2_callback(self, msg):
        ''' Callback function for quality control sensor
        @param msg, LogicalCameraImage, quality control sensor'''
        if not msg.models:
            self.faulty[1] = False
        else:
            self.faulty[1] = True

    def quality_control_sensor_3_callback(self, msg):
        ''' Callback function for quality control sensor
        @param msg, LogicalCameraImage, quality control sensor'''
        if not msg.models:
            self.faulty[2] = False
        else:
            self.faulty[2] = True

    def quality_control_sensor_4_callback(self, msg):
        ''' Callback function for quality control sensor
        @param msg, LogicalCameraImage, quality control sensor'''
        if not msg.models:
            self.faulty[3] = False
        else:
            self.faulty[3] = True

    def position_on_track(self, t0, t):
        ''' Calculate position of object on track at time t
        @param t0, int, the time, in seconds, when the object was spawn (detected by sensor) on the track
        @param t, int, time in seconds
        @return Pose of object at time t'''
        p = Pose()

        p.position.x = self.p_start[0]
        p.position.y = self.p_start[1] - self.v * float((t - t0)) / (10 ** 9)
        p.position.z = self.p_start[2]
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0

        # self.pub_pose_on_track.publish(p)
        return p



if __name__ == '__main__':
    rospy.init_node('sensors_subscribers', anonymous=True)
    subscribers = Sensors_subscribers()
    #functions = Sensors_functions()
    #objects = functions.get_object_pose_in_workcell()
    #  faulty = functions.tf_transform("logical_camera_2_assembly_pump_red_1_frame")
    #print(functions.tf_transform("kit_tray_1"))
