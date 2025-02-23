#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import Odometry
from vision_msgs.msg import ObjectHypothesisWithPose, Detection2DArray, Detection2D
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion
import copy
import math
import sys
from sss_object_detection.msg import line
from shapely.geometry import Point
from shapely.geometry import LineString



class sim_sss_detector:
    """A mock SSS object detector for simulation. Only objects within the
    detection_range of the vehicle will be detectable."""
    def __init__(self,
                 robot_name,
                 detection_range=8,
                 buoy_radius=0.20,
                 noise_sigma=.1):
        self.robot_name = robot_name
        self.detection_range = detection_range
        self.buoy_radius = buoy_radius
        self.noise_sigma = noise_sigma
        self.prev_pose = None
        self.current_pose = None
        self.yaw = None
        self.frame_id = None
        self.stamp = rospy.Time.now()
        self.marked_positions_x = []
        self.marked_positions_y =[]
        self.marked_ns = {}
        self.marked_positions = {}
        #self.m = {}
        #self.c ={}
        #self.ns = {}

        self.tf_listener = tf.TransformListener()
        self.odom_sub = rospy.Subscriber('/{}/dr/odom'.format(robot_name), Odometry,
                                         self._update_pose)
        #self.marked_pos_sub = rospy.Subscriber(
        #    '/{}/sim/marked_positions'.format(robot_name), MarkerArray,
        #    self._update_marked_positions)
        self.marked_points_sub = rospy.Subscriber(
            '/{}/sim/marked_positions'.format(robot_name), MarkerArray,
            self.detect_projected_points)
        #self._lineEq_sub = rospy.Subscriber(
        #    '/Line_Eq'.format(robot_name), line,
        #    self._detect_lines)
        #self.pub = rospy.Publisher(
        #    '/{}/sim/sidescan/detection_hypothesis'.format(robot_name),
        #    Detection2DArray,
        #    queue_size=2)
        #self.pub_detected_markers = rospy.Publisher(
        #    '/{}/sim/sidescan/detected_markers'.format(robot_name),
        #    Marker,
        #    queue_size=2)
    
    
    def _update_pose(self, msg):
        """Update prev_pose and current_pose according to the odom msg received"""
        if not self.prev_pose:
            self.prev_pose = msg.pose.pose
            self.current_pose = msg.pose.pose

        self.stamp = msg.header.stamp
        self.frame_id = msg.header.frame_id
        self.prev_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
        #markers_in_range = self.get_markers_in_detection_range()
        #heading = self.calculate_heading()
        print >>sys.stderr, 'robot pose::::::::::::::::: = "%s"'  % msg

        rospy.sleep(1)

        #for marker in markers_in_range:
            #cos_sim = self.calculate_marker_cosine_angle(heading, marker)
            #detectable = cos_sim <= self.buoy_radius

            #if detectable:
                #print('\t{} is within detection angle! Cos = {}'.format(marker, cos_sim))
                #print >>sys.stderr, 'Detected'
                #self._publish_marker_detection(self.marked_positions[marker],
                 #                              cos_sim)
        

    def detect_projected_points(self, msg):
        #print >>sys.stderr, 'marker pose = "%s"'  % msg

        for marker in msg.markers:
            #print >>sys.stderr, 'marker pose = "%s"'  % marker
            marker_transformed = self._wait_for_marker_transform(marker)
            print >>sys.stderr, ':::::::::::::::::::::::::::marker_transformed:::::::::::::::: = "%s"'  % marker_transformed

            self.marked_positions_x = marker_transformed.pose.position.x
            self.marked_positions_y = marker_transformed.pose.position.y
            self.marked_ns = marker.ns
            if self.marked_ns == "buoy_corner1":
                x1 = int(np.unique(self.marked_positions_x))
                y1 = int(np.unique(self.marked_positions_y))
            if self.marked_ns == "buoy_corner3":
                x3 = int(np.unique(self.marked_positions_x))
                y3 = int(np.unique(self.marked_positions_y))

        position_robot = self.current_pose.position 
        #print >>sys.stderr, ' position_robot = "%s"'  % position_robot           
        point = Point(self.current_pose.position.x, self.current_pose.position.y)
        #print >>sys.stderr, 'Point = "%s"'  % point
        
        line_s = LineString([(x1, y1), (x3, y3)])
        #print >>sys.stderr, 'line_s = "%s"'  % line_s


        x = np.array(point.coords[0])

        u = np.array(line_s.coords[0])
        v = np.array(line_s.coords[len(line_s.coords)-1])

        n = v - u
        n /= np.linalg.norm(n, 2)

        P = u + n*np.dot(x - u, n)
        #print >>sys.stderr, 'P = "%s"'  % P
        
        #if P[1]>=4.0 and P[1]<=14.0:
        marker.pose.position.x = P[0]
        marker.pose.position.y = P[1]
        #print >>sys.stderr, 'P_marker = "%s"'  % marker
        rospy.sleep(1)

        marker_transformed = self._wait_for_marker_transform(marker)
        #print >>sys.stderr, 'marker_transformed = "%s"'  % marker_transformed
        rospy.sleep(1)

        distance = self._calculate_distance_to_position(marker_transformed.pose.position)
        #print >>sys.stderr, 'distance = "%s"'  % distance

        heading = self.calculate_heading()
        cos_sim = self.calculate_marker_cosine_angle(heading, marker_transformed)
        #print >>sys.stderr, 'cos_sim = "%s"'  % cos_sim
        
        #if distance <= 20.0 and cos_sim >= 0.85:
        #        print >>sys.stderr, ':::::::::::: FAKE LINE Detected :::::::::::::'
        #    #rospy.sleep(1)
        #else:
        #    print >>sys.stderr, 'xxxxxxxxxxxxxxxxx FAKE LINE NOT Detected xxxxxxxxxxxxxx'


        

    def _wait_for_marker_transform(self, marker):
        marker_pose_stamped = self._construct_pose_stamped_from_marker_msg(
            marker)
        self.tf_listener.waitForTransform(marker_pose_stamped.header.frame_id,
                                          self.frame_id, rospy.Time(),
                                          rospy.Duration(1.0))
        marker_transformed = self.tf_listener.transformPose(
            self.frame_id, marker_pose_stamped)
        return marker_transformed    

    def _construct_pose_stamped_from_marker_msg(self, marker):
        marker_pose_stamped = PoseStamped()
        marker_pose_stamped.pose = marker.pose
        marker_pose_stamped.header.stamp = self.stamp
        marker_pose_stamped.header.frame_id = marker.header.frame_id
        return marker_pose_stamped


    def _calculate_distance_to_position(self, position):
        """Calculate the distance between current_pose.position and the given position"""
        dx, dy, dz = self._get_position_differences(position,
                                                    self.current_pose.position)
        return (dx**2 + dy**2 + dz**2)**.5

    def _get_position_differences(self, position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        dz = position1.z - position2.z
        return dx, dy, dz


    def calculate_heading(self):
        """Calculate a normalized heading vector using current orientation"""
        quaternion = self.current_pose.orientation
        (_, pitch, yaw) = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        heading = np.array([
            math.cos(yaw) * math.cos(pitch),
            math.sin(yaw) * math.cos(pitch),
            math.sin(pitch)
        ]).reshape(-1, 1)
        heading = self._normalize_vector(heading)
        return heading
    
    def _normalize_vector(self, position_array):
        """Given an np.ndarray, return the normalized equivalent"""
        norm = np.linalg.norm(position_array)
        if norm > 0:
            position_array = position_array / norm
        return position_array


    def calculate_marker_cosine_angle(self, heading, marker_transformed):
        """Calculate the cosine between the heading and the marker position.
        Used to determine whether the marker is observable:
        A marker is observable if the magnitude of the projection of the vector
        from self.current_pose.position onto the heading vector <= the marker's radius."""
        

        vec_to_marker_position = self._get_vec_to_position(
            marker_transformed.pose.position, normalized=True)
        cos_heading_marker = np.dot(heading.reshape(1, -1),
                                    vec_to_marker_position.reshape(-1,
                                                                   1))[0][0]
        return abs(cos_heading_marker)

    def _get_vec_to_position(self, position, normalized=True):
        """Return vector from current_pose.position to the given position"""
        dx, dy, dz = self._get_position_differences(position,
                                                    self.current_pose.position)
        vec_to_position = np.array([dx, dy, dz]).reshape(-1, 1)

        if normalized:
            vec_to_position = self._normalize_vector(
                position_array=vec_to_position)
        return vec_to_position




    

def main():
    rospy.init_node('sim_detect_lines', anonymous=True)
    rospy.Rate(5)  # ROS Rate at 5Hz

    robot_name_param = '~robot_name'
    if rospy.has_param(robot_name_param):
        robot_name = rospy.get_param(robot_name_param)
        print('Getting robot_name = {} from param server'.format(robot_name))
    else:
        robot_name = 'sam'
        print('{} param not found in param server.\n'.format(robot_name_param))
        print('Setting robot_name = {} default value.'.format(robot_name))

    detector = sim_sss_detector(robot_name=robot_name)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()
