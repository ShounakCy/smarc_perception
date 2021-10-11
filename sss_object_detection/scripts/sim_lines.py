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
import matplotlib.pyplot as plt
from sss_object_detection.msg import line


class sim_sss_detector:
    """A mock fake line formation between buoys for simulation."""
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

        self.tf_listener = tf.TransformListener()
        
        self.marked_pos_sub = rospy.Subscriber(
            '/{}/sim/marked_positions'.format(robot_name), MarkerArray,
            self._update_marked_positions)

        self.pub = rospy.Publisher("Line_Eq", line, queue_size=2)

    def _update_marked_positions(self, msg):
        """Update marked_positions based on the MarkerArray msg received."""

        Line = line()
        for marker in msg.markers:
            self.marked_positions_x = marker.pose.position.x
            self.marked_positions_y = marker.pose.position.y
            self.marked_ns = marker.ns
            
            if self.marked_ns == "buoy_corner1":
                x1 = int(np.unique(self.marked_positions_x))
                y1 = int(np.unique(self.marked_positions_y))
                
            elif self.marked_ns == "buoy_corner2":
                x2 = int(np.unique(self.marked_positions_x))
                y2 = int(np.unique(self.marked_positions_y))
                
            elif self.marked_ns == "buoy_corner4":
                x3 = int(np.unique(self.marked_positions_x))
                y3 = int(np.unique(self.marked_positions_y))
                
            elif self.marked_ns == "buoy_corner3":
                x4 = int(np.unique(self.marked_positions_x))
                y4 = int(np.unique(self.marked_positions_y))
               
        X_= np.array([x1, x2, x3+0.0001, x4+0.0001])
        Y_= np.array([y1, y2, y3, y4])

        
        for i in range(len(X_)):
            X= np.array([X_[i-1], X_[i]])
            Y= np.array([Y_[i-1], Y_[i]])

            A = np.vstack([X, np.ones(len(X))]).T

            """Least square method is used to get the slope and intercepts"""
            m, c = np.linalg.lstsq(A, Y, rcond=None)[0]
            Line.ns = "Rope_" +str(i)
            Line.m = m
            Line.c = c
           
            self.pub.publish(Line)
            rospy.sleep(1)

        #_ = plt.legend()
        #plt.show()
        
    def unique(list1):
        x = np.array(list1)

def main():
    rospy.init_node('sim_lines', anonymous=True)
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
