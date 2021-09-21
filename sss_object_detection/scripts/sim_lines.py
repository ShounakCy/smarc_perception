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
        #if len(self.marked_positions_x) > 0:
         #   return

        Line = line()
        for marker in msg.markers:
            #self.marked_positions['{}/{}'.format(marker.ns, marker.id)] = marker
            self.marked_positions_x = marker.pose.position.x
            self.marked_positions_y = marker.pose.position.y
            self.marked_ns = marker.ns
            
            if self.marked_ns == "buoy_corner1":
                x1 = int(np.unique(self.marked_positions_x))
                #print >>sys.stderr, 'Marked Position_corner1 of = "%s"'  % x1
                y1 = int(np.unique(self.marked_positions_y))
                #print >>sys.stderr, 'Marked Position_corner1 of = "%s"'  % y1
                #Line.ns = self.marked_ns
            elif self.marked_ns == "buoy_corner2":
                x2 = int(np.unique(self.marked_positions_x))
                #print >>sys.stderr, 'Marked Position_corner2 of = "%s"'  % x2
                y2 = int(np.unique(self.marked_positions_y))
                #print >>sys.stderr, 'Marked Position_corner2 of = "%s"'  % y2
                #Line.ns = self.marked_ns
            elif self.marked_ns == "buoy_corner4":
                x3 = int(np.unique(self.marked_positions_x))
                #print >>sys.stderr, 'Marked Position_corner4 of = "%s"'  % x3
                y3 = int(np.unique(self.marked_positions_y))
                #print >>sys.stderr, 'Marked Position_corner4 of = "%s"'  % y3
                #Line.ns = self.marked_ns
            elif self.marked_ns == "buoy_corner3":
                x4 = int(np.unique(self.marked_positions_x))
                #print >>sys.stderr, 'Marked Position_corner3 of = "%s"'  % x4
                y4 = int(np.unique(self.marked_positions_y))
                #print >>sys.stderr, 'Marked Position_corner3 of = "%s"'  % y4
                #Line.ns = self.marked_ns

            #_ = plt.plot(self.marked_positions_x, self.marked_positions_y, 'o', label='Buoys', markersize=10)

        X_= np.array([x1, x2, x3+0.0001, x4+0.0001])
        #print >>sys.stderr, 'X = "%s"'  % X_
        Y_= np.array([y1, y2, y3, y4])
        #print >>sys.stderr, 'Y = "%s"'  % Y_

        
        for i in range(len(X_)):
            X= np.array([X_[i-1], X_[i]])
            #print >>sys.stderr, 'X = "%s"'   % X
            Y= np.array([Y_[i-1], Y_[i]])
            #print >>sys.stderr, 'Y = "%s"'  % Y


            #X= np.array([-5,-5])
            #print >>sys.stderr, 'X = "%s"'   % X
            #Y= np.array([4,14])
            #print >>sys.stderr, 'Y = "%s"'  % Y

            A = np.vstack([X, np.ones(len(X))]).T
            #print >>sys.stderr, 'A = "%s"'  % A

            m, c = np.linalg.lstsq(A, Y, rcond=None)[0]
            Line.ns = "Rope_" +str(i)
            Line.m = m
            Line.c = c
            
            #print >>sys.stderr, 'ns  = "%s"'  % self.marked_ns
            #print >>sys.stderr, 'm  = "%s"'  % m
            #print >>sys.stderr, 'c = "%s"'  % c
            #return m, c
            #_ = plt.plot(X, Y, 'o', label='Original data', markersize=10)
            
            #_ = plt.plot(X, m*X + c, 'r', label='Fitted line')
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
