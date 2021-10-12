#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from vision_msgs.msg import ObjectHypothesisWithPose, Detection2DArray, Detection2D
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Point, Point32
from tf.transformations import euler_from_quaternion
import copy
import math
import sys
from sss_object_detection.msg import line
import tf_conversions
from sensor_msgs.msg import PointCloud

class sim_sss_detector:
    
    def __init__(self,
                 robot_name,
                 noise_sigma=.001):
        self.noise_sigma = noise_sigma
        self.robot_name = robot_name
        self.prev_pose = None
        self.current_pose = None
        self.robot_msg  = None
        self.yaw = None
        #self.frame_id = None
        self.stamp = rospy.Time.now()
        self.marked_positions_1 = None
        self.marked_positions_3 =None
        self.marked_ns = {}
        self.gt_frame_id = 'gt/{}/base_link'.format(self.robot_name)
        self.published_frame_id = '{}/base_link'.format(self.robot_name) 
        self.marked_positions = {}
        self.map_frame_id = "map"
        self.detected_points = PointCloud()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.odom_sub = rospy.Subscriber(
            '/{}/sim/odom'.format(self.robot_name), Odometry,
            self._update_pose)
        self.marked_points_sub = rospy.Subscriber(
            '/{}/sim/marked_positions'.format(robot_name), MarkerArray,
            self._tf_marker_pose)
        self.pub_intercept = rospy.Publisher('/{}/sim/intercepts'.format(robot_name) ,PointCloud, queue_size=10)
        self.pub_intercept_utm = rospy.Publisher('/{}/sim/intercepts_utm'.format(robot_name) ,PointStamped, queue_size=10)
        self.pub_detected_line = rospy.Publisher('/{}/sim/detected_line'.format(robot_name), Marker, queue_size=10)
        self.pub_marker3 = rospy.Publisher('/{}/sim/marker3'.format(robot_name), PointStamped, queue_size=10)
        self.pub_marker1 = rospy.Publisher('/{}/sim/marker1'.format(robot_name), PointStamped, queue_size=10)
    
    def _update_pose(self, msg):
        """Update prev_pose and current_pose according to the odom msg received"""

        self.current_pose = self._transform_pose(
            msg.pose, from_frame=msg.header.frame_id)

    def _tf_marker_pose(self,msg):

        for marker in msg.markers:
            self.marked_ns = marker.ns
            if self.marked_ns == "buoy_corner1":

                ### Transform buoy pose from map to robot frame
                marker_transformed = self._transform_pose(marker, marker.header.frame_id)
                self.marked_positions_1 = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
                x1 = marker_transformed.pose.position.x
                y1 = marker_transformed.pose.position.y
                z1 = marker_transformed.pose.position.z

            if self.marked_ns == "buoy_corner3":

                ### Transform buoy pose from map to robot frame
                marker_transformed = self._transform_pose(marker, marker.header.frame_id)
                self.marked_positions_3 = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
                x3 = marker_transformed.pose.position.x
                y3 = marker_transformed.pose.position.y
                z3 = marker_transformed.pose.position.z

        X_= np.array([x1, x3])

        Y_= np.array([y1, y3])

        Z_= np.array([z1, z3])

        marker3 = PointStamped()
        marker1 = PointStamped()

        #Transform buoy from map frame to utm frame
        marker3_utm = self._transform_pose_2_utm(marker, marker.header.frame_id, self.marked_positions_3)

        #shifted by 2 units in x in utm frame while transformation
        marker3.point.x = marker3_utm.pose.position.x  
        marker3.point.y = marker3_utm.pose.position.y
        marker3.point.z = marker3_utm.pose.position.z
        marker3.header.frame_id = marker3_utm.header.frame_id
        marker3.header.stamp = rospy.Time.now()
        self.pub_marker3.publish(marker3)

        #transform buoy from map frame to utm frame 
        marker1_utm = self._transform_pose_2_utm(marker, marker.header.frame_id, self.marked_positions_1)

        #shifted by 2 units in x in utm frame while transformation
        marker1.point.x = marker1_utm.pose.position.x  
        marker1.point.y = marker1_utm.pose.position.y
        marker1.point.z = marker1_utm.pose.position.z
        marker1.header.frame_id = marker1_utm.header.frame_id
        marker1.header.stamp = rospy.Time.now()
        self.pub_marker1.publish(marker1)

        """Calculating the slope and intercept of the line joining the two corner buoys"""
        A = np.vstack([X_, np.ones(len(X_))]).T
        m, c = np.linalg.lstsq(A, Y_, rcond=None)[0]

        rospy.sleep(0.5)
        
        dist_from_line= math.sqrt(c*c)
        
        if dist_from_line<6:

            #LINE DETECTED'
            if m<0 and x1>0 and x3<0 and c>=y1 and c<=y3 :
    
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)

            elif m<0 and x1<0 and x3>0 and c<=y1 and c>=y3 :

                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)

            elif m>0 and  x1<0 and x3>0 and c>=y1 and c<=y3 :
        
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)

                self.pub_intercept.publish(self.detected_points)
            elif m>0 and x1>0 and x3<0 and c<=y1 and c>=y3 :
            
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)

                self.pub_intercept.publish(self.detected_points)
            elif m==0 and (c==y1 or c==y3):
            
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)

        #NOT DETECTED  
            else:
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,0)
                
        else:
            self._rviz_line(self.marked_positions_1, self.marked_positions_3,0)
 
    def _detect_pcl(self,m_t, m1,c):

        #Transforming  the intercepts from Robot frame to Map frame
        intercept_msg= self._transform_pose_2_map(m_t,m_t.header.frame_id,c)
        self.detected_points.header.frame_id = intercept_msg.header.frame_id
        self.detected_points.header.stamp =rospy.Time.now()
        #adding noise to the x-coordinates
        m1[0] += np.random.randn() * self.noise_sigma

        #Publishing the intercepts as a PointCloud
        #shifting by 2 units in x, so as to be displayed in rviz
        self.detected_points.points.append(Point32(m1[0]+3, intercept_msg.pose.position.y,m1[2]))
        self.pub_intercept.publish(self.detected_points)

        #Transforming the intercepts from map frame to utm frame
        intercept_msg_utm = self._transform_pose_2_utm_intercept(m_t,self.detected_points.header.frame_id,intercept_msg.pose.position.y, m1 )
        
        intercept_msg_utm_line = PointStamped()
        #shifted by 2 units in x in utm frame while transformation
        intercept_msg_utm_line.point.x = intercept_msg_utm.pose.position.x
        intercept_msg_utm_line.point.y = intercept_msg_utm.pose.position.y
        intercept_msg_utm_line.point.z = intercept_msg_utm.pose.position.z
        intercept_msg_utm_line.header.frame_id = intercept_msg_utm.header.frame_id
        #Publishing as PointStamped
        self.pub_intercept_utm.publish(intercept_msg_utm_line)
        #print >>sys.stderr,'intercept_msg_utm "%s"'  % intercept_msg_utm


    def _rviz_line(self,m1, m3, n):
        # m1 and m2 are marker points published in RVIZ in the map frame

        marker_line = Marker()
        marker_line.header.frame_id = "map"
        marker_line.header.stamp =rospy.Time.now()
        marker_line.type = marker_line.LINE_STRIP
        marker_line.action = marker_line.ADD

        # marker scale
        marker_line.scale.x = 0.03
        marker_line.scale.y = 0.03
        marker_line.scale.z = 0.03

        # marker color
        if n==1:
            #Line detected
            marker_line.color.a = 1.0
            marker_line.color.r = 1.0
            marker_line.color.g = 1.0
            marker_line.color.b = 0.0
        if n==0:
            #Line Not detected
            marker_line.color.a = 1.0
            marker_line.color.r = 1.0
            marker_line.color.g = 0.0
            marker_line.color.b = 0.0

        # marker orientaiton
        marker_line.pose.orientation.x = 0.0
        marker_line.pose.orientation.y = 0.0
        marker_line.pose.orientation.z = 0.0
        marker_line.pose.orientation.w = 1.0

        # marker position
        marker_line.pose.position.x = 0.0
        marker_line.pose.position.y = 0.0
        marker_line.pose.position.z = 0.0
       
        # marker line points
        marker_line.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = m1[0]
        first_line_point.y = m1[1]
        first_line_point.z = m1[2]
        marker_line.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = m3[0]
        second_line_point.y = m3[1]
        second_line_point.z = m3[2]
        marker_line.points.append(second_line_point)

        # Publish the Marker
        self.pub_detected_line.publish(marker_line)

    def _wait_for_transform(self, from_frame, to_frame):
        """Wait for transform from from_frame to to_frame"""
        trans = None
        while trans is None:
            try:
                trans = self.tf_buffer.lookup_transform(
                    to_frame, from_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                print('Failed to transform. Error: {}'.format(error))
        return trans  

    def _transform_pose(self, pose, from_frame):
        """Transform the subscribed marker positions from map frame to robot frame, 
        in order to calculate the intercepts in robot frame"""
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame=self.published_frame_id)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed

    def _transform_pose_2_map(self, pose, from_frame,c):
        """Transform intercept from robot frame to map frame.
         Since it's coming from the robot frame, the x-position is cset to 0"""
        pose.pose.position.x = 0.0
        pose.pose.position.y = c
        pose.pose.position.z = 0.0
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame=self.map_frame_id)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed

    def _transform_pose_2_utm(self, pose, from_frame,m):
        """Transform marker from map frame to utm frame and shifting the x by 2 units, so that 
        when robot tries to goto this waypoint it doesn't hits the buoy marker"""
        pose.pose.position.x = m[0]+3
        pose.pose.position.y = m[1]
        pose.pose.position.z = m[2]
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame="utm")
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed

    def _transform_pose_2_utm_intercept(self, pose, from_frame,msg_y, m1):
        """Transform intercepts from map frame to utm frame and shifting the x by 2 units, so that 
        when robot tries to goto this waypoint it doesn't hits the rope"""
        pose.pose.position.x = m1[0]+3
        pose.pose.position.y = msg_y
        pose.pose.position.z = m1[2]
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame="utm")
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed 


def main():
    rospy.init_node('sim_tf_lines', anonymous=True)
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
