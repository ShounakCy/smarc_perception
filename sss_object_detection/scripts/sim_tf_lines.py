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
    """A mock SSS object detector for simulation. Only objects within the
    detection_range of the vehicle will be detectable."""
    def __init__(self,
                 robot_name,
                 noise_sigma=.01):
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
        self.pub_intercept_utm = rospy.Publisher('/{}/sim/intercepts_utm'.format(robot_name) ,line, queue_size=10)


        self.pub_detected_line = rospy.Publisher('/{}/sim/detected_line'.format(robot_name), Marker, queue_size=10)
        
        self.pub_marker3 = rospy.Publisher('/{}/sim/marker3'.format(robot_name), line, queue_size=10)
        



    
    def _update_pose(self, msg):
        """Update prev_pose and current_pose according to the odom msg received"""
        #rospy.sleep(1)
        #print >>sys.stderr, 'robot pose::::::::::::::::::    = "%s"'  % msg
        self.current_pose = self._transform_pose(
            msg.pose, from_frame=msg.header.frame_id)
        #print >>sys.stderr, 'robot pose transposed::::::::::::::::::    = "%s"'  % self.current_pose
        #self.intercept = self._transform_pose_2world(msg.pose, )


    def unique(list1):
            x = np.array(list1)
   
        
    def _tf_marker_pose(self,msg):
  
  
        for marker in msg.markers:
            self.marked_ns = marker.ns
            if self.marked_ns == "buoy_corner1":
                #print >>sys.stderr, 'marker pose 1= "%s"'  % marker
                marker_transformed = self._transform_pose(marker, marker.header.frame_id)
                self.marked_positions_1 = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
                #marker_transformed.pose.position.z =0.0
                #print >>sys.stderr, 'marked_positions_1  = "%s"'  % self.marked_positions_1
                x1 = marker_transformed.pose.position.x
                y1 = marker_transformed.pose.position.y
                z1 = marker_transformed.pose.position.z

            if self.marked_ns == "buoy_corner3":
                self.marked_positions_3 = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
                marker_transformed = self._transform_pose(marker, marker.header.frame_id)
                #marker_transformed.pose.position.z =0.0

                x3 = marker_transformed.pose.position.x
                y3 = marker_transformed.pose.position.y
                z3 = marker_transformed.pose.position.z

        X_= np.array([x1, x3])
        #print >>sys.stderr, 'X = "%s"'  % X_
        Y_= np.array([y1, y3])
        #print >>sys.stderr, 'Y = "%s"'  % Y_
        Z_= np.array([z1, z3])

        marker3 = line()



        marker3_utm = self._transform_pose_2_utm(marker, marker.header.frame_id, self.marked_positions_3)
        print >>sys.stderr,     'marker3_utm "%s"'  % marker3_utm

        marker3.x = marker3_utm.pose.position.x
        marker3.y = marker3_utm.pose.position.y
        marker3.z = marker3_utm.pose.position.z
        marker3.frame_id = marker3_utm.header.frame_id


        #marker3.x = self.marked_positions_3[0]+3.0
        #marker3.y = self.marked_positions_3[1]
        #marker3.z = self.marked_positions_3[2]
        #marker3.frame_id = "map"
        self.pub_marker3.publish(marker3)

        A = np.vstack([X_, np.ones(len(X_))]).T
        m, c = np.linalg.lstsq(A, Y_, rcond=None)[0]

        rospy.sleep(0.5)
        
        dist_from_line= math.sqrt(c*c)
        #rospy.sleep(0.5)
        #print >>sys.stderr,     'dist_from_line "%s"'  % dist_from_line
        if dist_from_line<6:
            if m<0 and x1>0 and x3<0 and c>=y1 and c<=y3 :
    
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)

            elif m<0 and x1<0 and x3>0 and c<=y1 and c>=y3 :
        
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,1)
                self._detect_pcl(marker_transformed, self.marked_positions_1, c)
                              
                #print >>sys.stderr, 'LINE ------------------ DETECTED'
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
             
            else:
              
                #print >>sys.stderr, 'marked_positions_1  = "%s"'  % self.marked_positions_1
                #print >>sys.stderr, 'marked_positions_3  = "%s"'  % self.marked_positions_3
                self._rviz_line(self.marked_positions_1, self.marked_positions_3,0)
                #print >>sys.stderr, 'XXXXXXXXXXXXXXXX NOT DETECTED XXXXXXXXXXXXXXXxx'
        else:
      
            #print >>sys.stderr, 'marked_positions_1  = "%s"'  % self.marked_positions_1
            #print >>sys.stderr, 'marked_positions_3  = "%s"'  % self.marked_positions_3
            self._rviz_line(self.marked_positions_1, self.marked_positions_3,0)
            #print >>sys.stderr, 'XXXXXXXXXXXXXXXX NOT DETECTED XXXXXXXXXXXXXXXxx'
    

    def _detect_pcl(self,m_t, m1,c):
        intercept_msg= self._transform_pose_2map(m_t,m_t.header.frame_id,c)
        intercept_msg_utm_line = line()
        intercept_msg_utm = self._transform_pose_2_utm_intercept(m_t,m_t.header.frame_id,c )
        intercept_msg_utm_line.x = intercept_msg_utm.pose.position.x
        intercept_msg_utm_line.y = intercept_msg_utm.pose.position.y
        intercept_msg_utm_line.z = intercept_msg_utm.pose.position.z
        intercept_msg_utm_line.frame_id = intercept_msg_utm.header.frame_id
        self.pub_intercept_utm.publish(intercept_msg_utm_line)
        print >>sys.stderr,'intercept_msg_utm "%s"'  % intercept_msg_utm
        #rospy.sleep(1)
        #self.detected_points = PointCloud()
        self.detected_points.header.frame_id = intercept_msg.header.frame_id
        self.detected_points.header.stamp =rospy.Time.now()
        m1[0]                           += np.random.randn() * self.noise_sigma
        #intercept_msg.pose.position.y   += np.random.randn() * self.noise_sigma
        #m1[2]                           += np.random.randn() * self.noise_sigma
        self.detected_points.points.append(Point32(m1[0]+1, intercept_msg.pose.position.y,m1[2]))
        #self.detected_points.points.append(Point32(0.0, 1.0, 0.0))
        self.pub_intercept.publish(self.detected_points)


    def _rviz_line(self,m1, m3, n):
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
            marker_line.color.a = 1.0
            marker_line.color.r = 1.0
            marker_line.color.g = 1.0
            marker_line.color.b = 0.0
        if n==0:
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
        #print >>sys.stderr, 'X = "%s"'  % X
        #print >>sys.stderr, 'Y = "%s"'  % Y
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

        #rospy.sleep(0.5)

  

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


    def _transform_pose_2map(self, pose, from_frame,c):
        pose.pose.position.x = 0.0
        pose.pose.position.y = c
        pose.pose.position.z = 0.0
        #print >>sys.stderr, 'pose_intercept = "%s"'  % pose
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame=self.map_frame_id)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed
    
    def _transform_pose_2_utm(self, pose, from_frame,m3):
        pose.pose.position.x = m3[0]+2
        pose.pose.position.y = m3[1]
        pose.pose.position.z = m3[2]
        #print >>sys.stderr, 'pose = "%s"'  % pose
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame="utm")
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed

    def _transform_pose_2_utm_intercept(self, pose, from_frame,c):
        pose.pose.position.x = 0.0
        pose.pose.position.y = c
        pose.pose.position.z = 0.0
        #print >>sys.stderr, 'pose_intercept = "%s"'  % pose
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame=self.map_frame_id)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed 


    def _transform_pose(self, pose, from_frame):
        trans = self._wait_for_transform(from_frame=from_frame,
                                         to_frame=self.published_frame_id)
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
