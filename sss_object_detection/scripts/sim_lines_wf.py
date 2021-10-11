#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Point, Point32
import sys
from sss_object_detection.msg import line
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud
import random
#import open3d as o3d
#import pyransac3d as pyrsc
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skimage.measure import LineModelND, ransac
from sklearn.linear_model import LinearRegression, RANSACRegressor


class sim_sss_detector:
    
    def __init__(self,
                 robot_name,):
        self.robot_name = robot_name
        self.prev_pose = None
        self.current_pose = None
        self.robot_msg  = None
        self.yaw = None
        self.frame_id = None
        self.stamp = rospy.Time.now()
        self.marked_positions_x = []
        self.marked_positions_y =[]
        self.marked_ns = {}
        self.gt_frame_id = 'gt/{}/base_link'.format(self.robot_name)
        self.published_frame_id = '{}/base_link'.format(self.robot_name)
        self.world_frame_id = "world_ned"

        self.inliers = []
        self.A = []
        self.B = []
        self.pcl = []
        #self.points = []
        self.marker3_x ={}
        self.marker3_y= {}

        self.X = []
        self.Y = []
    
        self.marked_positions = {}
        self.point_list=[]
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.odom_sub = rospy.Subscriber(
            '/{}/sim/odom'.format(self.robot_name), Odometry,
            self._update_pose)
        self.marked_3_sub = rospy.Subscriber(
            '/{}/sim/marker3'.format(robot_name), PointStamped,
            self.marker3_pose)
        self.intercept_sub = rospy.Subscriber('/{}/sim/intercepts'.format(robot_name), PointCloud, self._point_cloud)
        self.pub_intercept_map = rospy.Publisher('/{}/sim/intercept_points'.format(robot_name), PointStamped, queue_size=10)
        self.pub_fitted_line = rospy.Publisher('/{}/sim/fitted_line'.format(robot_name), Marker, queue_size=10)

        #self.intercept_utm = rospy.Subscriber('/{}/sim/intercepts_utm'.format(robot_name), PointStamped, self._point_cloud)

    
    def marker3_pose(self, msg):
        """Update prev_pose and current_pose according to the odom msg received"""
        #print >>sys.stderr, 'robot pose::::::::::::::::::    = "%s"'  % msg

        self.marker3_x = msg.point.x
        self.marker3_y = msg.point.y

        #return self.marker3_x, self.marker3_y


    def _update_pose(self, msg):
        #print >>sys.stderr, 'robot pose::::::::::::::::::    = "%s"'  % msg

        self.current_pose = msg.pose


    def _point_cloud(self,pts):
        
        #print >>sys.stderr, 'Point_Cloud= "%s"'  % pts
        for i in range(len(pts.points)):
            points_x = pts.points[i].x
            points_y = pts.points[i].y
            points_z = pts.points[i].z

            point_list = [points_x, points_y, points_z]
            #print >>sys.stderr, 'point_list = "%s"'  % point_list
            self.X.append(point_list[0])
            self.Y.append(point_list[1])
            self.pcl.append(point_list)
        points_np = np.array(self.pcl)

        intercept_point = PointStamped()
        intercept_point.header.frame_id = "map"
        intercept_point.header.stamp = rospy.Time(0)
        intercept_point.point.x = np.float64(points_np[len(points_np)-1][0])
        intercept_point.point.y = np.float64(points_np[len(points_np)-1][1])
        intercept_point.point.z = np.float64(points_np[len(points_np)-1][2])
        self.pub_intercept_map.publish(intercept_point)

        print >>sys.stderr, 'points_np[len(points_np)-1][1] =  "%s"'  % points_np[len(points_np)-1][1]
        
        """Using RANSAC to fit a line ,
        fitting the after the robot has passed y = 8 and has 25 samples to fit a line with"""

        if points_np[len(points_np)-1][1]>8.0 and len(points_np) > 25:
            """Chaning the corrdinates as in the moving point cloud self.Y increases as the robot moves forward"""
            X= np.array(self.Y)
            y= np.array(self.X)
            
            ransac = RANSACRegressor(LinearRegression(), 
                         max_trials=100, 
                         min_samples=20, 
                         residual_threshold=0.01)
            ransac.fit(X.reshape(-1,1), y)
            inlier_mask = ransac.inlier_mask_
            outlier_mask = np.logical_not(inlier_mask)
            
            line_X = np.arange(3, 14, 1)
            line_y_ransac = ransac.predict(line_X[:, np.newaxis])            
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.scatter(X[inlier_mask], y[inlier_mask], c='blue', marker='o', label='Inliers')
            ax.scatter(X[outlier_mask], y[outlier_mask], c='lightgreen', marker='s', label='Outliers')            
            ax.plot(line_X, line_y_ransac, color='red') 
            print >>sys.stderr,'Slope: "%.3f"' % ransac.estimator_.coef_[0]
            print >>sys.stderr,'Intercept: "%.3f"' % ransac.estimator_.intercept_
            plt.show()
            
        """Fit a line using 3D points"""
        """
        print >>sys.stderr, 'points = "%s"'  % points_np
        model_robust, inliers = ransac(points_np, LineModelND, min_samples=2,
                            residual_threshold=1, max_trials=100)
        #print(inliers)
        outliers = inliers == False
        line_x = np.arange(-5.1, -4.85, 0.01)
        #line_y = model.predict_y(line_x)
        line_y_robust = model_robust.predict_y(line_x)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points_np[inliers][:, 0], points_np[inliers][:, 1], points_np[inliers][:, 2], c='b',
                marker='o', label='Inlier data')
        ax.scatter(points_np[outliers][:, 0], points_np[outliers][:, 1], points_np[outliers][:, 2], c='r',
                marker='o', label='Outlier data')
        ax.set_ylim([3,14])
        ax.plot(line_x, line_y_robust, '-b', label='Robust line model')
        ax.legend(loc='lower left')
        print >>sys.stderr,'Slope: "%.3f"' % model_robust.estimator_.coef_[0]
        print >>sys.stderr,'Intercept: "%.3f"' % model_robust.estimator_.intercept_
        plt.show()
        """

        """Same as above, fit a line with 3d points"""
        """
        print >>sys.stderr, 'points = "%s"'  % points_np
        print >>sys.stderr, 'points = "%s"'  % type(points)

        print >>sys.stderr, 'self.pcl = "%s"'  % self.pcl

        self.A, self.B, self.inliers = self._Ransacc(points_np)

        print >>sys.stderr, 'self.A= "%s"'  % self.A
        print >>sys.stderr, 'self.B= "%s"'  % self.B
        print >>sys.stderr, 'self.inliers= "%s"'  % self.inliers
        """
    """
    def _Ransacc(self, pts, thresh=0.2, maxIteration=1000):
        n_points = pts.shape[0]
        best_inliers = []

        for it in range(maxIteration):

            # Samples 2 random points
            id_samples = random.sample(range(0, n_points), 2)
            pt_samples = pts[id_samples]

            # The line defined by two points is defined as P2 - P1
            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecA_norm = vecA / np.linalg.norm(vecA)

            # Distance from a point to a line
            pt_id_inliers = []  # list of inliers ids
            vecC_stakado = np.stack([vecA_norm] * n_points, 0)
            dist_pt = np.cross(vecC_stakado, (pt_samples[0, :] - pts))
            dist_pt = np.linalg.norm(dist_pt, axis=1)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]

            if len(pt_id_inliers) > len(best_inliers):
                best_inliers = pt_id_inliers
                self.inliers = best_inliers
                self.A = vecA_norm
                self.B = pt_samples[0, :]
             
            return self.A, self.B, self.inliers
    """


    def _rviz_line(self,X, Y):
        marker_line = Marker()
        marker_line.header.frame_id = self.published_frame_id
        marker_line.header.stamp =rospy.Time.now()
        marker_line.type = marker_line.LINE_STRIP
        marker_line.action = marker_line.ADD

        # marker scale
        marker_line.scale.x = 0.03
        marker_line.scale.y = 0.03
        marker_line.scale.z = 0.0

        # marker color

        marker_line.color.a = 1.0
        marker_line.color.r = 1.0
        marker_line.color.g = 1.0
        marker_line.color.b = 1.0


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
        first_line_point.x = X[0]
        first_line_point.y = Y[0]
        first_line_point.z = 0.0
        marker_line.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = X[1]
        second_line_point.y = Y[1]
        second_line_point.z = 0.0
        marker_line.points.append(second_line_point)

        # Publish the Marker
        self.pub_fitted_line.publish(marker_line)

def main():
    rospy.init_node('sim_lines_wf', anonymous=True)
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
