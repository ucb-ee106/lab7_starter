#!/usr/bin/env python

"""Code for Lab 7
Course: EECS C106A, Fall 2020
Author: Jay Monga
This file takes in a stream of data from a drone with known pose and generates a pointcloud.
"""

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
import cv2
import ros_numpy
from ros_numpy import numpy_msg
import numpy as np
import tf.transformations as transformations
import tf2_ros as tf2
from epipolar import FilterByEpipolarConstraint

def least_squares_triangulate(x_1, x_2, R, T, left_intrinsic, right_intrinsic):
    """
    Task 3
    ------
    Computes the coordinates of the point represented by the corresponding pair (x1, x2).
    x1, x2 are given in unnormalized homogeneous coordinates.
    You should compute the coordinates X of the point written in the reference frame of the
    right camera.

    (R, T) is the transform g_21.

    left_intrinsic and right_intrinsic are both numpy arrays of size (3, 3) representing the 
    3x3 intrinsic matrices of the left and right cameras respectively.
    """

    # Remove this return statement once you implement this function.
    return None

    left_intrinsic_inv = np.linalg.inv(left_intrinsic)
    right_intrinsic_inv = np.linalg.inv(right_intrinsic)
    
    # Use least squares to solve for lambda1 and lambda2.
    lambda_1, lambda_2 = TODO, TODO
    
    if lambda_1 > 0 and lambda_2 > 0:
        X1 = TODO
        X2 = TODO
        X = .5 * (X1 + X2)
        return X
    else:
        return None

class Stereo_Pointcloud:
    def __init__(self, vis=True): 
        # creating a BRISK feature dectector
        # First argument is a threshold used by the underlying Harris corner
        # detector. Raising this threshold will give you fewer but higher quality
        # keypoints.
        self.feature_detector = cv2.BRISK_create(50, octaves=5)	
        
        self.vis = vis

        # Task 0: Fill in the topic names for the following subscribers.
        left_image_sub = message_filters.Subscriber(TODO, Image, queue_size=10)
        right_image_sub = message_filters.Subscriber(TODO, Image, queue_size=10)
        left_camera_info_sub = message_filters.Subscriber(TODO, CameraInfo, queue_size=10)
        right_camera_info_sub = message_filters.Subscriber(TODO, CameraInfo, queue_size=10)
        # End Task 0.

        msg_filter = message_filters.ApproximateTimeSynchronizer([left_image_sub, right_image_sub, left_camera_info_sub, right_camera_info_sub], queue_size=10, slop=.05)
        msg_filter.registerCallback(self.camera_callback)
        self.match_pub = rospy.Publisher("/drone/matches", Image, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/drone/pointcloud", PointCloud2, queue_size=10)
        self.tfBuffer = tf2.Buffer()
        self.total_points = 0
        listener = tf2.TransformListener(self.tfBuffer)
        
        self.left_rect_pub = rospy.Publisher("/drone/left/undistorted_image", Image, queue_size=10)
        self.right_rect_pub = rospy.Publisher("/drone/right/undistorted_image", Image, queue_size=10)
         
    def extract_and_match_features(self, left_image, right_image): 
        """
        Task 1
        """
        # find the keypoints and descriptors with BRISK
        kp1, des1 = self.feature_detector.detectAndCompute(left_image,None)
        kp2, des2 = self.feature_detector.detectAndCompute(right_image,None)

        # Task 1: Create a cv2.BFMatcher object. Then, complete the call to
        # matcher.match
        # Hint: You will need to pass in an argument to specify the distance function opencv
        # should use. What argument should you use there? Should we set the crossCheck argument
        # to True or False?
        matcher = TODO
        matches = matcher.match(TODO, TODO)

        return kp1, kp2, matches

    def camera_callback(self, left_image_msg, right_image_msg, left_camera_info_msg, right_camera_info_msg):
        """
        Get synchronized camera messages
        """
        left_image = ros_numpy.numpify(left_image_msg)
        right_image = ros_numpy.numpify(right_image_msg)
        left_intrinsic = np.array(left_camera_info_msg.K).reshape((3, 3))
        right_intrinsic = np.array(right_camera_info_msg.K).reshape((3, 3))
        try:
            trans = self.tfBuffer.lookup_transform("right", "left", rospy.Time())
            dist_left = left_camera_info_msg.D
            dist_right = right_camera_info_msg.D
            left_undistorted = cv2.undistort(left_image, left_intrinsic, dist_left)
            right_undistorted = cv2.undistort(right_image, right_intrinsic, dist_right)
            
            self.left_rect_pub.publish(ros_numpy.msgify(Image, left_undistorted, "mono8"))
            self.right_rect_pub.publish(ros_numpy.msgify(Image, right_undistorted, "mono8"))
            
            # Task 1: Uncomment this call to self.process_images. 
            # self.process_images(left_undistorted, right_undistorted, left_intrinsic, right_intrinsic, trans)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            pass
    
    def process_images(self, left_image, right_image, left_intrinsic, right_intrinsic, trans):
        """
        Do fun stuff in lab
        """
        T, quat = ros_numpy.numpify(trans.transform.translation).reshape(-1, 1), ros_numpy.numpify(trans.transform.rotation)
        R = np.array(transformations.quaternion_matrix(quat))[:3, :3]
        
	kp1, kp2, matches = self.extract_and_match_features(left_image, right_image)

    	left_image_matches = np.array([kp1[i.queryIdx].pt for i in matches])
    	right_image_matches = np.array([kp2[i.trainIdx].pt for i in matches])

        threshold = .07
        inlier_mask = np.array(FilterByEpipolarConstraint(left_intrinsic, right_intrinsic, kp1, kp2, R, T, threshold, matches)) == 1
        left_image_masked = np.pad(left_image_matches[inlier_mask], [(0, 0), (0, 1)], mode='constant', constant_values=1)
        right_image_masked = np.pad(right_image_matches[inlier_mask], [(0, 0), (0, 1)], mode='constant', constant_values=1)

        if self.vis: 
            match_vis1 = cv2.drawMatches(
                left_image, kp1, 
                right_image, kp2, matches, 0,
                flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            filtered_matches = [m for m, b in zip(matches, inlier_mask) if b == 1]
            match_vis2 = cv2.drawMatches(
                left_image, kp1, 
                right_image, kp2, filtered_matches, 0,
                flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            match_vis = cv2.vconcat([match_vis1, match_vis2])
            self.match_pub.publish(ros_numpy.msgify(Image, match_vis, 'rgb8'))
       
        n = left_image_masked.shape[0]
        print "Total Matches:", n
        points_list = []
        for i in range(n):
            x_1 = left_image_masked[i, :].reshape((-1, 1))
            x_2 = right_image_masked[i, :].reshape((-1, 1))
            x_w = least_squares_triangulate(x_1, x_2, R, T, left_intrinsic, right_intrinsic)
            if x_w is not None:
            	x_1_i = x_1[:2].astype('uint32').reshape((1, 2))[:, ::-1]
            	x_2_i = x_2[:2].astype('uint32').reshape((1, 2))[:, ::-1]
            	rgb = (.5 * left_image[x_1_i[0, 0], x_1_i[0, 1]] + .5 * right_image[x_2_i[0, 0], x_2_i[0, 1]])
            	point = x_w.flatten().tolist() + [rgb]
            	points_list.append(point)
        m = len(points_list)
        print "Number of keypoints in pointcloud:", m
        if m > 0:
            points_arr_unordered = np.array(points_list)
            points_arr = np.zeros((m,), dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('r', np.uint8),
                ('g', np.uint8),
                ('b', np.uint8)])
            points_arr['x'] = points_arr_unordered[:, 0]
            points_arr['y'] = points_arr_unordered[:, 1]
            points_arr['z'] = points_arr_unordered[:, 2]
            points_arr['r'] = points_arr_unordered[:, 3]
            points_arr['g'] = points_arr_unordered[:, 3]
            points_arr['b'] = points_arr_unordered[:, 3]
            cloud_msg = ros_numpy.msgify(numpy_msg(PointCloud2), points_arr)
            cloud_msg.header.frame_id = "right"
            self.pointcloud_pub.publish(cloud_msg)
            print "Published a pointcloud."

if __name__ == "__main__":
    rospy.init_node("Stereo_Pointcloud")
    node = Stereo_Pointcloud(vis=True)
    rospy.spin()
