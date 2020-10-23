#!/usr/bin/env python
"""Code for Lab 7
Course: EECS C106A, Fall 2020
Author: Ritika Shrivatava
This file takes two images and their intrictics matrices as input and return matched features
between the two images with epipolar constraints. 
"""
import cv2
import numpy as np

def epipolar_error(x1, x2, l1, l2):
    """
       Task 2
       ------
       Computes the error of the candidate match (x1, x2), given in *normalized* image
       homogeneous coordinates. l1 and l2 are the respective epipolar lines.
       
       x1: np.array of size (3,): (u1, v1, 1)
       x2: np.array of size (3,): (u2, v2, 1)
       l1: np.array of size (3,): (a1, b1, c1)
       l2: np.array of size (3,): (a2, b2, c2)
    """ 
    # calculate the distance between the line l1 and x1.
    d1 = TODO

    # calculate the distance between the line l2 and x2.
    d2 = TODO

    # compute the total error.
    error = TODO

    return error


def FilterByEpipolarConstraint(intrinsics1, intrinsics2, points1, points2, R, T, threshold, matches):
    """
        Task 2
        ------
        Returns an array inlier_mask of length equal to the length of matches. inlier_mask[i] is 1
        if matches[i] satisfies the epipolar constraint (i.e. the error is less than the threshold.
        Otherwise, inlier_mask[i] = 0.
        
        intrinsics1: np.array of size (3, 3): intrinsic camera matrix of left camera.
        intrinsics2: np.array of size (3, 3): intrinsic camera matrix of right camera.
        points1: np.array of size (M, 3): homogeneous, unnormalized coordinates of keypoints in left image.
        points2: np.array of size (N, 3): homogeneous, unnormalized coordinates of keypoints in right image.
        matches: list of cv2.Match objects. matches[i].queryIdx is the index in points1 of the first keypoint
                 in the i-th match. matches[i].trainIdx is the index in points2 of the second keypoint in the
                 i-th match.
    """
    # Delete this return statement when you implement this function.
    return np.ones(len(matches)).astype(np.int32)

    # Compute Essential matrix
    E = TODO

    inlier_mask = []
    
    for i in matches:	
        u_v1 = points1[i.queryIdx]
        u_v2 = points2[i.trainIdx]

        (u1,v1) = u_v1.pt
        (u2,v2) = u_v2.pt  
            
        # normalize x1 and x2
        x1 = TODO
        x2 = TODO

	# compute epilines l1, l2.
        l1 = TODO
        l2 = TODO

        error = epipolar_error(x1, x2, l1, l2)
        m = (error < threshold).astype(int)	        
        inlier_mask.append(m)
    return np.array(inlier_mask)

