#!/usr/bin/env python3
#Author: lyleokoth

import numpy as np
from math import sqrt, atan2, sin, cos, pi
from RRbotUtilities.Transformations import homog_transform, rot_y

class ForwardKinematics(object):
    def __init__(self, link_lengths):
        """
        Provides the lenths for use in forward kinematics

        parameters
        ----------
        link_lenghts : 1x3 numpy array
            The rrbot dimensions
        """
        #Link lengths
        self.link1_length = link_lengths[0]
        self.link2_length = link_lengths[1]

        #Joint offset
        self.joint_offset = link_lengths[2]

    def rotation_base_to_l1(self, angle):
        """
        Rotation matrix from base frame to joint one frame

        parameters
        ----------
        angle : double
            The angle to move joint one to
        returns 
        ------
        theta_one_angle : double
            The angle to write
        """
        r_basetol1 = np.dot(np.eye(3), rot_y(angle))
        return r_basetol1

    def rotation_l1_to_l2(self, joint_two):
        """
        Rotation matrix from joint one frame to joint two frame

        parameters 
        ----------
        angle : double
            The angle to move joint two to
        returns 
        ------
        theta_one_angle : double
            The angle to write
        """
        r_l1_to_l2 = np.dot(np.eye(3), rot_y(joint_two))
        return r_l1_to_l2

    def rotation_l2_to_endeffector(self):
        """
        Rotation matrix from joint one frame to joint two frame

        parameters 
        ----------
        angle : double
            The angle to move joint two to
        returns 
        ------
        theta_one_angle : double
            The angle to write
        """
        r_l2_to_effector = np.eye(3)
        return r_l2_to_effector

    def rotation_base_to_endeffector(self, joint_one, joint_two):
        """
        Rotation matrix from joint one frame to joint two frame

        parameters 
        ----------
        angle : double
            The angle to move joint two to
        returns 
        ------
        theta_one_angle : double
            The angle to write
        """
        r_basetol1 = self.rotation_base_to_l1(joint_one)
        r_l1_to_l2 = self.rotation_l1_to_l2(joint_two)
        r_l2_to_effector = self.rotation_l2_to_endeffector()
        r_base_to_effector = r_basetol1.dot(r_l1_to_l2).dot(r_l2_to_effector)
        return r_base_to_effector
    