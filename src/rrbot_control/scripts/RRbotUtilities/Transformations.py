#!/usr/bin/env python3
#Author: lyleokoth 

import numpy as np
from math import sin, cos

def rot_x(alpha):
    """
    Creates a 3x3 rotation matrix around the x axis

    Parameters
    ----------
    alpha : double
        The rotation angle around the x axis in radians

    returns
    -------
    rx : numpy array, double
        A 3x3 matrix 

    """
    rx = np.array([[1, 0,          0,         ],
                   [0, cos(alpha), -sin(alpha)],
                   [0, sin(alpha), cos(alpha) ]])

    return rx   

def rot_y(beta):
    """
    Creates a 3x3 rotation matrix around the y axis

    Parameters
    ----------
    beta : double
        The rotation angle around the x axis in radians

    returns
    -------
    ry : numpy array, double
        A 3x3 matrix 

    """
    ry = np.array([[cos(beta),   0, sin(beta)],
                   [0,           1, 0        ],
                   [-sin(beta),  0, cos(beta)]])

    return ry


def rot_z(gamma):
    """
    Creates a 3x3 rotation matrix around the z axis

    Parameters
    ----------
    gamma : double
        The rotation angle around the x axis in radians

    returns
    -------
    rz : numpy array, double
        A 3x3 matrix 

    """
    rz = np.array([[cos(gamma), -sin(gamma), 0],
                   [sin(gamma), cos(gamma),  0],
                   [0,          0,           1]])

    return rz


def rot_xyz(alpha, beta, gamma):
    """
    Creates a 3x3 rotation matrix around the x,y,z axis

    Parameters
    ----------
    alpha : double
        The rotation angle around the x axis in radians
    beta : double
        The rotation angle around the x axis in radians
    gamma : double
        The rotation angle around the x axis in radians

    returns
    -------
    rxyz : numpy array, double
        A 3x3 matrix 

    """
    rxyz = rot_x(alpha).dot(rot_y(beta)).dot(rot_z(gamma))
    return rxyz


def homog_transxyz(dx, dy, dz):
    """
    Create a 4x4 homogeneous translation matrix (translation along the x,y,z axes)

    parameters
    ----------
    dx : double
        Translation along the x axis
    dy : double
        Translation along the y axis
    dz : double
        Translation along the z axis

    returns
    -------
    trans : 4x4 numpy array
        Homogenous translation matrix
    """
    trans = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1 ]])
    return trans


def homog_transform(dx,dy,dz,alpha,beta,gamma):
    """
    Create a homogeneous 4x4 transformation matrix

    parameters
    ----------
    alpha : double
        The rotation angle around the x axis in radians
    beta : double
        The rotation angle around the x axis in radians
    gamma : double
        The rotation angle around the x axis in radians   
    dx : double
        Translation along the x axis
    dy : double
        Translation along the y axis
    dz : double
        Translation along the z axis

    returns
    -------
    homog_trans_xyz : 4x4 numpy array 
        Homogenous tranformation matrix
    """
    rot4x4 = np.eye(4)
    rot4x4[:3,:3] = rot_xyz(alpha,beta,gamma)
    homog_trans_xyz = np.dot(homog_transxyz(dx,dy,dz),rot4x4)
    return homog_trans_xyz