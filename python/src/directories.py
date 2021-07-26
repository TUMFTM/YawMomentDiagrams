import os

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""

def create(folder):
    """
    Documentation
    Creates new subdirectories for the generated plots

    Input:
    folder:         path to the directory specified by user

    Output:

    """

    if not os.path.isdir(folder):
        os.makedirs(folder)

    # create parameter directory
    name = '/params'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)

    # create control directory
    name = '/control_moment'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)

    # create stability directory
    name = '/stability'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)

    # create steering sensitivity directory
    name = '/control_force'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)

    # create slip angle diff directory
    name = '/slip_angle_diff'
    path = folder + name
    if not os.path.exists(path):
        os.mkdir(path)
