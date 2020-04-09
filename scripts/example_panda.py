#!/usr/bin/env python

#from __future__ import print_function

import os
import IPython
import pb_robot
import pb_robot.utils_noBase as utils

if __name__ == '__main__':
    # Launch pybullet
    utils.connect(use_gui=True)
    utils.disable_real_time()
    utils.set_default_camera()

    # Create robot object 
    robot = pb_robot.panda.Panda() 
 
    # Add floor object 
    floor_file = 'models/short_floor.urdf'
    floor = pb_robot.body.createBody(floor_file)

    # Example function on body object
    print floor.get_transform()

    # Example functions over robot arm
    q = robot.arm.GetJointValues()
    pose = robot.arm.ComputeFK(q)
    pose[2, 3] -= 0.1
    pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(pose), length=0.5, width=10)
    newq = robot.arm.ComputeIK(pose)
    if newq is not None:
        raw_input("Move to desired pose?")
        robot.arm.SetJointValues(newq)

    IPython.embed()
    
    # Close out Pybullet
    utils.wait_for_user()
    utils.disconnect()
