#!/usr/bin/python
"""
BSD 2-Clause License

Copyright (c) 2020, Emilio Garcia-Fidalgo
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

PKG = 'tsdf_fusion_ros' # this package name
NAME = 'publish_pose_from_file'

import roslib; roslib.load_manifest(PKG)
import rospy
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

import os
import shutil
import sys
import time
import datetime


if __name__ == "__main__":
    rospy.init_node(NAME)

    rospy.loginfo('Reading poses from file ...')

    # Reading params
    # Working dir
    pose_filename = rospy.get_param('~pose_filename', os.getcwd() + '/csv/_worldpose_global_pose.csv')

    pose_filename = '/home/emilio/Escritorio/Bagfiles/airlab/csv/_worldpose_global_pose.csv'

    rospy.loginfo('Pose filename: %s', pose_filename)
    if (not os.path.exists(pose_filename)):
        rospy.logerr('Filename doesn\'t exist')
        sys.exit(0)
    
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

    lines = []
    with open(pose_filename) as f:
        lines = f.readlines()
    
    lines = lines[1:]

    r = rospy.Rate(10.0)
    for line in lines:       

        parts = line.split(',')    

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"

        odom.pose.pose.position.x = float(parts[5])
        odom.pose.pose.position.y = float(parts[6])
        odom.pose.pose.position.z = float(parts[7])

        # Filling rotation
        qw = float(parts[11])
        qx = float(parts[8])
        qy = float(parts[9])
        qz = float(parts[10])

        quat = Quaternion(qw, qx, qy, qz)
        rmat = quat.rotation_matrix
        quat2 = Quaternion(matrix=rmat)

        # Filling rotation
        odom.pose.pose.orientation.w = quat2.elements[0]
        odom.pose.pose.orientation.x = quat2.elements[1]
        odom.pose.pose.orientation.y = quat2.elements[2]
        odom.pose.pose.orientation.z = quat2.elements[3]

        # publish the message
        odom_pub.publish(odom)
    
        r.sleep()