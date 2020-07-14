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
NAME = 'tsdf_server'

import roslib; roslib.load_manifest(PKG)
import rospy
from tsdf_fusion_ros.srv import FusionFromDir,FusionFromDirResponse

import os
import shutil
import sys
import time
import datetime

class TsdfServer:

    def __init__(self):
        rospy.init_node(NAME)  
        
        rospy.loginfo('Starting TSDF Server ...')

        # Reading parameters
        params_ok = self._read_params()
        
        if params_ok:
            # Preparing working dir
            self._prepare_wdir()

            # Launching ROS publishers and subscribers
            self._fusion_from_dir_srv = rospy.Service(rospy.get_name() + '/fusion_from_dir',FusionFromDir, self._fusion_from_dir)

            rospy.spin()
    
    def _read_params(self):
        # Voxel size
        self._voxel_size = rospy.get_param('~voxel_size', 0.02)
        rospy.loginfo('Voxel size: %.2f', self._voxel_size)

        # Working dir
        self._curr_wdir = rospy.get_param('~cwd', os.getcwd() + '/tsdf_fusion')
        rospy.loginfo('Working dir: %s', self._curr_wdir)
        if (not os.path.exists(self._curr_wdir)):
            rospy.logerr('Working dir doesn\'t exist')
            return False
        
        return True
    
    def _prepare_wdir(self):
        # Remove contents of the directory
        shutil.rmtree(self._curr_wdir)

        # Creating required dirs
        os.makedirs(self._curr_wdir + '/results')
    
    def _fusion_from_dir(self, req):
        rospy.loginfo('%s', req.directory)
        return FusionFromDirResponse(True)


if __name__ == "__main__":
    TsdfServer()