#!/usr/bin/env python

# Copyright (c) 2013 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-ros.
# hpp-ros is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-ros is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-ros.  If not, see
# <http://www.gnu.org/licenses/>.

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf import TransformBroadcaster, transformations

class ScenePublisher (object):
    def __init__ (self, jointNames):
        self.pubRobots = dict ()
        self.pubRobots ['robot'] = rospy.Publisher ('/joint_states', JointState)
        rospy.init_node ('hpp')
        self.broadcaster = TransformBroadcaster ()
        self.js = JointState ()
        self.js.name = jointNames
        # Create constant transformation between the odom frame and the robot's base link frame.
        self.odom_trans = TransformStamped ()
        self.odom_trans.header.frame_id = "odom";
        self.odom_trans.child_frame_id = "base_link"
        # Create constant transformation between the map frame and the obstacle frame.
        # Here, the obstacle can move in the map frame (see __call__, with the move q_obs) but is without any joint.
        self.trans_map_obstacle = TransformStamped ()
        self.trans_map_obstacle.header.frame_id = "map";
        self.trans_map_obstacle.child_frame_id = "obstacle_base"
        

    def publishObjects (self):
        if not rospy.is_shutdown ():
            now = rospy.Time.now ()
            self.broadcaster.sendTransform \
                (self.obstacleConfig [0: 3], self.obstacleConfig [3: 7], now, "obstacle_base", "map")

    def publish (self):
        self.publishObjects ()
        self.publishRobots ()


    def publishRobots (self):
        if not rospy.is_shutdown ():
            now = rospy.Time.now ()
            self.js.header.stamp.secs = now.secs
            self.js.header.stamp.nsecs = now.nsecs
            self.js.header.seq += 1
            self.odom_trans.header.stamp.secs = now.secs
            self.odom_trans.header.stamp.nsecs = now.nsecs
            self.odom_trans.header.seq = self.js.header.seq

            self.odom_trans.transform.rotation = (self.robotConfig [4],
                                                  self.robotConfig [5],
                                                  self.robotConfig [6],
                                                  self.robotConfig [3])
            self.js.position = self.robotConfig[7:]
            self.js.velocity = 40*[0.,]
            self.js.effort = 40*[0.,]

            rospy.loginfo (self.odom_trans)
            rospy.loginfo (self.js)
            self.broadcaster.sendTransform \
                (self.robotConfig [0: 3], self.odom_trans.transform.rotation,
                 now, "base_link", "odom")
            self.pubRobots ['robot'].publish (self.js)


    def __call__ (self, *args):
        try:
            self.robotConfig = args[0]
            # Lines to get self.obstacleConfig in the correct order : (q_obs[4], q_obs[5], q_obs[6], q_obs[3])
            self.obstacleConfig = args[1]
            self.q_tmp = (args [1]) [3]
            self.obstacleConfig.pop (3)
            self.obstacleConfig.append (self.q_tmp)
            self.publish ()
        except:
            self.robotConfig = args[0]
            self.publish ()
