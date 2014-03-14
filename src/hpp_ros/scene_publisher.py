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
        self.pubObjects = dict ()
        self.pubRobots ['robot'] = rospy.Publisher ('/joint_states', JointState)
        rospy.init_node ('hpp')
        self.broadcaster = TransformBroadcaster ()
        self.js = JointState ()
        self.js.name = jointNames
        self.odom_trans = TransformStamped ()
        self.odom_trans.header.frame_id = "odom";
        self.odom_trans.child_frame_id = "base_link"
        self.obstacleMsg = dict ()

    def addObject (self, name):
        self.pubObjects [name] = rospy.Publisher ('/' + name, Marker)
        self.obstacleMsg [name] = Marker ()
        self.obstacleMsg [name].header.frame_id = "odom"
        self.obstacleMsg [name].header.stamp = rospy.Time.now ()
        self.obstacleMsg [name].type = Marker.SPHERE
        self.obstacleMsg [name].action = Marker.ADD
        self.obstacleMsg [name].lifetime = rospy.Duration ()
        self.obstacleMsg [name].scale.x = 1.
        self.obstacleMsg [name].scale.y = 1.
        self.obstacleMsg [name].scale.z = 1.
        self.obstacleMsg [name].color.r = 0
        self.obstacleMsg [name].color.g = 1
        self.obstacleMsg [name].color.b = 0
        self.obstacleMsg [name].color.a = 1

    def moveObject (self, name, cfg):
        R = np.identity (4)
        R [0,0:3] = cfg.rot [:3]
        R [1,0:3] = cfg.rot [3:6]
        R [2,0:3] = cfg.rot [6:9]
        quat = transformations.quaternion_from_matrix (R)
        self.obstacleMsg [name].pose.position.x = cfg.trs [0]
        self.obstacleMsg [name].pose.position.y = cfg.trs [1]
        self.obstacleMsg [name].pose.position.z = cfg.trs [2]
        self.obstacleMsg [name].pose.orientation.x = quat [0]
        self.obstacleMsg [name].pose.orientation.y = quat [1]
        self.obstacleMsg [name].pose.orientation.z = quat [2]
        self.obstacleMsg [name].pose.orientation.w = quat [3]

    def publishObjects (self):
        for name, p in self.pubObjects.iteritems ():
            if not rospy.is_shutdown ():
                p.publish (self.obstacleMsg [name])

    def publish (self):
        self.publishRobots ()
        self.publishObjects ()

    def publishRobots (self):
        if not rospy.is_shutdown ():
            now = rospy.Time.now ()
            self.js.header.stamp.secs = now.secs
            self.js.header.stamp.nsecs = now.nsecs
            self.js.header.seq += 1
            self.odom_trans.header.stamp.secs = now.secs
            self.odom_trans.header.stamp.nsecs = now.nsecs
            self.odom_trans.header.seq = self.js.header.seq

            self.odom_trans.transform.translation.x = self.robotConfig [0]
            self.odom_trans.transform.translation.y = self.robotConfig [1]
            self.odom_trans.transform.translation.z = self.robotConfig [2]
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

    def __call__ (self, q):
        self.robotConfig = q
        self.publish ()
