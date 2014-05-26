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

from math import sin, cos
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf import TransformBroadcaster, transformations

class Obstacle (object):
    def __init__ (self, name, frameId):
        self.name = name
        self.frameId = frameId
        self.position = (0,0,0,1,0,0,0)

def computeRobotPositionAnchor (self):
    self.odom_trans.transform.rotation = (0.0, 0.0, 0, 1)
    self.odom_trans.transform.translation = (0.0, 0.0, 0.0)
    self.js.position = self.robotConfig

def computeRobotPositionFreeflyer (self):
    self.odom_trans.transform.rotation = (self.robotConfig [4],
                                          self.robotConfig [5],
                                          self.robotConfig [6],
                                          self.robotConfig [3])
    self.odom_trans.transform.translation = (self.robotConfig [0],
                                             self.robotConfig [1],
                                             self.robotConfig [2])
    self.js.position = self.robotConfig[7:]

def computeRobotPositionPlanar (self):
    theta = .5*self.robotConfig [2]
    self.odom_trans.transform.rotation = (0 , 0, sin (theta), cos (theta))
    self.odom_trans.transform.translation = \
        (self.robotConfig [0], self.robotConfig [1], 0)
    self.js.position = self.robotConfig[3:]

class ScenePublisher (object):
    def __init__ (self, robot):
        self.tf_root = robot.tf_root
        self.rootJointType = robot.rootJointType
        if self.rootJointType == "freeflyer":
            jointNames = robot.jointNames [4:]
            self.computeRobotPosition = computeRobotPositionFreeflyer
        elif self.rootJointType == "planar":
            jointNames = robot.jointNames [3:]
            self.computeRobotPosition = computeRobotPositionPlanar
        elif self.rootJointType == "anchor":
            jointNames = robot.jointNames
            self.computeRobotPosition = computeRobotPositionAnchor
        else:
            raise RuntimeError ("Unknow root joint type: " + self.rootJointType)
        self.pubRobots = dict ()
        self.pubRobots ['robot'] = rospy.Publisher ('/joint_states', JointState)
        rospy.init_node ('hpp')
        self.broadcaster = TransformBroadcaster ()
        self.js = JointState ()
        self.js.name = jointNames
        # Create constant transformation between the odom frame and the robot
        # base link frame.
        self.odom_trans = TransformStamped ()
        self.odom_trans.header.frame_id = "odom";
        self.odom_trans.child_frame_id = "base_link"
        # Create constant transformation between the map frame and the obstacle
        # frame.
        # Here, the obstacle can move in the map frame (see __call__, with the
        # move q_obs) but is without any joint.
        self.trans_map_obstacle = TransformStamped ()
        self.trans_map_obstacle.header.frame_id = "map";
        self.trans_map_obstacle.child_frame_id = "obstacle_base"
        self.objects = dict ()

    def addObject (self, name, frameId):
        """
        Add an object with given name and attached to given frame
        """
        self.objects [name] = Obstacle (name, frameId)

    def publishObjects (self):
        if not rospy.is_shutdown ():
            now = rospy.Time.now ()
            for n, obj in self.objects.iteritems ():
                self.broadcaster.sendTransform \
                    (obj.position [0:3], (obj.position [4],
                                          obj.position [5],
                                          obj.position [6],
                                          obj.position [3]), now,
                     obj.frameId, "map")

    def moveObject (self, name, position):
        self.objects [name].position = position

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
            self.computeRobotPosition (self)
            self.js.velocity = len (self.js.position)*[0.,]
            self.js.effort = len (self.js.position)*[0.,]

            rospy.loginfo (self.odom_trans)
            rospy.loginfo (self.js)
            self.broadcaster.sendTransform \
                (self.odom_trans.transform.translation,
                 self.odom_trans.transform.rotation,
                 now, self.tf_root, "odom")
            self.pubRobots ['robot'].publish (self.js)


    def __call__ (self, args):
        self.robotConfig = args
        self.publish ()
