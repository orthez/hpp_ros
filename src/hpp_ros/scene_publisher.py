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
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf import TransformBroadcaster, transformations

class Obstacle (object):
    def __init__ (self, name, frameId):
        self.name = name
        self.frameId = frameId
        self.position = (0,0,0,1,0,0,0)

def computeRobotPositionAnchor (self):
    self.transform.transform.rotation = (0.0, 0.0, 0, 1)
    self.transform.transform.translation = (0.0, 0.0, 0.0)
    self.js.position = self.robotConfig

def computeRobotPositionFreeflyer (self):
    self.transform.transform.rotation = (self.robotConfig [4],
                                          self.robotConfig [5],
                                          self.robotConfig [6],
                                          self.robotConfig [3])
    self.transform.transform.translation = (self.robotConfig [0],
                                             self.robotConfig [1],
                                             self.robotConfig [2])
    self.js.position = self.robotConfig[7:]

def computeRobotPositionPlanar (self):
    theta = .5*self.robotConfig [2]
    self.transform.transform.rotation = (0 , 0, sin (theta), cos (theta))
    self.transform.transform.translation = \
        (self.robotConfig [0], self.robotConfig [1], 0)
    self.js.position = self.robotConfig[3:]

## Display of robot and obstacle configurations in Rviz
#
#  This class implements
#  \li a tranform broadcaster that broadcasts the position of the robot root
#      link with respect to the global frame "map",
#  \li a joint state publisher that publishes the joint positions. These joint
#      positions are read by a node of type "robot_state_publisher" that
#      computes the relative positions of all links of the robot. Rviz then
#      displays the positions of the links.
class ScenePublisher (object):
    def __init__ (self, robot):
        self.tf_root = robot.tf_root
        self.referenceFrame = "map"
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
        self.pubRobots ['marker'] = \
            rospy.Publisher ('/visualization_marker_array', MarkerArray)
        rospy.init_node ('hpp', log_level=rospy.FATAL )
        self.broadcaster = TransformBroadcaster ()
        self.js = JointState ()
        self.js.name = jointNames
        # Create constant transformation between the map frame and the robot
        # base link frame.
        self.transform = TransformStamped ()
        self.transform.header.frame_id = self.referenceFrame;
        self.transform.child_frame_id = self.tf_root
        # Create constant transformation between the map frame and the obstacle
        # frame.
        # Here, the obstacle can move in the map frame (see __call__, with the
        # move q_obs) but is without any joint.
        self.trans_map_obstacle = TransformStamped ()
        self.trans_map_obstacle.header.frame_id = "map";
        self.trans_map_obstacle.child_frame_id = "obstacle_base"
        self.objects = dict ()
        self.markerArray = MarkerArray()
        self.oid = 0

    def addObject (self, name, frameId):
        """
        Add an object with given name and attached to given frame
        """
        self.objects [name] = Obstacle (name, frameId)

    def addPolygonFilled(self, points):
        oid = self.oid+1
        name = "/polygonFilled"+str(self.oid)
        marker = Marker()
        marker.id = self.oid
        marker.ns = "/polygonFilled"
        marker.header.frame_id = name
        marker.type = marker.TRIANGLE_LIST
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.points = []
        for i in range(0,len(points)-2,1):
                pt = Point(points[0][0], points[0][1], points[0][2])
                marker.points.append(pt)
                pt = Point(points[i+1][0], points[i+1][1], points[i+1][2])
                marker.points.append(pt)
                pt = Point(points[i+2][0], points[i+2][1], points[i+2][2])
                marker.points.append(pt)
        self.markerArray.markers.append(marker)

    def addPolygon(self, points, linewidth=0.02):
        self.oid = self.oid+1
        self.name = "/polygon"+str(self.oid)
        self.marker = Marker()
        self.marker.id = self.oid
        self.marker.ns = "/polygon"
        self.marker.header.frame_id = self.name
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.scale.x = linewidth
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.points = []
        for p in points:
                pt = Point()
                pt.x = p[0]; pt.y = p[1]; pt.z = p[2]
                self.marker.points.append(pt)
        #connect last marker to first marker
        pt = Point()
        pt.x = points[0][0]; pt.y = points[0][1]; pt.z = points[0][2]
        self.marker.points.append(pt)

        self.markerArray.markers.append(self.marker)

    def addSphere(self, x, y, z):
        self.addSphere(x, y, z, 0.05, 0.05, 0.05)

    def addSphere(self, x, y, z, sx, sy, sz):
        self.oid = self.oid+1
        self.name = "/sphere"+str(self.oid)
        self.marker = Marker()
        self.marker.id = self.oid
        self.marker.ns = "/shapes"
        self.marker.header.frame_id = self.name
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = sx
        self.marker.scale.y = sy
        self.marker.scale.z = sz
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.markerArray.markers.append(self.marker)

    def publishObjects (self):
        if not rospy.is_shutdown ():
            now = rospy.Time.now ()
            r = rospy.Rate(10)
            for n, obj in self.objects.iteritems ():
                self.broadcaster.sendTransform \
                    (obj.position [0:3], (obj.position [4],
                                          obj.position [5],
                                          obj.position [6],
                                          obj.position [3]), now,
                     obj.frameId, "map")
            for m in self.markerArray.markers:
                    #pos = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
                    pos = (0,0,0)
                    ori = ( m.pose.orientation.x,  \
                            m.pose.orientation.y, \
                            m.pose.orientation.z, \
                            m.pose.orientation.w)
                    self.broadcaster.sendTransform \
                        (pos, ori, now, m.header.frame_id, "/"+self.referenceFrame)

            self.pubRobots ['marker'].publish (self.markerArray)


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
            self.transform.header.stamp.secs = now.secs
            self.transform.header.stamp.nsecs = now.nsecs
            self.transform.header.seq = self.js.header.seq
            self.computeRobotPosition (self)
            self.js.velocity = len (self.js.position)*[0.,]
            self.js.effort = len (self.js.position)*[0.,]

            rospy.loginfo (self.transform)
            rospy.loginfo (self.js)
            self.broadcaster.sendTransform \
                (self.transform.transform.translation,
                 self.transform.transform.rotation,
                 now, self.tf_root, self.referenceFrame)
            self.pubRobots ['robot'].publish (self.js)


    def __call__ (self, args):
        self.robotConfig = args
        self.publish ()
