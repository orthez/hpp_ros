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

import roslib
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster, transformations

class RobotStatePublisher (object):
    def __init__ (self, jointNames):
        self.pub = rospy.Publisher ('/joint_states', JointState)
        rospy.init_node ('hpp')
        self.broadcaster = TransformBroadcaster ()
        self.js = JointState ()
        self.js.name = jointNames
        self.odom_trans = TransformStamped ()
        self.odom_trans.header.frame_id = "odom";
        self.odom_trans.child_frame_id = "base_link"


    def publish (self, q):
        if not rospy.is_shutdown ():
            now = rospy.Time.now ()
            self.js.header.stamp.secs = now.secs
            self.js.header.stamp.nsecs = now.nsecs
            self.js.header.seq += 1
            self.odom_trans.header.stamp.secs = now.secs
            self.odom_trans.header.stamp.nsecs = now.nsecs
            self.odom_trans.header.seq = self.js.header.seq

            self.odom_trans.transform.translation.x = q [0]
            self.odom_trans.transform.translation.y = q [1]
            self.odom_trans.transform.translation.z = q [2]
            self.odom_trans.transform.rotation = \
                transformations.quaternion_from_euler (ak = q [3], aj = q [4],
                                                       ai = q [5], axes='szyx')

            self.js.position = q[6:]
            self.js.velocity = 40*[0.,]
            self.js.effort = 40*[0.,]

            rospy.loginfo (self.odom_trans)
            rospy.loginfo (self.js)
            self.broadcaster.sendTransform \
                (q [0: 3], self.odom_trans.transform.rotation,
                 now, "base_link", "odom")
            self.pub.publish (self.js)

