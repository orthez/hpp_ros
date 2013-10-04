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

# To use this package with hrp2_14
#  1. install hpp-wholebody-step-corba (for instance using robotpkg)
#  2. run hpp-wholebody-step-server
#  3. roslaunch hpp_ros hpp_ros.launch
#  4. in rviz, add a robot model (CTRL-n then choose RobotModel)
#  5. execute this script: python script.py
#  6. in rviz select "/odom" as "Fixed Frame"

import time
from hpp.corbaserver import Client
from hpp_ros import RobotStatePublisher

cl = Client ()
cl.robot.loadHrp2Model (0.05)
jn = cl.robot.getJointNames (0)

r = RobotStatePublisher (jn [6:])
q = cl.robot.getCurrentConfig (0)
time.sleep (2.)

r.publish (q)
