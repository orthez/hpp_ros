# Copyright (c) 2013 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp_ros.
# hpp_ros is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_ros is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_ros.  If not, see
# <http://www.gnu.org/licenses/>.

import time

class PathPlayer (object):
    dt = 0.01
    def __init__ (self, client, publisher) :
        self.client = client
        self.publisher = publisher

    def __call__ (self, pathId) :
        length = self.client.problem.pathLength (pathId)
        t = 0
        while t < length :
            q = self.client.problem.configAtDistance (pathId, t)
            self.publisher.robotConfig = q
            self.publisher.publish ()
            t += self.dt
            time.sleep (self.dt)

