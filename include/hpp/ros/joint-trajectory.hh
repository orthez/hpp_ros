//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp_ros
// hpp_ros is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp_ros is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp_ros  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_ROS_JOINT_TRAJECTORY_HH
# define HPP_ROS_JOINT_TRAJECTORY_HH

namespace hpp {
  namespace ros {
    using trajectory_msgs::JointTrajectory;
    using trajectory_msgs::JointTrajectoryPoint;
    using hpp::model::ConfigurationIn_t;
    using hpp::model::Configuration_t;
    using hpp::model::size_type;
    using hpp::model::value_type;
    using hpp::model::DevicePtr_t;
    using hpp::core::JointVector_t;
    using hpp::core::PathPtr_t;
    using hpp::core::PathVectorPtr_t;

    /// Convert an hpp configuration to a JointTrajectoryPoint
    /// \param configuration intput configuration,
    /// \retval jtPoint joint trajectory point
    /// We assume that the joints are in the same order in both data structures.
    inline void configurationToJointTrajectoryPoint
    (ConfigurationIn_t configuration, JointTrajectoryPoint& jtPoint)
    {
      for (size_type i=0; i < configuration.size (); ++i) {
	jtPoint.positions [i] = configuration [i];
      }
    }
    /// Convert hpp::core::PathVector to ros JointTrajectory
    /// \param robot robot the path corresponds to,
    /// \param path vector of straight interpolation in joint space,
    /// \retval jointTrajectory trajectory in ros format.
    void pathVectorToJointTrajectory (const DevicePtr_t& robot,
				      const PathVectorPtr_t& path,
				      JointTrajectory& jointTrajectory)
    {
      // joint_names
      jointTrajectory.joint_names.clear ();
      const JointVector_t& joints = robot->getJointVector ();
      for (std::size_t i=0; i<jointTrajectory.joint_names.size (); ++i) {
	jointTrajectory.joint_names.push_back (joints [i]->name ());
      }
      // Fill trajectory, starting by the initial configuration
      Configuration_t q = (*path) (0);
      JointTrajectoryPoint jtp;
      configurationToJointTrajectoryPoint (q, jtp);
      jointTrajectory.points.push_back (jtp);
      for (std::size_t i=0; i<path->numberPaths (); ++i) {
	PathPtr_t localPath = path->pathAtRank (i);
	value_type t = localPath->timeRange ().second;
	q = (*localPath) (t);
	configurationToJointTrajectoryPoint (q, jtp);
	jointTrajectory.points.push_back (jtp);
      }
    }

  } // namespace ros
} // namespace hpp

#endif // HPP_ROS_JOINT_TRAJECTORY_HH
