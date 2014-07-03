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

#ifndef HPP_ROS_JOINT_STATE_HH
# define HPP_ROS_JOINT_STATE_HH

# include <sstream>
# include <hpp/model/configuration.hh>

namespace hpp {
  namespace ros {
    using sensor_msgs::JointState;
    using hpp::model::JointPtr_t;
    using hpp::model::DevicePtr_t;
    using core::ConfigurationOut_t;

    /// Convert sensor_msgs::JointState to hpp::model::Configuration_t
    /// \param robot the kinematic chain the configuration corresponds to,
    /// \param jointState names and joint values,
    /// \retval configuration configuration.
    inline void jointStateToConfig (const DevicePtr_t& robot,
				    const JointState& jointState,
				    ConfigurationOut_t configuration)
    {
      if (jointState.position.size () != jointState.name.size ()) {
	std::ostringstream oss ("sensor_msgs::JointState position and name "
				"vector size differ: ");
	  oss << jointState.position.size () << " != "
	      << jointState.name.size ();
	throw std::runtime_error (oss.str ().c_str ());
      }
      for (std::size_t i=0; i<jointState.position.size (); ++i) {
	JointPtr_t joint = robot->getJointByName (jointState.name [i]);
	configuration [joint->rankInConfiguration ()] = jointState.position [i];
      }
    }
  } // namespace ros
} // namespace hpp

#endif // HPP_ROS_JOINT_STATE_HH
