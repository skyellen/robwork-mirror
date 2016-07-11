/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWSIM_DYNAMICS_SCAPEPOSEFORMAT_HPP_
#define RWSIM_DYNAMICS_SCAPEPOSEFORMAT_HPP_

#include <vector>

#include <rw/common/Ptr.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace dynamics { class RigidBody; } }

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

	/**
	 * @brief saving object poses in Scape format.
	 *
	 * @note format:
	 * CObjectPoseContainer  Version: 0.1
	 * Description: Any one-line description # If not set, this whole line will not be streamed!
	 *	IsObjectsInitialized: 1
	 *	CPart  Version: 0.20
	 *	  IsInitialized: 1
	 *		Name: PartName # Must be in one word, i.e. no spaces
	 *		Description: Short part description - should not be more than 80 characters
	 *		CADFileName: path/cadfile.stl # Format is STL in ASCII-mode. Path is optional
	 *		# Part property regarding symmetry
	 *		Symmetry: PARTSYMMETRY
	 *	...
	 *	IsPosesInitialized: 1
	 *	CObjectPose  Pose: (0,0,150)  XDir: (0,0,-1) YDir: (0,1,0) Active: 1 Name: 140321
	 *	CObjectPose  Pose: (0,0,150)  XDir: (0,0,-1) YDir: (0,1,0) Active: 0 Name: 140321
	 *	CObjectPose  Pose: (0,0,150)  XDir: (0,0,-1) YDir: (0,1,0) Active: 0 Name: 140321
	 *	CObjectPose  Pose: (0,0,150)  XDir: (0,0,-1) YDir: (0,1,0) Active: 1 Name: 140321
	 *	...
	 */
	class ScapePoseFormat {
	public:
		/**
		 * @brief save a single pose of all rigid bodies in a workcell.
		 */
		static void savePoses(const std::string& FileNameAndPath,
				const std::vector<rw::common::Ptr<rwsim::dynamics::RigidBody> >& bodies,
				const rw::kinematics::State& state,
				const std::string& ObjectName,
				const std::string& SimulationDescription);

		/**
		 * @brief save a multiple pose of all rigid bodies in a workcell.
		 */
		static void savePoses(const std::string& FileNameAndPath,
				const std::vector<rw::common::Ptr<rwsim::dynamics::RigidBody> >& bodies,
				const std::vector< rw::kinematics::State> states,
				const std::string& ObjectName,
				const std::string& SimulationDescription);

	};
	//! @}
}
}

#endif /* SCAPEPOSEFORMAT_HPP_ */
