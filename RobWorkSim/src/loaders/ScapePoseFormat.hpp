/*
 * ScapePoseFormat.hpp
 *
 *  Created on: 06-02-2009
 *      Author: jimali
 */

#ifndef SCAPEPOSEFORMAT_HPP_
#define SCAPEPOSEFORMAT_HPP_

#include <vector>
#include <rw/kinematics/State.hpp>
#include <dynamics/RigidBody.hpp>

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
			const std::vector<dynamics::RigidBody*>& bodies,
			const rw::kinematics::State state,
			const std::string& ObjectName,
			const std::string& SimulationDescription);

	/**
	 * @brief save a multiple pose of all rigid bodies in a workcell.
	 */
	static void savePoses(const std::string& FileNameAndPath,
			const std::vector<dynamics::RigidBody*>& bodies,
			const std::vector< rw::kinematics::State> states,
			const std::string& ObjectName,
			const std::string& SimulationDescription);

};


#endif /* SCAPEPOSEFORMAT_HPP_ */
