/**
 * @file Gripper.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <iostream>
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwsim/rwsim.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>
#include "JawPrimitive.hpp"



namespace rw {
	namespace kinematics {
		class Frame;
		class State;
	}
	
	namespace loaders {
		class GripperXMLLoader;
	}
}



namespace rw {
	namespace models {
		
/**
 * Contains detailed evaluation of specific gripepr design.
 * 
 * @todo ...
 */
struct GripperQuality
{
	typedef rw::common::Ptr<GripperQuality> Ptr;
	
	GripperQuality() {}
	
	int nOfTargets;
	int nOfSamples;
	
	double shape; /// Shape evaluation based on geometry objective function.
	double coverage; /// Ratio of filtered succesful grasps to filtered all samples.
	double success; /// Ratio of succesful grasps to all generated grasps.
	//double wrenchQ; // ?
	//double interferenceQ;
	double quality; /// Ultimate measurement of gripper quality.
};



/**
 * @class Gripper
 * @brief Gripper device (parallel jaw gripper) with parametrized geometry and kinematic and dynamic parameters.
 */
class Gripper // : public TreeDevice
{
	public:
	// typedefs
		/// Smart pointer.
		typedef rw::common::Ptr<Gripper> Ptr;
		
	// constructors
		/// Basic constructor
		Gripper(const std::string& name="gripper");
		
		/// @brief Destructor
		virtual ~Gripper() {}
		
	// methods
		//! @brief Get name
		std::string getName() { return _name; }
		
		//! @brief Get access to the geometry
		rw::geometry::JawPrimitive::Ptr getGeometry() { return _jaw; }
		
		//! @brief Get TCP
		rw::math::Transform3D<> getTCP() { return _tcp; }
		
		//! @brief Get min. jaw separation
		double getJawdist() { return _jawdist; }
		
		//! @brief Get max. jaw separation
		double getOpening() { return _opening; }
		
		//! @brief Get max. force
		double getForce() { return _force; }
		
		//! @brief Sets name of the gripper
		void setName(const std::string& name) { _name = name; _dataFilename = name + "_data.xml"; }
		
		//! @brief Set gripper geometry
		void setGeometry(rw::geometry::JawPrimitive::Ptr geometry) { _jaw = geometry; }
		
		//! @brief Set TCP
		void setTCP(rw::math::Transform3D<> tcp) { _tcp = tcp; }
		
		//! @brief Set jawdist
		void setJawdist(double jawdist) { _jawdist = jawdist; }
		
		//! @brief Set opening
		void setOpening(double opening) { _opening = opening; }
		
		//! @brief Set force
		void setForce(double force) { _force = force; }
		
		//! @brief Get tasks
		rwlibs::task::GraspTask::Ptr getTasks() { return _tasks; }
		
		//! @brief Set tasks
		void setTasks(rwlibs::task::GraspTask::Ptr tasks) { _tasks = tasks; }
		
		//! @brief Loads tasks from a specified file
		void loadTasks(std::string filename);
		
		//! @brief Save tasks/results to a specified file
		void saveTasks(std::string filename);
		
		//! @brief Get data filename
		std::string getDataFilename() { return _dataFilename; }
		
		//! @brief Set data filename
		void setDataFilename(std::string filename) { _dataFilename = filename; }
		
		/**
		 * @brief Updates selected gripper device in the workcell according to data in this class
		 * THIS IS A DIRTY HACK
		 */
		void updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
			rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state);
		
		/// Get gripper quality measurement.
		GripperQuality::Ptr getQuality()
		{
			if (_quality == NULL) {
				_quality = rw::common::ownedPtr(new GripperQuality);
			}
			
			return _quality;
		}
		
	// friends
		friend class rw::loaders::GripperXMLLoader;
		
	private:
	// data
		std::string _name;
		
		rw::geometry::JawPrimitive::Ptr _jaw;
		math::Transform3D<> _tcp;
		
		double _jawdist;
		double _opening;
		double _force;
		
		GripperQuality::Ptr _quality;
		
		rwlibs::task::GraspTask::Ptr _tasks;
		std::string _dataFilename;
};
}} // end namespaces
