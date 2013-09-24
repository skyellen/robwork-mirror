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
		/// Basic constructor.
		Gripper(const std::string& name="gripper");
		
		/// Destructor.
		virtual ~Gripper() {}
		
	// methods
		std::string getName() { return _name; }
		void setName(const std::string& name) { _name = name; }
		
		rwlibs::task::GraspTask::Ptr getTasks() { return _tasks; }
		void setTasks(rwlibs::task::GraspTask::Ptr tasks) { _tasks = tasks; }
		
		double getForce() { return _force; }
		void setForce(double force) { _force = force; }
		
		rw::math::Transform3D<> getTCP() { return _tcp; }
		void setTCP(rw::math::Transform3D<> tcp) { _tcp = tcp; }
		
		double getJawdist() { return _jawdist; }
		void setJawdist(double jawdist) { _jawdist = jawdist; }
		
		double getOpening() { return _opening; }
		void setOpening(double opening) { _opening = opening; }
		
	// DEPRECATED
		rw::geometry::JawPrimitive::Ptr getGeometry() { return _jaw; }
		void setGeometry(rw::geometry::JawPrimitive::Ptr geometry) { _jaw = geometry; }

		/// Loads tasks from a specified file
		void loadTasks(std::string filename);
		
		/// Save tasks/results to a specified file
		void saveTasks(std::string filename);
		
		/**
		 * @brief Updates selected gripper device in the workcell according to data in this class.
		 * 
		 * THIS IS A DIRTY HACK.
		 * Basically, workcell contains a skeleton of gripper device and dynamic gripper device. What this class and
		 * method does, is to dress this skeleton up. Geometries are removed and replaced in:
		 * - workcell
		 * - workcellscene (for displaying purposes)
		 * - collision detector
		 * - also dynamicworkcell has internal storage for some internal purposes apparently
		 * 
		 * This was done as the easiest way to try different gripper designs in the same workcell without
		 * having it reloaded, which requires restarting RobWorkStudio. Another approach would be to
		 * add gripper to the workcell and then remove it, but methods neccessary for are not yet implemented.
		 */
		void updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
			rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state);
		
		/// Get gripper quality measurement structure.
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
		
		// gripper geometry parametrizations
		bool _isBasePrimitive;
		bool _isJawPrimitive;
		rw::math::Q _baseParameters;
		rw::math::Q _jawParameters;
		
		// gripper part geometries
		rw::geometry::Geometry::Ptr _baseGeometry;
		rw::geometry::Geometry::Ptr _jawGeometry;
		
		rw::geometry::JawPrimitive::Ptr _jaw;
		
		// kinematic & dynamic parameters
		math::Transform3D<> _tcp;
		double _jawdist;
		double _opening;
		double _force;
		
		// quality & tasks
		GripperQuality::Ptr _quality;
		rwlibs::task::GraspTask::Ptr _tasks;
};
}} // end namespaces
