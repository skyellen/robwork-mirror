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
#include "TaskDescription.hpp"



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
 * Contains detailed evaluation of specific gripper design.
 * 
 * @todo ...
 */
struct GripperQuality
{
	// typedefs
	typedef rw::common::Ptr<GripperQuality> Ptr;
	
	// constructors
	GripperQuality() :
		nOfExperiments(0),
		nOfSuccesses(0),
		nOfSamples(0),
		//shape(0.0),
		coverage(0.0),
		success(0.0),
		wrench(0.0),
		topwrench(0.0),
		robustness(0.0),
		maxstress(0.0),
		quality(0.0)
	{}
	
	// methods
	friend std::ostream& operator<<(std::ostream& stream, const GripperQuality& q)
	{
		stream 	<< "GripperQuality:\n"
				<< "- experiments= " << q.nOfExperiments << '\n'
				<< "- successes= " << q.nOfSuccesses << '\n'
				<< "- samples= " << q.nOfSamples << '\n'
				//<< "- shape= " << q.shape << '\n'
				<< "- coverage= " << q.coverage << '\n'
				<< "- success= " << q.success << '\n'
				<< "- wrench= " << q.wrench << '\n'
				<< "- topwrench= " << q.topwrench << '\n'
				<< "- robustness= " << q.robustness << '\n'
				<< "- maxstress= " << q.maxstress << '\n'
				<< "- volume= " << q.volume << '\n'
				<< "- quality= " << q.quality << std::endl;
				
		return stream;
	}
	
	// data	
	int nOfExperiments; // Number of performed experiments.
	
	int nOfSuccesses; // Number of succesful grasps (filtered).
	int nOfSamples; // Number of generated samples (filtered).
	
	double coverage; // Ratio of filtered succesful grasps to filtered all samples.
	double success; // Ratio of succesful grasps to all generated grasps.
	double wrench; // Average wrench of succesful grasps.
	double topwrench; // Average quality of top 20% of grasps.
	double robustness; // Robustness of succesful grasps.
	double maxstress; // Max. stress a gripper takes.
	double volume; // The volume of the gripper's jaw.
	double quality; // Ultimate measurement of gripper quality.
};



/**
 * @class Gripper
 * @brief Gripper device (parallel jaw gripper) with parametrized geometry and kinematic and dynamic parameters.
 * 
 * Gripper is described by its geometric and kinematic & dynamic parameters. It contains geometries for jaws (shared between
 * left & right jaw) and gripper base. These geometries can either be initialized from mesh (e.g. from STL file) or
 * procedurally generated using JawPrimitive and Box primitive for fingers and base respectively.
 * Kinematic & dynamic parameters include:
 * - min. and max. opening
 * - max. force
 * - TCP position
 * 
 * Gripper contains a GripperQuality structure which describes gripper's qualities.
 * 
 * Default gripper constructor creates gripper with following parameters:
 * - parametrized base: Box 0.15 x 0.1 x 0.05
 * - parametrized jaws: simple cuboids 0.1 x 0.025 x 0.2
 * - force: 50 N
 * - opening: 0.05
 * - jawdist: 0
 * - TCP offset: 0.075 from base
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
		
		double getForce() { return _force; }
		void setForce(double force) { _force = force; _quality.maxstress = getMaxStress(); }
		
		rw::math::Transform3D<> getTCP() { return _tcp; }
		void setTCP(rw::math::Transform3D<> tcp) { _tcp = tcp; }
		
		double getJawdist() { return _jawdist; }
		void setJawdist(double jawdist) { _jawdist = jawdist > 0.0 ? jawdist : 0.0; }
		
		double getOpening() { return _opening; }
		void setOpening(double opening) { _opening = opening; }
		
		/*double getStroke() { return _stroke; }
		void setStroke(double stroke) {
			_stroke = stroke;
		}*/
		
		bool isJawParametrized() const { return _isJawParametrized; }
		
		/**
		 * @brief Returns vector of jaw parameters.
		 * 
		 * If jaw geometry is not parametrized, returns Q of zero length.
		 */
		rw::math::Q getJawParameters() const
		{
			if (_isJawParametrized) {
				return _jawParameters;
			} else {
				return rw::math::Q();
			}
		}
		
		rw::geometry::Geometry::Ptr getJawGeometry()
		{
			using rw::geometry::Geometry;
			using rw::geometry::JawPrimitive;
			using rw::common::ownedPtr;
			
			if (_isJawParametrized) {
				std::cout << "Generating jaw geometry..." << std::endl;
				_leftGeometry = ownedPtr(new Geometry(new JawPrimitive(_jawParameters), std::string("LeftFingerGeo")));
				//_rightGeometry = ownedPtr(new Geometry(new JawPrimitive(_jawParameters), std::string("RightFingerGeo")));
			}
			
			return new Geometry(*_leftGeometry);
		}
		
		/// Set jaws geometry to a mesh.
		void setJawGeometry(rw::geometry::Geometry::Ptr geo)
		{
			_leftGeometry = geo;
			_rightGeometry = geo;
			_isJawParametrized = false;
		}
		
		/**
		 * @brief Set parametrized jaw geometry.
		 * 
		 * JawPrimitive is used to generate geometry mesh from given parameters.
		 * Parameters are (in order):
		 * - type (0 for prismatic, 1 for cylindrical)
		 * - length
		 * - width
		 * - depth
		 * - chamfer depth
		 * - chamfer angle
		 * - cut position
		 * - cut depth
		 * - cut angle
		 * - cut radius
		 * - cut tilt
		 */
		void setJawGeometry(rw::math::Q params)
		{			
			_jawParameters = params;
			//_leftGeometry = NULL;
			//_rightGeometry = NULL;
			_isJawParametrized = true;
			
			_quality.maxstress = getMaxStress();
			_quality.volume = getVolume();
		}
		
		bool isBaseParametrized() const { return _isBaseParametrized; }
		
		/**
		 * @brief Returns vector of base parameters.
		 * 
		 * If base geometry is not parametrized, returns Q of zero length.
		 */
		rw::math::Q getBaseParameters() const
		{
			if (_isBaseParametrized) {
				return _baseParameters;
			} else {
				return rw::math::Q();
			}
		}
		
		rw::geometry::Geometry::Ptr getBaseGeometry()
		{
			using rw::geometry::Geometry;
			using rw::geometry::Box;
			using rw::common::ownedPtr;
			
			if (_isBaseParametrized) {
				_baseGeometry = ownedPtr(new Geometry(new Box(_baseParameters), std::string("BaseGeo")));
			}
			
			return _baseGeometry;
		}
		
		/// Set base geometry to a mesh.
		void setBaseGeometry(rw::geometry::Geometry::Ptr geo) { _baseGeometry = geo; _isBaseParametrized = false; }
		
		/**
		 * @brief Sets parametrized base geometry.
		 * 
		 * Box is used to generate mesh using given parameters.
		 * Parameters are the dimensions of the box (length x width x depth).
		 */
		void setBaseGeometry(rw::math::Q params)
		{
			_baseParameters = params;
			//_baseGeometry = NULL;
			_isBaseParametrized = true;
		}
		
	// DEPRECATED
		//rw::geometry::JawPrimitive::Ptr getGeometry() { return _jaw; }
		//void setGeometry(rw::geometry::JawPrimitive::Ptr geometry) { _jaw = geometry; }
	// /DEPRECATED
		
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
		 * 
		 * @todo Make this use only TaskDescription.
		 */
		void updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
			rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state,
			TaskDescription::Ptr td);
		
		/// Get gripper quality measurement structure.
		GripperQuality& getQuality() { return _quality; }
		
		/**
		 * @brief Calculates parametrized gripper's crossection breadth at the point along its length.
		 * 
		 * @param x [in] distance from the base
		 * 
		 * @return crossection height
		 */
		double getCrossHeight(double x) const;
		
		/**
		 * @brief Calculates worst case maximum stress for the gripper.
		 * 
		 * It's calculated by assuming max. grippers force acting on the tip (which gives the largest moment).
		 * 
		 * @return Stress.
		 */
		double getMaxStress() const;
		
		/**
		 * @brief Calculates the volume of the gripper's jaw outline.
		 * 
		 * This is an estimation of material neccesary to make the finger.
		 * 
		 * @return Volume [m^3].
		 */
		double getVolume() const;
		
	// friends
		friend class rw::loaders::GripperXMLLoader;
		
	private:
	// data
		std::string _name;
		
		// gripper geometry parametrizations
		bool _isBaseParametrized;
		bool _isJawParametrized;
		rw::math::Q _baseParameters;
		rw::math::Q _jawParameters;
		
		// gripper part geometries
		rw::geometry::Geometry::Ptr _baseGeometry;
		rw::geometry::Geometry::Ptr _leftGeometry;
		rw::geometry::Geometry::Ptr _rightGeometry;
		
		//rw::geometry::JawPrimitive::Ptr _jaw;
		
		// kinematic & dynamic parameters
		math::Transform3D<> _tcp;
		double _jawdist;
		double _opening;
		double _force;
		double _stroke;
		
		// quality
		GripperQuality _quality;
};
}} // end namespaces
