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
		struct GripperQuality
		{
			typedef rw::common::Ptr<GripperQuality> Ptr;
			
			GripperQuality() :
				nOfExperiments(0),
				shapeQ(0),
				coverageQ(0),
				successQ(0),
				wrenchQ(0),
				interferenceQ(0),
				finalQ(0) {}
			
			friend std::ostream& operator<<(std::ostream& stream, const GripperQuality& q)
			{
				stream << "[N: " << q.nOfExperiments << ", GEO: " << q.shapeQ << ", COV: ";
				stream << q.coverageQ << ", WSM: " << q.wrenchQ << ", INT: " << q.interferenceQ;
				stream << ", Q: " << q.finalQ << "]" << std::endl;
				
				return stream;
			}
			
			int nOfExperiments;
			double shapeQ;
			double coverageQ;
			double successQ; // done
			double wrenchQ; // ?
			double interferenceQ;
			double finalQ;
		};
		
		
		
		/**
		 * @class Gripper
		 * @brief Gripper device (parallel jaw gripper) with parametrized geometry and kinematic and dynamic parameters
		 */
		class Gripper // : public TreeDevice
		{
			public:
				//! @brief Smart pointer
				typedef rw::common::Ptr<Gripper> Ptr;
				
				// constructors
				//! @brief Basic constructor
				Gripper(const std::string& name="gripper");
				
				//! @brief Destructor
				virtual ~Gripper() {}
				
				//methods
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
				void setName(const std::string& name) { _name = name; }
				
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
				
				/**
				 * @brief Places the gripper in the workcell
				 * NOT IMPLEMENTED
				 */
				//void addToWorkcell(WorkCell::Ptr wc, kinematics::Frame* base, math::Transform3D<> transform);
				
				/**
				 * @brief Updates selected gripper device in the workcell according to data in this class
				 * THIS IS A DIRTY HACK
				 */
				void updateGripper(rw::models::WorkCell::Ptr wc, rwsim::dynamics::DynamicWorkCell::Ptr dwc,
					rw::models::TreeDevice::Ptr dev, rwsim::dynamics::RigidDevice::Ptr ddev, rw::kinematics::State& state);
				
				//! @brief Save gripper data to XML file
				//void saveToXML(const std::string& filename) const;
				
				//! @brief Get gripper quality measurement structure
				GripperQuality::Ptr getQuality()
				{
					if (_quality == NULL) {
						_quality = rw::common::ownedPtr(new GripperQuality);
					}
					
					return _quality;
				}
				
				// friends
				//! @brief Output operator
				//friend std::ostream& operator<<(std::ostream& stream, const Gripper& gripper);
				
				friend class rw::loaders::GripperXMLLoader;
				
			private:
				std::string _name;
				
				//WorkCell::Ptr _wc;
				//TreeDevice::Ptr _device;
				
				// geometry parameters
				rw::geometry::JawPrimitive::Ptr _jaw;
				math::Transform3D<> _tcp;
				
				// kinematic & dynamic parameters
				double _jawdist;
				double _opening;
				double _force;
				
				// quality
				GripperQuality::Ptr _quality;
		};
}} // end namespaces
