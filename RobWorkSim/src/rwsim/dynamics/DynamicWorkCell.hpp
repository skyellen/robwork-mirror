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

#ifndef RWSIM_DYNAMICS_DYNAMICWORKCELL_HPP_
#define RWSIM_DYNAMICS_DYNAMICWORKCELL_HPP_

#include <vector>
#include <map>

#include <rw/models/WorkCell.hpp>

#include "Body.hpp"
#include "Constraint.hpp"
#include "ContactDataMap.hpp"
#include "MaterialDataMap.hpp"
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include "DynamicDevice.hpp"
#include <boost/any.hpp>
namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{
    /**
     * @brief the WorkCellDimension describe a center and the box halflengths of
     * the space that the WorkCell expands.
     *
     */
    struct WorkCellDimension {
    public:
    	/**
    	 * @brief Constructor.
    	 * @param c [in] center.
    	 * @param box [in] the halflengths of the workspace.
    	 */
        WorkCellDimension(const rw::math::Vector3D<>& c, const rw::math::Vector3D<>& box):
            center(c),boxDim(box){}
        //! @brief The center of the workspace.
        rw::math::Vector3D<> center;
        //! @brief The halftlengths of the workspace.
        rw::math::Vector3D<> boxDim;

    };

    /**
     * @brief the DynamicWorkcell class is a container class for dynamic
     * information/data in a workcell, much like WorkCell is a container
     * class for the kinematic information/data in a workcell
     *
     * The dynamic description includes:
     * - Body: severel different forms of bodies that may be constrained by rules
     * - Constraint: constraining the bodies to move in a specific way relative to each other
     * - Controllers: severel controllers that in some way control/influence the bodies
     * - Sensors: bodies that have tactile sensing capability
     *
     */
    class DynamicWorkCell
    {
    public:
        //! @brief Type for the collection of bodies.
        typedef std::vector<Body::Ptr> BodyList;
        //! @brief Type for the collection of constraints.
        typedef std::vector<Constraint::Ptr> ConstraintList;
        //! @brief Type for the collection of devices.
        typedef std::vector<DynamicDevice::Ptr> DeviceList;
        //! @brief Type for the collection of simulated controllers.
        typedef std::vector<rwlibs::simulation::SimulatedController::Ptr> ControllerList;
        //! @brief Type for the collection of simulated sensors.
        typedef std::vector<rwlibs::simulation::SimulatedSensor::Ptr> SensorList;
        //! @brief Smart pointer type for DynamicWorkCell.
        typedef rw::common::Ptr<DynamicWorkCell> Ptr;

        /**
         * @brief Constructor
         * @param workcell [in] a smart pointer to the rw::models::WorkCell
         */
    	DynamicWorkCell(rw::models::WorkCell::Ptr workcell);

    	/**
    	 * @brief Constructor
    	 * @param workcell [in] a smart pointer to the rw::models::WorkCell
    	 * @param bodies [in] a list of bodies.
    	 * @param allbodies [in] a list of bodies.
    	 * @param constraints [in] a list of constraints.
    	 * @param devices [in] a list of devices.
    	 * @param controllers [in] a list of controllers.
    	 */
    	DynamicWorkCell(rw::models::WorkCell::Ptr workcell,
                        const BodyList& bodies,
                        const BodyList& allbodies,
                        const ConstraintList& constraints,
                        const DeviceList& devices,
                        const ControllerList& controllers);

    	/**
    	 * @brief destructor
    	 */
    	virtual ~DynamicWorkCell();

        /**
         * @brief gets a list of all bodies in the dynamic workcell
         */
        const BodyList& getBodies() const { return _allbodies;};

        /**
         * @brief find a specific body with name \b name
         * @param name [in] name of body
         * @return body if found, NULL otherwise
         */
    	Body::Ptr findBody(const std::string& name) const;

    	/**
    	 * @brief find a specific body with name \b name and type \b T
    	 * @param name [in] name of body
    	 * @return body if found, NULL otherwise
    	 */
        template<class T>
        rw::common::Ptr<T> findBody(const std::string& name) const{
            Body::Ptr body = findBody(name);
            if(body==NULL) return NULL;
            return body.cast<T>();
        }

        /**
         * @brief find all bodies with type \b T
         * @return list of all bodies of type \b T
         */
        template<class T>
        std::vector<rw::common::Ptr<T> > findBodies() const{
            std::vector<rw::common::Ptr<T> > bodies;
            BOOST_FOREACH(Body::Ptr b, _allbodies ){
                if(rw::common::Ptr<T> tb = b.cast<T>()){
                    bodies.push_back(tb);
                }
            }
            return bodies;
        }

        /**
         * @brief Add a constraint to the dynamic workcell.
         * @param constraint [in] a smart pointer to the constraint to add.
         */
        void addConstraint(Constraint::Ptr constraint);

        /**
         * @brief gets a list of all constraints in the dynamic workcell
         */
        const ConstraintList& getConstraints() const { return _constraints;};

        /**
         * @brief find a specific constraint with name \b name
         * @param name [in] name of constraint
         * @return constraint if found, NULL otherwise
         */
    	Constraint::Ptr findConstraint(const std::string& name) const;

    	/**
    	 * @brief gets a list of all dynamic devices in the dynamic workcell
    	 * @return a list of dynamic devices.
    	 */
    	const DeviceList& getDynamicDevices() const { return _devices; };

        /**
         * @brief add a device to the dynamic workcell
         * @param device [in] a device
         */
        void addDevice(DynamicDevice::Ptr device){
        	device->registerIn( _workcell->getStateStructure() );
            _devices.push_back(device);
            _changedEvent.fire(DeviceAddedEvent, boost::any(device) );
        };

    	/**
    	 * @brief find a dynamic device of name \b name
    	 * @param name [in] name of device
    	 * @return a device with name \b name or null
    	 */
    	DynamicDevice::Ptr findDevice(const std::string& name) const;

        /**
         * @brief find a specific device with name \b name and type \b T
         * @param name [in] name of body
         * @return body if found, NULL otherwise
         */
        template<class T>
        rw::common::Ptr<T> findDevice(const std::string& name) const{
            DynamicDevice::Ptr dev = findDevice(name);
            if(dev==NULL) return NULL;
            return dev.cast<T>();
        }

    	/**
    	 * @brief gets a list of all controllers in the dynamic workcell
    	 */
        const ControllerList& getControllers() const {
             return _controllers;
        }

        /**
         * @brief get the list of sensors
         */
        const SensorList& getSensors() const {
            return _sensors;
        };

        /**
         * @brief add a sensor to the dynamic workcell
         * @param sensor [in] a simulated sensor
         */
        void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

        /**
         * @brief find a sensor
         * @param name [in] the sensor
         * @return
         */
        rwlibs::simulation::SimulatedSensor::Ptr findSensor(const std::string& name) const;

        /**
         * @brief find a sensor of a specific type.
         * @param name [in] name of sensor
         * @return
         */
        template<class T>
        rw::common::Ptr<T> findSensor(const std::string& name) const {
            rwlibs::simulation::SimulatedSensor::Ptr sensor = findSensor(name);
            if(sensor==NULL) 
		        return NULL;
        	return sensor.cast<T>();
        }

        /**
         * @brief adds a body to the dynamic workcell.
         *
         * Notice that this will change the length of the default
         * State.
         */
        void addBody(Body::Ptr body);

    	/**
    	 * @brief adds a body controller to the dynamic workcell.
    	 *
    	 * Notice that this will change the length of the default
    	 * State.
    	 */
    	void addController(rwlibs::simulation::SimulatedController::Ptr manipulator){
    	    //TODO: change STATE and WorkCell accordingly
    		if(!manipulator->getControllerModel()->isRegistered())
    			_workcell->add(manipulator->getControllerModel());
        	if(!manipulator->isRegistered())
        		manipulator->registerIn(_workcell->getStateStructure());

    		_controllers.push_back(manipulator);
    		_changedEvent.fire(ControllerAddedEvent, boost::any(manipulator) );
    	}

    	/**
    	 * @brief Find a simulated controller.
    	 * @param name [in] name of the controller.
    	 * @return the simulated controller if found, NULL otherwise.
    	 */
    	rwlibs::simulation::SimulatedController::Ptr findController(const std::string& name) const;

    	/**
    	 * @brief Find a simulated controller.
    	 * @param name [in] name of the controller.
    	 * @return the simulated controller if found, NULL otherwise.
    	 */
    	template<class T>
    	rw::common::Ptr<T> findController(const std::string& name) const {
    	    rwlibs::simulation::SimulatedController::Ptr controller = findController(name);
              if(controller==NULL) return NULL;
              return controller.cast<T>();
    	}

    	/**
    	 * @brief gets the static contact data information
    	 */
    	ContactDataMap& getContactData(){
    	    return _contactDataMap;
    	}

    	/**
    	 * @brief Get the static contact data information.
    	 * @return a reference to a constant rwsim::dynamics::ContactDataMap.
    	 */
    	const ContactDataMap& getContactData() const{
    	    return _contactDataMap;
    	}

    	/**
    	 * @brief gets the material data information, like friction
    	 * properties
    	 */
    	MaterialDataMap& getMaterialData(){
    	    return _matDataMap;
    	}

    	/**
    	 * @brief Get the material data information, like friction.
    	 * @return a reference to a constant rwsim::dynamics::ContactDataMap.
    	 */
    	const MaterialDataMap& getMaterialData() const {
    	    return _matDataMap;
    	}

        /**
          * @brief gets the body associated with frame f if any.
          */
        Body::Ptr getBody(rw::kinematics::Frame *f);

    	/**
    	 * @brief gets the default kinematic workcell
    	 */
    	rw::models::WorkCell::Ptr getWorkcell() const { return _workcell; };

    	/**
    	 * @brief the collision margin describe how close
    	 */
    	double getCollisionMargin(){
    	    return _collisionMargin;
    	}

    	/**
    	 * @brief Set the collision margin.
    	 * @param margin [in] the new margin.
    	 */
        void setCollisionMargin(double margin){
            _collisionMargin = margin;
        }

        /**
         * @brief Get dimensions of workspace.
         * @return the dimensions.
         */
        WorkCellDimension getWorldDimension(){
            return _worldDimension;
        }

        /**
         * @brief tests if a body is part of a device
         * @param body [in] the body to test for.
         * @return true if body is part of the device.
         */
        bool inDevice(rw::common::Ptr<const Body> body) const;

        /**
         * @brief Set the gravity in this dynamic workcell
         * @return gravity
         */
        void setGravity(const rw::math::Vector3D<>& grav){
            _gravity = grav;
            _changedEvent.fire(GravityChangedEvent, boost::any(grav) );
        }

        /**
         * @brief get the gravity in this dynamic workcell
         * @return gravity
         */
        const rw::math::Vector3D<>& getGravity() const {
            return _gravity;
        }

        /**
         * @brief get the settings and properties for the physics engine
         * @return propertymap
         */
        rw::common::PropertyMap& getEngineSettings(){
        	return _engineSettings;
        }

        /**
         * @brief Get the settings and properties for the physics engine.
         * @return a reference to a constant PropertyMap.
         */
        const rw::common::PropertyMap& getEngineSettings() const {
        	return _engineSettings;
        }

        //! @brief Types of events a DynamicWorkCell can emit.
        typedef enum{GravityChangedEvent,//!< If the gravity is changed.
                    ConstraintAddedEvent,//!< When a constraint is added.
                    BodyAddedEvent,      //!< When a body is added.
                    DeviceAddedEvent,    //!< When a device is added.
                    ControllerAddedEvent,//!< When a controller is added.
                    SensorAddedEvent     //!< When a sensor is added.
        } DWCEventType;

        //! @brief Type for an event listener.
        typedef boost::function<void(DWCEventType, boost::any)> DWCChangedListener;
        //! @brief Type for the event.
        typedef rw::common::Event<DWCChangedListener, DWCEventType, boost::any> DWCChangedEvent;

        /**
         * @brief Returns StateChangeEvent needed for subscribing and firing the event.
         * @return Reference to the StateChangedEvent
         */
        DWCChangedEvent& changedEvent() { return _changedEvent; }

    private:
        DWCChangedEvent _changedEvent;
        // the workcell
        rw::models::WorkCell::Ptr _workcell;
        // length of nr of bodies
        BodyList _bodies;
        BodyList _allbodies;
        ConstraintList _constraints;
        DeviceList _devices;
        // list of controllers in the workcell
        ControllerList _controllers;

        double _collisionMargin;
        WorkCellDimension _worldDimension;
        rw::math::Vector3D<> _gravity;

        // list of sensors
        SensorList _sensors;

        // deprecated
        std::map<rw::kinematics::Frame*, Body::Ptr> _frameToBody;

        MaterialDataMap _matDataMap;
        ContactDataMap _contactDataMap;

        rw::common::PropertyMap _engineSettings;

    };
    //! @}
} // namespace dynamics
}

#endif /*DYNAMICWORKCELL_HPP_*/
