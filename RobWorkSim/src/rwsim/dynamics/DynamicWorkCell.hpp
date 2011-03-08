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
#include <rw/kinematics/State.hpp>


#include "Body.hpp"
#include "ContactDataMap.hpp"
#include "MaterialDataMap.hpp"
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include "DynamicDevice.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics @{
    /**
     * @brief the WorkCellDimension describe a center and the box halflengths of
     * the space that the WorkCell expands.
     *
     */
    struct WorkCellDimension {
    public:
        WorkCellDimension(const rw::math::Vector3D<>& c, const rw::math::Vector3D<>& box):
            center(c),boxDim(box){}
        rw::math::Vector3D<> center, boxDim;

    };

    /**
     * @brief the DynamicWorkcell class is a container class for dynamic
     * information/data in a workcell, much like WorkCell is a container
     * class for the kinematic information/data in a workcell
     *
     * The dynamic description includes:
     * Body: severel different forms of bodies that may be constrained by rules
     * Controllers: severel controllers that in some way control/influence the bodies
     * Sensors: bodies that have tactile sensing capability
     *
     */
    class DynamicWorkCell
    {
    public:
        typedef std::vector<Body*> BodyList;
        typedef std::vector<DynamicDevice*> DeviceList;
        typedef std::vector<rwlibs::simulation::SimulatedController::Ptr> ControllerList;
        typedef std::vector<rwlibs::simulation::SimulatedSensor::Ptr> SensorList;
        typedef rw::common::Ptr<DynamicWorkCell> Ptr;


        /**
         * @brief Constructor
         */
    	DynamicWorkCell(rw::models::WorkCell::Ptr workcell,
                        const BodyList& bodies,
                        const DeviceList& devices,
                        const ControllerList& controllers);

    	/**
    	 * @brief destructor
    	 */
    	virtual ~DynamicWorkCell();

        /**
         * @brief gets a list of all bodies in the dynamic workcell
         */
        const BodyList& getBodies(){ return _bodies;};

        /**
         * @brief find a specific body with name \b name
         * @param name [in] name of body
         * @return body if found, NULL otherwise
         */
    	Body* findBody(const std::string& name) const;

    	/**
    	 * @brief find a specific body with name \b name and type \b T
    	 * @param name [in] name of body
    	 * @return body if found, NULL otherwise
    	 */
        template<class T>
        T* findBody(const std::string& name) const{
            Body *body = findBody(name);
            if(body==NULL) return NULL;
            return dynamic_cast<T*>(body);
        }

        /**
         * @brief find all bodies with type \b T
         * @return list of all bodies of type \b T
         */
        template<class T>
        std::vector<T*> findBodies() const{
            std::vector<T*> bodies;
            BOOST_FOREACH(Body* b, _bodies ){
                if(T* tb = dynamic_cast<T*>(b)){
                    bodies.push_back(tb);
                }
            }
            return bodies;
        }

    	/**
    	 * @brief gets a list of all dynamic devices in the dynamic workcell
    	 */
    	const DeviceList& getDynamicDevices(){ return _devices; };

    	/**
    	 * @brief find a dynamic device of name \b name
    	 * @param name [in] name of device
    	 * @return a device with name \b name or null
    	 */
    	DynamicDevice* findDevice(const std::string& name);

    	/**
    	 * @brief gets a list of all controllers in the dynamic workcell
    	 */
        const ControllerList& getControllers(){
             return _controllers;
        }

        /**
         *
         * @param body
         */
        const SensorList& getSensors(){
            return _sensors;
        };

        void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){
            sensor->addStateData( _workcell->getStateStructure() );
            _sensors.push_back(sensor);
        };

        /**
         * @brief
         */
        //const std::vector<Constraint>& getConstraints();

        /**
         * @brief adds a body to the dynamic workcell.
         *
         * Notice that this will change the length of the default
         * State.
         */
        void addBody(Body* body);

    	/**
    	 * @brief adds a body controller to the dynamic workcell.
    	 *
    	 * Notice that this will change the length of the default
    	 * State.
    	 */
    	void addController(rwlibs::simulation::SimulatedController::Ptr manipulator){
    	    //TODO: change STATE and WorkCell accordingly
    	    manipulator->addStateData( _workcell->getStateStructure() );
    		_controllers.push_back(manipulator);
    	}

    	rwlibs::simulation::SimulatedController::Ptr findController(const std::string& name);

    	/**
    	 * @brief gets the static contact data information
    	 */
    	ContactDataMap& getContactData(){
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
          * @brief gets the body associated with frame f if any.
          */
        Body* getBody(rw::kinematics::Frame *f);

    	/**
    	 * @brief gets the default kinematic workcell
    	 */
    	rw::models::WorkCell* getWorkcell(){ return _workcell.get(); };

    	/**
    	 * @brief the collision margin describe how close
    	 */
    	double getCollisionMargin(){
    	    return _collisionMargin;
    	}

        void setCollisionMargin(double margin){
            _collisionMargin = margin;
        }

        WorkCellDimension getWorldDimension(){
            return _worldDimension;
        }

        void setGravity(const rw::math::Vector3D<>& grav){
            _gravity = grav;
        }

        const rw::math::Vector3D<>& getGravity(){
            return _gravity;
        }

        rw::common::PropertyMap& getEngineSettings(){
        	return _engineSettings;
        }

    private:
        // the workcell
        rw::models::WorkCell::Ptr _workcell;
        // length of nr of bodies
        BodyList _bodies;
        DeviceList _devices;
        // list of controllers in the workcell
        ControllerList _controllers;

        double _collisionMargin;
        WorkCellDimension _worldDimension;
        rw::math::Vector3D<> _gravity;

        // list of sensors
        SensorList _sensors;

        // deprecated
        std::map<rw::kinematics::Frame*,Body*> _frameToBody;

        MaterialDataMap _matDataMap;
        ContactDataMap _contactDataMap;

        rw::common::PropertyMap _engineSettings;

    };
    //! @}
} // namespace dynamics
}

#endif /*DYNAMICWORKCELL_HPP_*/
