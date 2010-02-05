#ifndef FIXEDDEVICE_HPP_
#define FIXEDDEVICE_HPP_

#include <rw/models/Device.hpp>

#include "Body.hpp"
#include "BodyController.hpp"
#include "FixedLink.hpp"

namespace dynamics {

    /**
     * @brief Controls a device by applying force/torque at each joint. Each
     * joint is modelled as a fixed link and therefore has no dynamic properties 
     * like mass and inertia. 
     * 
     *  The device controlled by a FixedDeviceController will move until 
     * jointlimits is reached, goal position is reached or 
     * contact forces equals the controlling torque.
     */
    class FixedDevice : public BodyController
    {
    public:
        /**
         * @brief constructor
         */
    	FixedDevice( rw::models::Device &device , 
    			     std::vector<FixedLink*> links, 
    			     rw::kinematics::State& state);
    	
    	/**
    	 * @brief destructor
    	 */
    	virtual ~FixedDevice(){}
    	
    	/**
    	 * @brief returns the device that is being controlled by this
    	 * FixedDevice controller
    	 */
    	rw::models::Device* getSlaveDevice(){
    		return _device;
    	}

        rw::models::Device* getKinematicModel(){
             return _device;
         }
    	
    	/**
    	 * @brief Set the goal configuration 
    	 */
    	virtual void setQ(const rw::math::Q& q, rw::kinematics::State& state) {
    		_qGoal = q;
    	}
    	
    	void setGoalQ(const rw::math::Q& goal){
    		_qGoal = goal;
    	}
    	
    	/**
    	 * 
    	 */
    	const std::vector<FixedLink*>& getLinks(){
    		return _links;
    	}
    	
    	/**
    	 * @brief inherited from BodyController
    	 */
    	virtual void addForces(rw::kinematics::State &state, double time);
    	
    	void setKpPos( double p ){
    		_qKp = p;
    	}
    	
    	void setKpVel( double p ){
    		_vKp = p;
    	}
    	
    	void setKdPos( double p ){
    		_qKd = p;
    	}
    	
    	void setKdVel( double p ){
    		_vKd = p;
    	}
    	
    	 void resetState(rw::kinematics::State &state){
    		 _qGoal = _device->getQ(state);
    	 }
    	
    private:
        std::vector<FixedLink*> _links;
    	rw::models::Device *_device;
        rw::math::Q _qGoal;
        double _qKp,_qKd,_vKp,_vKd,_lastTime;
    };
    
} // dynamics

#endif /*FIXEDDEVICE_HPP_*/
