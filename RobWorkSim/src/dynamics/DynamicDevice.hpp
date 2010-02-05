#ifndef DYNAMICDEVICE_HPP_
#define DYNAMICDEVICE_HPP_

#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include "Body.hpp"

/**
 * @brief base class for dynamic devices that has dynamic state values
 * such as velocity and acceleration.
 */
class DynamicDevice {

public:
	/**
	 * @brief destructor
	 */
    virtual ~DynamicDevice(){};

    /**
     * @brief gets the position
     */
    rw::math::Q getQ(const rw::kinematics::State& state){
        return _dev->getQ(state);
    }

    /**
     * @brief gets the kinematic model of the DynamicDevice.
     */
    rw::models::Device& getModel(){
        return *_dev;
    }

    /*rw::models::Device* getModel(){
        return _dev;
    }*/

    // Joint acceleration
    //void setQdd(const rw::kinematics::Q& qdd, const rw::kinematics::State& state);
    //rw::math::Q getQdd(const rw::kinematics::State& state);

    dynamics::Body* getBase(){ return _base;};

protected:

    DynamicDevice(dynamics::Body* base, rw::models::Device* dev, rw::models::WorkCell* wc):
        _dev(dev),
        _wc(wc),
        _base(base)
    {}


    rw::models::Device *_dev;
    rw::models::WorkCell *_wc;
    dynamics::Body* _base;
private:
	DynamicDevice();

};


#endif /*DYNAMICDEVICE_HPP_*/
