#ifndef KINEMATICDEVICE_HPP_
#define KINEMATICDEVICE_HPP_

#include "DynamicDevice.hpp"
#include "KinematicBody.hpp"

/**
 * @brief a kinematic device is able to influence the simulated environment
 * but the device is not influenced by any external force as is the RigidDevice.
 *
 * This class is especially usefull for animating robot devices in a
 * simulated environment.
 */
class KinematicDevice: public DynamicDevice {

public:
    KinematicDevice(dynamics::Body *base,
					const std::vector<dynamics::KinematicBody*> bodies,
                    rw::models::Device *dev,
                    rw::models::WorkCell* wc);

    /**
     * @brief destructor
     * @return
     */
    virtual ~KinematicDevice();

    virtual void setQ(const rw::math::Q &q, const rw::kinematics::State& state){_q = q;};

    virtual rw::math::Q getQ(const rw::kinematics::State& state){return _q;}

    rw::math::Q getVelocity(const rw::kinematics::State& state){return _velQ;};

    rw::math::Q setVelocity(const rw::math::Q &vel, const rw::kinematics::State& state){return _velQ = vel;};

    /**
     * @brief get the kinematic bodies that this KinematicDevice controls. The
     * bodies are ordered such that device joint \b i maps to kinematic body  \b i
     * @return all bodies that the device controls.
     */
    const std::vector<dynamics::KinematicBody*>& getBodies();

    // parameters for velocity profile
    void setMaxAcc(const rw::math::Q& acc);
    rw::math::Q getMaxAcc();

    void setMaxVel(const rw::math::Q& vel);
    rw::math::Q getMaxVel();

private:
    std::vector<dynamics::KinematicBody*> _bodies;
    rw::math::Q _maxVel, _maxAcc;
    rw::math::Q _q, _velQ;
};


#endif /*KINEMATICDEVICE_HPP_*/
