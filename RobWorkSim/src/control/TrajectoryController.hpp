#ifndef TRAJECTORYCONTROLLER_HPP
#define TRAJECTORYCONTROLLER_HPP

#include <sandbox/control/JointController.hpp>
#include <sandbox/VelocityRamps/SyncVelocityRamp.hpp>
#include <sandbox/simulation/SimulatedController.hpp>

#include <dynamics/RigidDevice.hpp>

/**
 * @brief a JointController that use a PD loop on each joint
 * to control the velocity such that the position target is
 * reached at the same time. The PD controls the joint position and
 * velocity from a generated synchronous ramp profile.
 */
class TrajectoryController: public JointController, public rwlibs::simulation::SimulatedController{

public:

	TrajectoryController(RigidDevice* rdev, const rw::kinematics::State& state):
        JointController(&rdev->getModel()),
        _ddev(rdev),
        _time(0.0),
        _target(rdev->getModel().getQ(state)),
        _lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
        _velramp(&(rdev->getModel())),
        _currentQ(_target)
    {
        _velramp.setTarget(_target,_target);
    }

    virtual ~TrajectoryController(){};

    unsigned int getControlModes(){
        return POSITION || VELOCITY;
    }

    void setControlMode(ControlMode mode);

    void setTargetPos(const rw::math::Q& target);

    void setTargetVel(const rw::math::Q& vals);

    void setTargetAcc(const rw::math::Q& vals);

    /**
     * @brief updates the state of the dynamicdevice
     */
    void update(double dt, rw::kinematics::State& state);

    /**
     *
     * @param state
     */
    void reset(const rw::kinematics::State& state);

    rw::math::Q getQ(){
        return _currentQ;
    }

    rw::math::Q getQd(){ return _target;}

    Controller* getController(){ return this;};

private:
    RigidDevice *_ddev;
    double _time;
    rw::math::Q _target;
    rw::math::Q _lastError;
    rw::sandbox::SyncVelocityRamp _velramp;
    rw::math::Q _currentQ;
    rw::math::Q _maxVel;
    rw::math::Q _x;
    int _mode;


};


#endif /*TrajectoryController_HPP_*/
