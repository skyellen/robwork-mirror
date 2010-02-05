#ifndef PDCONTROLLER_HPP_
#define PDCONTROLLER_HPP_

#include <sandbox/control/JointController.hpp>
#include <sandbox/simulation/SimulatedController.hpp>

#include <dynamics/RigidDevice.hpp>

/**
 * @brief a JointController that use a PD loop on each joint
 * to control the velocity such that the position target is
 * reached
 *
 */

class PDController: public JointController, public rwlibs::simulation::SimulatedController {

public:

    PDController(RigidDevice* rdev, const rw::kinematics::State& state):
        JointController(&rdev->getModel()),
        _ddev(rdev),
        _lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
        _target(rdev->getModel().getQ(state)),
        _currentQ(_target),_currentVel(rw::math::Q::zero(_target.size())),
        _targetVel(rw::math::Q::zero(_target.size()))
    {}

    virtual ~PDController(){};

    unsigned int getControlModes(){
        return POSITION || VELOCITY;
    }

    void setControlMode(ControlMode mode){
        if(mode!=POSITION || mode !=VELOCITY )
            RW_THROW("Unsupported control mode!");
        _mode = mode;
    }

    void setTargetPos(const rw::math::Q& target){
        _target = target;
    }

    void setTargetVel(const rw::math::Q& vals){
        _targetVel = vals;
    }

    void setTargetAcc(const rw::math::Q& vals){};

    /**
     * @brief updates the state of the dynamicdevice
     */
    void update(double dt, rw::kinematics::State& state) {
        const double P = 10;
        const double D = 0.3;
        rw::math::Q q = _ddev->getModel().getQ(state);
        rw::math::Q error = _target-q;
        // std::cout  << "PD TARGET: " << _target << std::endl;
        // std::cout  << "PD ERROR: " << error << std::endl;
        rw::math::Q nvel = P*error + (error-_lastError)*D;
        _lastError = error;

        _ddev->setVelocity(_targetVel + nvel, state);

        _currentVel = (q - _currentQ)/dt;
        _currentQ = q;
    }

    rw::math::Q getQ(){
        return _currentQ;
    }

    rw::math::Q getQd(){
        return _currentVel;
    }

    void reset(const rw::kinematics::State& state){
        _currentQ = _ddev->getModel().getQ(state);
        _target = _currentQ;
        _targetVel = rw::math::Q::zero(_currentQ.size());
        //_time = 0;
    }

    Controller* getController(){ return this;};

private:
    RigidDevice *_ddev;
    rw::math::Q _maxVel;
    rw::math::Q _lastError, _target, _currentQ, _currentVel;
    rw::math::Q _targetVel;
    int _mode;
};

typedef rw::common::Ptr<PDController> PDControllerPtr;


#endif /*PDController_HPP_*/
