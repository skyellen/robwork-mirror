#ifndef JOINTCONTROLLER_HPP_
#define JOINTCONTROLLER_HPP_

#include <rw/math/Q.hpp>

#include <rw/common/macros.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>

#include "Controller.hpp"

/**
 * @brief the joint controller interface describe how to input to a joint controller.
 * The output Force,Vel,Pos... must be available in the class implementing JointController interface
 *
 */
class JointController: public Controller {

public:
    typedef enum {
        POSITION = 1 ,
        CNT_POSITION = 2,
        VELOCITY = 4,
        FORCE = 8,
        CURRENT = 16} ControlMode;

    /**
     * @brief destructor
     */
    virtual ~JointController(){};

    /**
     * @brief gets the control mode mask. Defines which types of control the JointController
     * supports
     */
    virtual unsigned int getControlModes() = 0;

    /**
     * @brief sets the control mode of this JointController. If the mode
     * is unsupported an exception is thrown
     */
    virtual void setControlMode(ControlMode mode) = 0;

    /**
     * @brief sets the target joint value for the current control mode.
     */
    virtual void setTargetPos(const rw::math::Q& vals) = 0;
    virtual void setTargetVel(const rw::math::Q& vals) = 0;
    virtual void setTargetAcc(const rw::math::Q& vals) = 0;

    /**
     * @brief
     *
     */
    virtual rw::models::Device& getModel(){
        return *_dev;
    }

    /**
     * @brief return the current position of the controlled robot
     * @return
     */
    virtual rw::math::Q getQ() = 0;


    virtual rw::math::Q getQd() = 0;
protected:

    JointController(rw::models::Device* dev):
        _dev(dev)
    {
        RW_ASSERT(_dev);
    }

private:
    rw::models::Device *_dev;

};


#endif /*JOINTCONTROLLER_HPP_*/
