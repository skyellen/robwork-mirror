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

#ifndef RWLIBS_CONTROL_JOINTCONTROLLER_HPP_
#define RWLIBS_CONTROL_JOINTCONTROLLER_HPP_

#include <rw/math/Q.hpp>

#include <rw/common/macros.hpp>
#include <rw/models/Device.hpp>

#include <rw/common/Ptr.hpp>

#include "Controller.hpp"

namespace rwlibs {
namespace control {
/**
 * @brief the joint controller interface describe how to input to a joint controller.
 * The output Force, Vel, Pos... must be available in the class implementing JointController interface
 *
 */
class JointController: public Controller
{

public:
    //! @brief smart pointer of this class
    typedef rw::common::Ptr<JointController> Ptr;

    //! control mode
    typedef enum
    {
        POSITION = 1, CNT_POSITION = 2, VELOCITY = 4, FORCE = 8, CURRENT = 16
    } ControlMode;

    /**
     * @brief destructor
     */
    virtual ~JointController()
    {
    }
    ;

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

    /**
     * @brief sets the target velocity
     * @param vals [in] in m/s
     */
    virtual void setTargetVel(const rw::math::Q& vals) = 0;

    /**
     * @brief sets the target acceleration
     * @param vals [in] in m/s^2
     */
    virtual void setTargetAcc(const rw::math::Q& vals) = 0;

    /**
     * @brief get kinematic model of device that is controlled
     */
    virtual rw::models::Device& getModel()
    {
        return *_dev;
    }

    /**
     * @brief return the current position of the controlled robot
     */
    virtual rw::math::Q getQ() = 0;

    /**
     * @brief return the current velocity
     */
    virtual rw::math::Q getQd() = 0;
protected:

    /**
     * @brief constructor
     * @param name [in] name of controller
     * @param dev [in] device model of the controlled device
     * @return
     */
    JointController(const std::string& name, rw::models::Device* dev) :
        Controller(name),
        _dev(dev)
    {
        RW_ASSERT(_dev);
    }

private:
    rw::models::Device *_dev;

};

typedef rw::common::Ptr<JointController> JointControllerPtr;
}
}
#endif /*JOINTCONTROLLER_HPP_*/
