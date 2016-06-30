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

#ifndef RWLIBS_CONTROL_CONTROLLER_HPP_
#define RWLIBS_CONTROL_CONTROLLER_HPP_

#include <string>

#include <rw/common/Ptr.hpp>

namespace rwlibs {
namespace control {



/**
 * @brief interface that defines functionality for control of devices and actuators
 */
class Controller
{

public:
    //! smart poiner definition for controller
    typedef rw::common::Ptr<Controller> Ptr;

    //! @brief destructor
    virtual ~Controller(){};

    /**
     * @brief get the unique name of this controller
     * @return name of the controller.
     */
    const std::string& getName() const { return _name; }

    /**
     * @brief set the name of the controller
     * @param name [in] the name
     */
    void setName(const std::string& name) { _name = name; }

protected:
    /**
     * @brief constructor
     * @param name [in] name of controller
     */
    Controller(const std::string& name):
        _name(name)
    {
    }

private:
    Controller(){};
    std::string _name;

};

}
}

#endif /*CONTROLLER_HPP_*/
