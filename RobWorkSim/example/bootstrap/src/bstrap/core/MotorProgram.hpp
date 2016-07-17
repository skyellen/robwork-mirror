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

#ifndef MOTORPROGRAM_HPP_
#define MOTORPROGRAM_HPP_

#include <rw/common/Ptr.hpp>
#include <QThread>
#include <QWidget>

namespace rw { namespace common { class PropertyMap; } }

class BrainState;

class MotorProgram: public QThread {
    Q_OBJECT
public:
    typedef rw::common::Ptr<MotorProgram> Ptr;
    MotorProgram(const std::string& name, QWidget *parent=NULL);

    void execute(rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate);

    //! @brief executes the command
    void run();

    void stop();

    bool isStopped();

    // called to update the motor program of with the current state
    virtual void update(const BrainState& bstate){};

protected:

    virtual void setParameters(rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate) = 0;
    virtual void executionloop() = 0;

private:
    std::string _name;
    bool _stop, _stopped, _finished;

};

#endif
