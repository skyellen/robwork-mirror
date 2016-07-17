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

#ifndef BRAIN_HPP_
#define BRAIN_HPP_

#include <QThread>
#include <QWidget>

#include "Schema.hpp"
#include "ActionTree.hpp"
#include "ActionGoal.hpp"
#include "BrainState.hpp"
#include "Memory.hpp"

//namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
//namespace rwsim { namespace simulator { class DynamicSimulator; } }
//namespace rwsim { namespace simulator { class ThreadSimulator; } }

class Abstraction;

/**
 * @brief this class handles the execution of lua commands such that they
 * are executed in a seperate thread.
 */
class Brain: public QThread
{
    Q_OBJECT
public:
    typedef rw::common::Ptr<Brain> Ptr;

    Brain(const std::string& name,
           //rw::common::Ptr<rwsim::simulator::ThreadSimulator> sim,
		   //rw::common::Ptr<rwsim::simulator::DynamicSimulator> dsim,
		   //rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc,
           QWidget *parent=NULL) :
            QThread(parent), _name(name), _stopped(true)
			//_dsim(dsim), _sim(sim), _dwc(dwc)
    {
    }

    //! @brief executes the command
    void run();


    // getActionHistory();
    // getStateHistory();


    /**
     * @brief stops all actions and save the complete state.
     */
    void stop(){ _stop=true; }

    bool isStopped(){ return _stopped; }

    std::vector<Schema*> getSchemas();

    ActionTree::Ptr getActionTree();

    std::vector<ActionGoal::Ptr> getActionGoals();

    void add(rw::common::Ptr<Abstraction> abstraciton){ _abstractions.push_back(abstraciton); }

protected:
    BrainState computeSensorState();

private:
    std::string _name;
    bool _stop, _stopped;
    //rw::common::Ptr<rwsim::simulator::DynamicSimulator> _dsim;
    //rw::common::Ptr<rwsim::simulator::ThreadSimulator> _sim;
    //rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::kinematics::State _rwstate;
    std::vector<rw::common::Ptr<Abstraction> > _abstractions;
    Memory _memory;
    std::vector<Schema*> _schemas;

};


#endif /* LUAEDITORWINDOW_HPP_ */
