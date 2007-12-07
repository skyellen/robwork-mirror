/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "State.hpp"

#include "Frame.hpp"
#include "Tree.hpp"
#include "QStateSetup.hpp"

using namespace rw::kinematics;

State::State()
{    
}

State::State(boost::shared_ptr<Tree> tree) :
    _tree_state(tree),
    _q_state(
        boost::shared_ptr<QStateSetup>(
            new QStateSetup(*tree)))
{}
