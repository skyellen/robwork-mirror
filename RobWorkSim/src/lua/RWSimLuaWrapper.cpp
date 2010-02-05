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

#include "RWSimLuaWrapper.hpp"
#include "RWSimLua.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/EAA.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Models.hpp>
#include <rw/common/StringUtil.hpp>

#include <rw/pathplanning/QIKSampler.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>

#include <iostream>
using namespace std;

#include <sstream>
#include <algorithm>

#define NS dynamics::lua::internal
using namespace NS;
