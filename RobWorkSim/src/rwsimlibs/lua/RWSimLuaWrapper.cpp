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
