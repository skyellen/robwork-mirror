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


#ifndef RW_USE_ROBWORK_NAMESPACE_HPP
#define RW_USE_ROBWORK_NAMESPACE_HPP

/**
   @file rw/use_robwork_namespace.hpp
*/

// Declare all namespaces so that we don't get compiler errors in the using
// statements below.

namespace rw { namespace proximity {}}
namespace rw { namespace common {}}
namespace rw { namespace control {}}
namespace rw { namespace geometry {}}
namespace rw { namespace interpolator {}}
namespace rw { namespace trajectory {}}
namespace rw { namespace invkin {}}
namespace rw { namespace kinematics {}}
namespace rw { namespace math {}}
namespace rw { namespace task {}}
namespace rw { namespace models {}}
namespace rw { namespace pathplanning {}}
namespace rw { namespace sensor {}}
namespace rw { namespace loaders {}}

namespace robwork
{
    using namespace rw;

    using namespace rw::proximity;
    using namespace rw::common;
    using namespace rw::control;
    using namespace rw::geometry;
    using namespace rw::interpolator;
    using namespace rw::trajectory;
    using namespace rw::invkin;
    using namespace rw::kinematics;
    using namespace rw::math;
    using namespace rw::task;
    using namespace rw::models;
    using namespace rw::pathplanning;
    using namespace rw::sensor;
    using namespace rw::loaders;
}

#endif // end include guard
