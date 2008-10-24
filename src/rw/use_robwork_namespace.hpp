/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_USE_ROBWORK_NAMESPACE_HPP
#define RW_USE_ROBWORK_NAMESPACE_HPP

/**
   @file use_robwork_namespace.hpp
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
