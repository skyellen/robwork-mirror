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

#ifndef RWLIBS_USE_ROBWORK_NAMESPACE_HPP
#define RWLIBS_USE_ROBWORK_NAMESPACE_HPP

/**
   @file rwlibs/use_robwork_namespace.hpp
*/

// Declare all namespaces so that we don't get compiler errors in the using
// statements below.
namespace rwlibs { namespace algorithms {}}
namespace rwlibs { namespace devices {}}
namespace rwlibs { namespace dll {}}
namespace rwlibs { namespace drawable {}}
namespace rwlibs { namespace io {}}
namespace rwlibs { namespace lua {}}
namespace rwlibs { namespace os {}}
namespace rwlibs { namespace pathoptimization {}}
namespace rwlibs { namespace pathplanners {}}
namespace rwlibs { namespace proximitystrategies {}}
namespace rwlibs { namespace sensors {}}

namespace robwork
{
    using namespace rwlibs;

    using namespace rwlibs::algorithms;
    using namespace rwlibs::devices;
    using namespace rwlibs::dll;
    using namespace rwlibs::drawable;
    using namespace rwlibs::io;
    using namespace rwlibs::lua;
    using namespace rwlibs::os;
    using namespace rwlibs::pathoptimization;
    using namespace rwlibs::pathplanners;
    using namespace rwlibs::proximitystrategies;
    using namespace rwlibs::sensors;
}

#endif // end include guard
