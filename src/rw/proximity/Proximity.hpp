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

#ifndef rw_collision_Proximity_HPP_
#define rw_collision_Proximity_HPP_

#include "ProximityCommon.hpp"
#include "CollisionSetup.hpp"
#include "CollisionStrategy.hpp"
#include <rw/models/WorkCell.hpp>

namespace rw { namespace proximity {

	/** @addtogroup proximity */
	/*@{*/

    /**
       @brief Utility functions for the rw::proximity module.
    */
    class Proximity {
    public:
        /**
           @brief
        */
        static
        FramePairSet makeFramePairSet(
            const rw::models::WorkCell& workcell,
            CollisionStrategy& strategy,
            const CollisionSetup& setup);

        /**
           @brief
        */
        static
        FramePairSet makeFramePairSet(
            const rw::models::WorkCell& workcell,
            CollisionStrategy& strategy);

    private:
        Proximity();
        Proximity(const Proximity&);
        Proximity& operator=(const Proximity&);
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
