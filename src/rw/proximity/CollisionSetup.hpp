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

#ifndef rw_collision_CollisionSetup_HPP
#define rw_collision_CollisionSetup_HPP

/**
 * @file CollisionSetup.hpp
 */

#include "ProximityCommon.hpp"

#include <string>
#include <vector>

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief Setup for the collision checker
     *
     * The CollisionSetup contains information about
     * which frames, not be checked against each other
     */
    class CollisionSetup
    {
    public:
        /**
         * @brief Default constructor for when no excludes are described
         */
        CollisionSetup() {}

        /**
         * @brief Constructs CollisionSetup with list of exclusions
         * @param exclude [in] pairs to be excluded
         */
        explicit CollisionSetup(const ProximityPairList& exclude) :
            exclude_(exclude)
        {}

        /**
         * @brief Returns the exclude list
         * @return the exclude list
         */
        const ProximityPairList& getExcludeList() const
        { return exclude_; }

        /**
         * @brief Combine setup of this and setup of \a b into this collision setup.
         */
        void merge(const CollisionSetup& b);

        /**
         * @brief Combine setup \a a and setup \a b into a single collision setup.
         */
        static CollisionSetup Merge(
            const CollisionSetup& a, const CollisionSetup& b);

    private:
        ProximityPairList exclude_;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
