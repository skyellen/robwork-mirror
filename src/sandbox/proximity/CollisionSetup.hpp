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

#ifndef RW_COLLISION_COLLISIONSETUP_HPP
#define RW_COLLISION_COLLISIONSETUP_HPP

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
        CollisionSetup();

        /**
           @brief Constructs CollisionSetup with list of exclusions

           @param exclude [in] pairs to be excluded
         */
        explicit CollisionSetup(const ProximityPairList& exclude);

        /**
           @brief CollisionSetup for a list of pairs to exclude and a sequence
           of volatile frames.

           @param exclude [in] pairs to be excluded

           @param volatileFrames [in] names of frames to treat as volatile.

           @param excludeStaticPairs [in] if true exclude statically related pairs.
         */
        CollisionSetup(const ProximityPairList& exclude,
                       const std::set<std::string>& volatileFrames,
                       bool excludeStaticPairs);

        /**
         * @brief Returns the exclude list
         * @return the exclude list
         */
        const ProximityPairList& getExcludeList() const { return _exclude; }

        /**
           @brief True iff all statically related pairs of frames should be
           excluded.

           Note that this will exclude also statically related pairs of frames
           for which one or both of the pairs are volatile.
        */
        bool excludeStaticPairs() const { return _excludeStaticPairs; }

        /**
         * @brief Combine setup of this and setup of \b b into this collision setup.
         */
        void merge(const CollisionSetup& b);

        /**
         * @brief Combine setup \b a and setup \b b into a single collision setup.
         */
        static CollisionSetup merge(const CollisionSetup& a, const CollisionSetup& b);

    private:
        ProximityPairList _exclude;
        bool _excludeStaticPairs;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
