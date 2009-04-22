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

#ifndef RW_COLLISION_COLLISIONDETECTOR_HPP
#define RW_COLLISION_COLLISIONDETECTOR_HPP

/**
 * @file CollisionDetector.hpp
 *
 * @brief Class rw::proximity::CollisionDetector
 */

#include "ProximityCommon.hpp"
#include "CollisionSetup.hpp"
#include "CollisionStrategy.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/Face.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <vector>

namespace rw {
    namespace kinematics {
        class Frame;
    }
}

namespace rw {
    namespace proximity {
    namespace sandbox {

    /** @addtogroup proximity */
    /*@{*/

    class CollisionDetector;

    //! A pointer to a CollisionDetector.
    typedef rw::common::Ptr<CollisionDetector> CollisionDetectorPtr;

    /**
     @brief The CollisionDetector implements an efficient way of checking a
     complete frame tree for collisions.

     It contain a set of pairs of frames that are not to be checked against
     each other. The collision detector does not dictate a specific detection
     strategy or algorithm, instead it relies on the CollisionStrategy interface for
     the actual collision checking between two frames.

     The CollisionDetector supports switching between multiple strategies.
     */
    class CollisionDetector
    {
    public:
        /**
         @brief Collision detector for a workcell.

         The default collision setup stored in the workcell is used.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.
         */
        CollisionDetector(rw::models::WorkCellPtr workcell,
                          CollisionStrategyPtr strategy);

        /**
         @brief Collision detector for a workcell.

         Collision checking is done for the provided collision setup alone.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.

         @param setup [in] the setup for the collision checking.
         */
        CollisionDetector(rw::models::WorkCellPtr workcell,
                          CollisionStrategyPtr strategy,
                          const CollisionSetup& setup);

        /**
         @brief Check the workcell for collisions.

         @param state [in] The state for which to check for collisions.

         @param result [out] If non-NULL, the pairs of colliding frames are
         inserted in \b result.

         @param stopAtFirstContact [in] If \b result is non-NULL and \b
         stopAtFirstContact is true, then only the first colliding pair is
         inserted in \b result. By default all colliding pairs are inserted.

         @return true if a collision is detected; false otherwise.
         */
        bool inCollision(const kinematics::State& state,
                         FramePairSet* result = 0,
                         bool stopAtFirstContact = false) const;

        /**
         @brief Set the primitive collision strategy to \b strategy.

         \b strategy must be non-NULL.

         @param strategy [in] - the primitive collision checker to use.
         */
        void setCollisionStrategy(CollisionStrategyPtr strategy);

        /**
         @brief The collision strategy of the collision checker.
         */
        CollisionStrategy& getCollisionStrategy() const
        {
            return *_strategy;
        }

        /**
         @brief The collision strategy of the collision checker.
         */
        CollisionStrategyPtr getCollisionStrategyPtr() const
        {
            return _strategy;
        }

        /**
         * @brief Returns the frame pairs which will be checked for collision
         */
        const FramePairSet& getFramePairSet() const
        {
            return _collisionPairs;
        }

        // Constructor functions.

        /**
         @brief Collision detector for a workcell and collision setup.

         The default collision setup stored in the workcell is used.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.
         */
        static CollisionDetectorPtr make(rw::models::WorkCellPtr workcell,
                                         CollisionStrategyPtr strategy);

        /**
         @brief Collision detector for a workcell.

         Collision checking is done for the provided collision setup alone.

         @param workcell [in] the workcell.

         @param strategy [in] the collision checker strategy to use.

         @param setup [in] the setup for the collision checking.
         */
        static CollisionDetectorPtr make(rw::models::WorkCellPtr workcell,
                                         CollisionStrategyPtr strategy,
                                         const CollisionSetup& setup);

        /**
         @brief Collision detector for a set of pairs of frames.

         The collision detector checks if any of the pairs of frames are in
         collision.

         \b strategy must be non-NULL.

         @param strategy [in] Collision checker for a frame pair.
         @param pairs [in] Pairs of frames.
         */
        static CollisionDetectorPtr make(CollisionStrategyPtr strategy,
                                         const FramePairSet& pairs);

        /**
         @brief Collision detector for a device.

         This collision detector assumes that all frames of the workcell
         (including the DAFs) are fixed, except for the frames that can be
         controlled by \b device.

         The collision also assumes that frame pairs that are not in the set
         of pairs stored in \b detector need not be checked.

         The collision strategy of \b detector is reused for the new collision
         detector.
         */
        static CollisionDetectorPtr
                make(const CollisionDetector& detector,
                     const rw::models::Device& device,
                     const rw::kinematics::State& state);



    private:
        CollisionStrategyPtr _strategy;
        FramePairSet _collisionPairs;

    private:
        CollisionDetector(CollisionStrategyPtr strategy,
                          const FramePairSet& pairs);

        CollisionDetector(const CollisionDetector&);
        CollisionDetector& operator=(const CollisionDetector&);

    };

/*@}*/
}
}
} // end namespaces

#endif // end include guard
