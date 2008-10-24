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

#ifndef RW_COLLISION_PROXIMITY_HPP
#define RW_COLLISION_PROXIMITY_HPP

#include "ProximityCommon.hpp"
#include "CollisionSetup.hpp"
#include "CollisionStrategy.hpp"
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace proximity {

	/** @addtogroup proximity */
	/*@{*/

    /**
       @brief Utility functions for the rw::proximity module.
    */
    class Proximity {
    public:
        /**
           @brief The full set of pairs of frames for which to perform collision
           checking when given a workcell \b workcell and a collision setup \b
           setup for the workcell. The collision strategy \b strategy is used to
           verify if a frame has a model of if it can be safely excluded (unless
           other has been specified in \b setup).
        */
        static
        FramePairSet makeFramePairSet(
            const rw::models::WorkCell& workcell,
            CollisionStrategy& strategy,
            const CollisionSetup& setup);

        /**
           @brief Like makeFramePairSet(\b workcell, \b setup, \b setup) where
           \b setup is the default collision setup registered for the workcell
           (or \b setup is the empty collision setup if no collision setup has
           been specified).
        */
        static
        FramePairSet makeFramePairSet(
            const rw::models::WorkCell& workcell,
            CollisionStrategy& strategy);

        /**
           @brief Assuming that \b device is the only active device, and that
           all other frames are fixed including DAF attachments, return the
           smallest set of pairs of frames that can be deduced to be necessary
           for collision checking.

           The function assumes that DAFs have been attached according to \b
           state.

           Unlike other versions of the makeFramePairSet() function, the
           function *does not* know about the collision setup of the \b workcell
           and *does not* care about DAFs. You may therefore want to take the
           intersection between the set returned here, and the maximum set of
           frames to include in collision checking that has been returned by
           another makeFramePairSet() function.
        */
        static
        FramePairSet
        makeFramePairSet(
            const rw::models::Device& device,
            const rw::kinematics::State& state);

        /**
           @brief Pair of (staticSet, dynamicSet) where \b staticSet are the
           pairs of frames to use for collision checking \b controlledDevices
           against the static part of the workcell and \b dynamicSet are the
           pairs of frames to use for dynamically checking \b controlledDevices
           against movement in \b obstacleDevices.

           The construction of (staticSet, dynamicSet) assumes that the state of
           the rest of the workcell is otherwise given by \b state.

           @param workcellSet [in] The standard collision setup for the workcell.
           @param obstacleDevices [in] The set of devices serving as dynamic obstacles.
           @param controlledDevices [in] The set of devices for which planning is done.

           @param state [in] The fixed state relative to which \b
           obstacleDevices and \b controlledDevices move.

           @return (staticSet, dynamicSet) where \b staticSet contain \b
           workcellSet with all pairs removed that reference frames affected by
           \b obstacleDevices, and \b dynamicSet contain all pairs of frames of
           \b workcellSet that (for the same pair) refer to both the frames
           affected by \b obstacleDevices and the frames affected by \b
           controlledDevices.
        */
        static
        std::pair<FramePairSet, FramePairSet>
        makeStaticDynamicFramePairSet(
            const FramePairSet& workcellSet,
            const std::vector<rw::models::DevicePtr>& obstacleDevices,
            const std::vector<rw::models::DevicePtr>& controlledDevices,
            const rw::kinematics::State& state);

        /**
           @brief Write to \b b the intersection of \b a and \b b.

           This is equivalent to erasing from \b b all elements of \b b that are
           not elements of \b a.
        */
        static
        void intersect(const FramePairSet& a, FramePairSet& b);

        /**
           @brief Write to \b a all elements of \b a that are also elements of
           \b b.
        */
        static
        void subtract(FramePairSet& a, const FramePairSet& b);

        /**
           @brief Write to \b b the union of the sets \b a and \b b.
        */
        static
        void frameSetUnion(const FrameSet& a, FrameSet& b);

        /**
           @brief The collision setup of the workcell.

           If no collision setup is stored in the workcell, then the empty
           collision setup is returned.

           @param workcell [in] Workcell containing a collision setup.
        */
        static
        CollisionSetup getCollisionSetup(const rw::models::WorkCell& workcell);

        static
        FramePairList getExcludePairList(const rw::models::WorkCell& workcell,
                                         const CollisionSetup& setup);
    private:
        Proximity();
        Proximity(const Proximity&);
        Proximity& operator=(const Proximity&);
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
