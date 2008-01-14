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

#ifndef rw_kinematics_Kinematics_HPP
#define rw_kinematics_Kinematics_HPP

/**
 * @file Kinematics.hpp
 */

#include <rw/math/Transform3D.hpp>
#include "Frame.hpp"

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Kinematics provides various utility functions to help use
     * the classes in the kinematics namespace.
     */
    class Kinematics {
    public:
        /**
         * @brief The transform of \b frame in relative to the world frame.
         *
         * If to=NULL the method returns a \f$4\times 4\f$ identify matrix
         *
         * @param to [in] The transform for which to find the world frame.
         *
         * @param state [in] The state of the kinematics tree.
         *
         * @return The transform of the frame relative to the world frame.
         */
        static math::Transform3D<> WorldTframe(const Frame* to, const State& state);

        /**
         * @brief The transform of frame \b to relative to frame \b from.
         *
         * FrameTframe() is related to WorldTframe() as follows:
         \code
         frameTframe(from, to, state) ==
         inverse(worldTframe(from, state)) *
         worldTframe(to, state);
         \endcode
         *
         * @param from [in] The start frame.
         *
         * @param to [in] The end frame.
         *
         * @param state [in] The state of the kinematics tree.
         *
         * @return The transform from the start frame to the end frame.
         */
        static math::Transform3D<> FrameTframe(
            const Frame* from, const Frame* to, const State& state);

        /** @brief All frames reachable from \b root for a tree structure of \b
         * state.
         *
         * This is a tremendously useful utility. An alternative would be to have an
         * iterator interface for trees respresented by work cell states.
         *
         * We give no guarantee on the ordering of the frames.
         *
         * @param root [in] The root node from where the frame search is started.
         *
         * @param state [in] The structure of the tree.
         *
         * @return All reachable frames.
         */
        static std::vector<Frame*> FindAllFrames(Frame* root, const State& state);

        /**
           @brief The chain of frames connecting \b child to \b parent.

           \b child is included in the chain, but \b parent is not included. If
           \b parent is NULL then the entire path from \b child to the world
           frame is returned. If \b child as well as \b parent is NULL then the
           empty chain is gracefully returned.

           The \b state gives the connectedness of the tree.

           If \b parent is not on the chain from \b child towards the root, then
           an exception is thrown.
        */
        static std::vector<Frame*> ChildToParentChain(
            Frame* child, Frame* parent, const State& state);

        /**
           @brief Like ChildToParentChain() except that the frames are returned
           in the reverse order.
        */
        static std::vector<Frame*> ReverseChildToParentChain(
            Frame* child, Frame* parent, const State& state);

        /**
           @brief The chain of frames connecting \b parent to \b child.

           \b parent is included in the list, but \b child is excluded. If \b
           parent as well as \b child is NULL then the empty chain is returned.
           Otherwise \b parent is included even if \b parent is NULL.
         */
        static std::vector<Frame*> ParentToChildChain(
            Frame* parent, Frame* child, const State& state);

        /**
         * @brief A map linking frame names to frames.
         */
        typedef std::map<std::string, kinematics::Frame*> FrameMap;

        /**
         * @brief A map linking frame names to frames.
         *
         * The map contains an entry for every frame below \b root in the tree with
         * structure described by \b state.
         *
         * @param root [in] Root of the kinematics tree to search.
         *
         * @param state [in] The kinematics tree structure.
         */
        static FrameMap BuildFrameMap(
            kinematics::Frame& root, const kinematics::State& state);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
