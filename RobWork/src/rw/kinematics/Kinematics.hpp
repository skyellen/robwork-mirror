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


#ifndef RW_KINEMATICS_KINEMATICS_HPP
#define RW_KINEMATICS_KINEMATICS_HPP

/**
 * @file kinematics/Kinematics.hpp
 */

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Transform3D.hpp>
#include <map>

namespace rw { namespace kinematics {
    class MovableFrame;

    /** @addtogroup kinematics */
    /*@{*/

    /**
       @brief Utility functions for the rw::kinematics module.
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
        static math::Transform3D<> worldTframe(const Frame* to, const State& state);


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
        static math::Transform3D<> frameTframe(
            const Frame* from, const Frame* to, const State& state);


        /** @brief All frames reachable from \b root for a tree structure of \b
         * state.
         *
         * This is a tremendously useful utility. An alternative would be to have an
         * iterator interface for trees represented by work cell states.
         *
         * We give no guarantee on the ordering of the frames.
         *
         * @param root [in] The root node from where the frame search is started.
         *
         * @param state [in] The structure of the tree.
         *
         * @return All reachable frames.
         */
        static std::vector<Frame*> findAllFrames(Frame* root, const State& state);

        /** @brief All frames reachable from \b root for a tree structure.
         *
         * This is a tremendously useful utility. An alternative would be to have an
         * iterator interface for trees represented by work cell states.
         *
         * We give no guarantee on the ordering of the frames.
         *
         * DAF are not included.
         *
         * @param root [in] The root node from where the frame search is started.
         *
         * @param state [in] The structure of the tree.
         *
         * @return All reachable frames.
         */
        static std::vector<Frame*> findAllFrames(Frame* root);

        /**
           @brief Find the world frame of the workcell by traversing the path
           from \b frame to the root of the tree.

           The state \b state is needed to retrieve the parent frames, but the
           world frame returned is the same for any (valid) state.
        */
        static Frame* worldFrame(Frame* frame, const State& state);

        /**
           @brief Find the world frame of the workcell by traversing the path
           from \b frame to the root of the tree.

           The state \b state is needed to retrieve the parent frames, but the
           world frame returned is the same for any (valid) state.
        */
        static const Frame* worldFrame(const Frame* frame, const State& state);

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
        static std::vector<Frame*> childToParentChain(Frame* child,
                                                      Frame* parent,
                                                      const State& state);

        /**
           @brief Like ChildToParentChain() except that the frames are returned
           in the reverse order.
        */
        static std::vector<Frame*> reverseChildToParentChain(Frame* child,
                                                             Frame* parent,
                                                             const State& state);

        /**
           @brief The chain of frames connecting \b parent to \b child.

           \b parent is included in the list, but \b child is excluded. If \b
           parent as well as \b child is NULL then the empty chain is returned.
           Otherwise \b parent is included even if \b parent is NULL.
         */
        static std::vector<Frame*> parentToChildChain(Frame* parent,
                                                      Frame* child,
                                                      const State& state);

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
        static std::map<std::string, kinematics::Frame*> buildFrameMap(kinematics::Frame* root,
                                                                       const kinematics::State& state);


#ifdef RW_USE_DEPRECATED
        /**
           @brief Find the world frame of the workcell by traversing the path
           from \b frame to the root of the tree.

           The state \b state is needed to retrieve the parent frames, but the
           world frame returned is the same for any (valid) state.
        */
        static Frame& worldFrame(Frame& frame, const State& state);

        /**
           @brief Find the world frame of the workcell by traversing the path
           from \b frame to the root of the tree.

           The state \b state is needed to retrieve the parent frames, but the
           world frame returned is the same for any (valid) state.
        */
        static const Frame& worldFrame(const Frame& frame, const State& state);

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
        static FrameMap buildFrameMap(kinematics::Frame& root,
                                      const kinematics::State& state);


        /**
           @brief True if \b frame is a DAF and false otherwise.
        */
        static bool isDAF(const Frame& frame);

        static bool isFixedFrame(const Frame& frame);

        /**
           @brief Grip \b item with \b gripper thereby modifying \b state.

           \b item must be a DAF and of type MovableFrame.

           An exception is thrown if \b item is not of this type.
        */
		static void gripFrame(State& state, Frame& item, Frame& gripper);
        /**
           @brief Like gripFrame(), except the state is not modified but updated
           and returned.
         */
        static State grippedFrame(const State& state, Frame& item, Frame& gripper);

        /**
           @brief Grip \b item with \b gripper thereby modifying \b state.

           \b item must be a DAF.

           An exception is thrown if \b item is not a DAF.

           See also gripFrame().
        */
        static void gripMovableFrame(State& state, MovableFrame& item, Frame& gripper);

        /**
           @brief Like gripMovableFrame(), except the state is not modified but
           updated and returned.

           See also grippedFrame().
        */
        static State grippedMovableFrame(
            const State& state, MovableFrame& item, Frame& gripper);


#endif

        /**
           @brief True if \b frame is a DAF and false otherwise.
        */
        static bool isDAF(const Frame* frame);

        static bool isFixedFrame(const Frame* frame);

		static void gripFrame(Frame* item, Frame* gripper, State& state);

        /**
           @brief Grip \b item with \b gripper thereby modifying \b state.

           \b item must be a DAF.

           An exception is thrown if \b item is not a DAF.

           See also gripFrame().
        */
        static void gripFrame(MovableFrame* item, Frame* gripper, State& state);

		static std::vector<FrameList> getStaticFrameGroups(Frame* root, const State& state);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
