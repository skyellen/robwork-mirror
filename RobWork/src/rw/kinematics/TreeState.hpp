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


#ifndef RW_KINEMATICS_TREESTATE_HPP
#define RW_KINEMATICS_TREESTATE_HPP

/**
 * @file TreeState.hpp
 */

#include <boost/shared_ptr.hpp>
#include <vector>

namespace rw { namespace kinematics {

    class Frame;
    class StateSetup;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief The tree structure state of a work cell.
     *
     * The tree structure state gives access to the parent and child frames of a
     * frame.
     *
     * Currently modification of the tree structure is not supported. (This
     * implementation simply forwards to the non-public Tree data structure.)
     *
     * Tree structure states can be copied and assigned freely.
     */
    class TreeState
    {
    public:
        //! @brief vector of frames
        typedef std::vector<Frame*> FrameList;

        /**
         * @brief Construct an empty TreeState
         */
        TreeState();

        /**
         * @brief Construct an empty TreeState
         */
        explicit TreeState(boost::shared_ptr<StateSetup> setup);

        explicit TreeState(const TreeState &);

        /**
         * @brief destructor
         */
        virtual ~TreeState();

        /**
         * @brief The parent frame of \b frame.
         *
         * If the frame has no parent, then NULL is returned.
         *
         * (We should query the modifiable part of the tree here, but that is
         * not implemented yet.)
         *
         * @param frame [in] The frame for which to retrieve the parent.
         *
         * @return The parent of the frame or NULL if the frame has no parent.
         */
        const Frame* getParent(const Frame* frame) const;

        /**
         * @copydoc getParent
         */
        Frame* getParent(Frame* frame) const;

        /**
         * @brief The child frames of \b frame.
         *
         * (We should query the modifiable part of the tree here, but that is
         * not implemented yet.)
         *
         * Note that we break const-correctness. We treat TreeState as an
         * implementation detail upon which an iterator interface in Frame is
         * then built.
         *
         * @param frame [in] The frame for which to retrieve the children.
         *
         * @return The children of the frame if any children exist, else NULL.
         */
        const FrameList& getChildren(const Frame* frame) const;


        /**
         * @brief Move a frame within the tree.
         *
         * The frame \b frame is detached from its parent and reattached to \b
         * parent. The frames \b frame and \b parent must both belong to the
         * same tree.
         *
         * We may want to later restrict this method so that only frames of
         * certain types can be moved.
         *
         * @param frame [in] The frame to move.
         * @param parent [in] The frame to attach \b frame to.
         */
        void attachFrame(Frame* frame, Frame* parent);

        /**
         * @brief gets the StateSetup used to create the TreeState
         * @return the StateSetup
         */
        boost::shared_ptr<StateSetup> getStateSetup() const;

    private:
		
        boost::shared_ptr<StateSetup> _setup;

        // map descring parent to child relationships
        // size == <nr of Frames>
        std::vector< int > _parentIdxToChildList;

        // a list of all child-arrays
        // size == <nr of DAF parents>
        std::vector< FrameList > _childLists;

        // map describing child to parent relationships of DAFs
        // size == <nr of DAFs>
        std::vector<int> _dafIdxToParentIdx;
		

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
