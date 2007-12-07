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

#ifndef rw_kinematics_TreeState_HPP
#define rw_kinematics_TreeState_HPP

/**
 * @file TreeState.hpp
 */

#include <boost/shared_ptr.hpp>
#include <vector>

namespace rw { namespace kinematics {

    class Frame;
    class Tree;

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
        /**
         * @brief Construct an empty TreeState
         */
        TreeState();
        
        /**
         * @brief Construct a tree state for the initial structure of \a tree.
         *
         * As a user of RobWork you should not expect to use this constructor,
         * but instead rely on the copy constructor.
         *
         * @param tree [in] The fixed part of the tree data structure.
         */
        explicit TreeState(boost::shared_ptr<Tree> tree);

        /**
         * @brief The parent frame of \a frame.
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
        const Frame* getParent(const Frame& frame) const;

        /**
         * @copydoc getParent
         */
        Frame* getParent(Frame& frame) const;

        /**
         * @brief The child frames of \a frame.
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
         * @return The children of the frame.
         */
        const std::vector<Frame*>& getChildren(const Frame& frame) const;

        /**
         * @brief Move a frame within the tree.
         *
         * The frame \a frame is detached from its parent and reattached to \a
         * parent. The frames \a frame and \a parent must both belong to the
         * same tree.
         *
         * We may want to later restrict this method so that only frames of
         * certain types can be moved.
         *
         * @param frame [in] The frame to move.
         *
         * @param parent [in] The frame to attach \a frame to.
         */
        void attachFrame(Frame& frame, Frame& parent);

    private:
        // The shared (fixed) tree structure.
        boost::shared_ptr<Tree> _tree;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
