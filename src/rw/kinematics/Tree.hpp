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

#ifndef rw_kinematics_Tree_HPP
#define rw_kinematics_Tree_HPP

/**
 * @file Tree.hpp
 */

#include <vector>
#include <boost/shared_ptr.hpp>

namespace rw { namespace kinematics {
    class Frame;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief The kinematics tree of frames.
     *
     * Tree is the low level datastructure representing a forward kinematics
     * tree.
     *
     * A frame can be present in at the most one tree at a time.
     *
     * Tree is shared and therefore should be kept fixed (const). On the other
     * hand, for some applications we might want to later change the structure
     * of the tree and also the methods that modify the tree are convenient when
     * building the tree. Among the modifying methods, we distinguish between
     * those that change the numbering of the frames and those that don't. The
     * tree has a getVersion() method that returns a number that is incremented
     * whenever the numbering of frames has globally changed in any way.
     *
     * QState is an example of a class for which Tree values are shared. QState
     * calls getVersion() to check that the tree hasn't been modified since the
     * initial version. If it has, it means that the state representation of
     * QState probably isn't valid anymore.
     *
     * Tree is not part of the public interface of RobWork.
     *
     * Trees can be copied and assigned freely, but show some constraint.
     */
    class Tree
    {
    public:
        /**
         * @brief An empty tree.
         */
        Tree();

        /**
         * @brief Destructor for the tree.
         *
         * Every frame still in the tree is destructed.
         */
        ~Tree();

        /**
         * @brief Add \a frame to the tree.
         *
         * The frame must not previously have been added to this tree or any
         * other tree.
         *
         * Ownership of the frame is taken: Every frame of the tree will be
         * destructed when the tree is destructed.
         *
         * @param frame [in] The frame to add.
         */
        void addFrame(Frame* frame);

        /**
         * @brief Adds \a frame chain structure to the tree.
         *
         * None of the frames in the chain must not previously have been added
         * to this tree or any other tree.
         *
         * Ownership of the frame is taken: Every frame of the tree will be
         * destructed when the tree is destructed.
         *
         * @param first [in] The first frame in the chain to add.
         * @param last [in] The last frame in the chain to add.
         */
        void addFrameChain(Frame* first, Frame* last);

        /**
         * @brief Let the parent of \a frame be \a parent.
         *
         * Both of the frames must previously have been added to the tree.
         *
         * setParent() is related to getParent() by
         * \code
         * tree.setParent(frame, parent);
         * &parent == tree.getParent(frame);
         * \endcode
         *
         * The frame must not already have a parent. If you want to move a frame
         * from one parent to another, you must remove the frame, add the frame,
         * and finally set the parent of the frame.
         *
         * @param frame [inout] The frame for which to set the parent.
         *
         * @param parent [inout] The parent to assign to the frame.
         */
        void setDafParent(Frame& frame, Frame& parent);

        /**
         * @brief The parent of \a frame.
         *
         * If the frame has no parent then NULL is returned.
         *
         * The frame must previously have been added to the tree.
         *
         * See also setParent().
         *
         * @param frame [in] The frame for which to retrieve the parent.
         *
         * @return The parent frame or NULL if the frame has no parent.
         */
        Frame* getDafParent(Frame& frame) const;

        /**
         * @copydoc Tree::getDafParent
         */
        const Frame* getDafParent(const Frame& frame) const;

        /**
         * @brief The children of \a frame.
         *
         * If the frame has no children, the empty list of children is returned.
         *
         * The frame must previously have been added to the tree.
         *
         * Note that we break const-correctness. We treat Tree as an
         * implementation detail upon which an iterator interface in Frame is
         * then built.
         *
         * @param frame [in] The frame for which to retrieve the children.
         *
         * @return The children of the frame.
         */
        const std::vector<Frame*>& getChildren(const Frame& frame) const;

        /**
         * @brief The number of calls to frame ID modifying methods of the tree.
         *
         * This method is provided to allow other classes to check if the frame
         * ID assignment (this includes the number of frames) of the tree has
         * changed since the tree was last used.
         *
         * @return Frame ID modification count for the tree.
         */
        int getVersion() const { return _version; }

        /**
         * @brief The greatest number of frames ever present in the tree.
         *
         * All frame IDs (see Frame::getID()) for the frames of the tree are
         * lower than this number (and greater than or equal to zero).
         *
         * @return The greatest number of frames ever present in the tree.
         */
        int getMaxCnt() const { return _frames.size(); }

        /**
         * @brief All frames of the tree.
         *
         * @return All frames of the tree.
         */
        std::vector<Frame*> getFrames() const;

        /**
         * @brief Move a frame within the tree.
         *
         * The frame \a frame is detached from its parent and reattached to \a
         * parent. The frames \a frame and \a parent must both belong to the
         * same tree.
         *
         * The method is guaranteed no to change the ID of \a frame or any other
         * frame.
         *
         * @param frame [in] The frame to move.
         *
         * @param parent [in] The frame to attach \a frame to.
         */
        void attachFrame(Frame& frame, Frame& parent);

    private:
        void incVersion() { ++_version; }
        void addChild(const Frame& parent, Frame& frame);
        void detachFrame(const Frame& frame);
        int allocateFrameID();
        bool isInTree(const Frame& frame) const;

    private:
        typedef std::vector<Frame*> FrameList;

        std::vector<boost::shared_ptr<Frame> > _frames;

        FrameList _parents;
        std::vector<FrameList> _children;
        std::vector<int> _available_ids;
        int _version;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
