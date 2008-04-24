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

#ifndef rw_kinematics_Frame_HPP
#define rw_kinematics_Frame_HPP

/**
 * @file Frame.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyBase.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/ConcatVectorIterator.hpp>

#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <ostream>

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    class State;

    /**
     * @brief The type of node of forward kinematic trees.
     *
     * Types of joints are implemented as subclasses of Frame. The
     * responsibility of a joint is to implement the getTransform() method that
     * returns the transform of the frame relative to whatever parent it is
     * attached to.
     *
     * The getTransform() method takes as parameter the set of joint values
     * State for the tree. Joint values for a particular frame can be accessed
     * via State::getQ(). A frame may contain pointers to other frames so that
     * the transform of a frame may depend on the joint values for other frames
     * also.
     */
    class Frame
    {
        typedef std::vector<Frame*> ChildList;

    public:
        /**
         * @brief The transform of the frame relative to its parent.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * The exact implementation of getTransform() depends on the type of
         * frame. See for example RevoluteJoint and PrismaticJoint.
         *
         * @param state [in] Joint values for the forward kinematics tree.
         *
         * @return The transform of the frame relative to its parent.
         */
        virtual math::Transform3D<> getTransform(const State& state) const = 0;

        /**
         * @brief The number of degrees of freedom (dof) of the frame.
         *
         * The dof is the number of joint values that are used for controlling
         * the frame.
         *
         * Given a set joint values of type State, the getDOF() number of joint
         * values for the frame can be read and written with State::getQ() and
         * State::setQ().
         *
         * @return The number of degrees of freedom of the frame.
         */
        int getDOF() const { return _dof; }

        /**
           @brief DEPRECATED. Use getDOF().
        */
        int getDof() const;

        /**
         * @brief An integer ID for the frame.
         *
         * IDs are assigned to the frame upon insertion in a tree. Frames that
         * are not in a tree have an ID of -1.
         *
         * Frames present in different trees may have identical IDs.
         *
         * IDs are used for the efficient implementation of State. Normally,
         * you should not make use of frame IDs yourself.
         *
         * @return An integer ID for the frame.
         */
        int getID() const { return _id; }

        /**
         * @brief Miscellaneous properties of the frame.
         *
         * The property map of the frame is provided to let the user store
         * various settings for the frame. The settings are typically loaded
         * from setup files.
         *
         * The low-level manipulations of the property map can be cumbersome. To
         * ease these manipulations, the PropertyAccessor utility class has been
         * provided. Instances of this class are provided for a number of common
         * settings, however it is undecided if these properties are a public
         * part of robwork.
         *
         * @return The property map of the frame.
         */
        const common::PropertyMap& getPropertyMap() const { return _propertyMap; }

        /**
         * @copydoc getPropertyMap
         */
        common::PropertyMap& getPropertyMap() { return _propertyMap; }

        /**
         * @brief The name of the frame.
         *
         * @return The name of the frame.
         */
        const std::string& getName() const { return _name; }

        /**
         * @brief Destructor for the frame.
         */
        virtual ~Frame() { }

        // The parents

        /**
         * @brief The parent of the frame or NULL if the frame is a DAF.
         */
        const Frame* getParent() const { return _parent; }

        /**
         * @copydoc getParent
         */
        Frame* getParent() { return _parent; }

        /**
         * @brief Returns the parent of the frame
         *
         * If no static parent exists it look for at DAF parent. If such
         * does not exists either it returns NULL.
         *
         * @param state [in] the state to consider
         * @return the parent
         */
        Frame* getParent(const State& state);

        /**
         * @copydoc getParent(const State&)
         */
        const Frame* getParent(const State& state) const;

        /**
         * @brief The dynamically attached parent or NULL if the frame is not a
         * DAF.
         */
        const Frame* getDafParent(const State& state) const;

        /**
         * @copydoc getDafParent
         */
        Frame* getDafParent(const State& state);

        // The children.

        /**
         * @brief Forward iterator for frames.
         */
        typedef common::ConcatVectorIterator<Frame> iterator;

        /**
         * @brief Forward iterator for const frames.
         */
        typedef common::ConstConcatVectorIterator<Frame> const_iterator;

        /**
         * @brief Pair of iterators
         */
        typedef std::pair<iterator, iterator> iterator_pair;

        /**
         * @brief Pair of const iterators
         */
        typedef std::pair<const_iterator, const_iterator> const_iterator_pair;

        /**
         * @brief Iterator pair for the fixed children of the frame.
         */
        const_iterator_pair getChildren() const
        { return makeConstIteratorPair(_children); }

        /**
         * @copydoc getChildren
         */
        iterator_pair getChildren()
        { return makeIteratorPair(_children); }

        /**
         * @brief Iterator pair for all children of the frame.
         */
        const_iterator_pair getChildren(const State& state) const;

        /**
         * @brief Iterator pair for all children of the frame.
         */
        iterator_pair getChildren(const State& state);

        /**
         * @brief Iterator pair for the dynamically attached children of the
         * frame.
         */
        const_iterator_pair getDafChildren(const State& state) const;

        /**
         * @copydoc getDafChildren
         */
        iterator_pair getDafChildren(const State& state);

        // The frame values.

        /**
         * @brief An array of length getDOF() containing the joint values for
         * the frame.
         *
         * It is OK to call this method also for a frame with zero degrees of
         * freedom.
         *
         * @param state [in] The state containing the joint values.
         *
         * @return The joint values for the frame.
         */
        const double* getQ(const State& state) const;

        /**
         * @brief Assign for \b frame the getDOF() joint values of the array \b
         * vals.
         *
         * The array \b vals must be of length at least getDOF().
         *
         * @param state [inout] The state to which \b vals are written.
         *
         * @param vals [in] The joint values to assign.
         *
         * setQ() and getQ() are related as follows:
         * \code
         * frame.setQ(state, q_in);
         * const double* q_out = frame.getQ(state);
         * for (int i = 0; i < frame.getDOF(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        void setQ(State& state, const double* vals) const;

        // Dynamic frame attachments.

        /**
         * @brief Move a frame within the tree.
         *
         * The frame \b frame is detached from its parent and reattached to \b
         * parent. The frames \b frame and \b parent must both belong to the
         * same kinematics tree.
         *
         * Only frames with no static parent (see getParent()) can be moved.
         *
         * @param parent [in] The frame to attach \b frame to.
         * @param state [inout] The state to which the attachment is written.
         */
        void attachFrame(Frame& parent, State& state);

    protected:
        /**
         * @brief A frame with \b dof number of degrees of freedom.
         *
         * \b dof must be non-negative.
         *
         * The newly created frame can be added to a tree with Tree::addFrame().
         *
         * The number of degrees of freedom of the frame is constant throughout
         * the lifetime of the frame.
         *
         * @param parent [in] The parent frame. If the parent is NULL, the frame
         * will be attachable.
         *
         * @param dof [in] The number of degrees of freedom of the frame.
         *
         * @param name [in] The name of the frame.
         */
        Frame(Frame* parent, int dof, const std::string& name);

    private:
        // The number of degrees of freedom. This value remains fixed throughout
        // the life time of the frame.
        int _dof;

        // An integer ID for the frame. Assignment of ID values is the
        // responsibility of the tree in which the frame is inserted. The ID of
        // a frame may change over time.
        int _id;

        // Various attributes stored for the frame. Users and constructors of
        // frame are free to use this value as they please.
        common::PropertyMap _propertyMap;

        // The name of the frame. This is really too convenient for error
        // messages to leave out.
        std::string _name;

        Frame* _parent;
        ChildList _children;

        void addChild(Frame* child) { _children.push_back(child); }

        static iterator_pair makeIteratorPair(const ChildList& children)
        {
            return std::make_pair(
                iterator(&children, children.begin(), NULL),
                iterator(&children, children.end(), NULL));
        }

        static const_iterator_pair makeConstIteratorPair(
            const ChildList& children)
        {
            return std::make_pair(
                const_iterator(iterator(&children, children.begin(), NULL)),
                const_iterator(iterator(&children, children.end(), NULL)));
        }

        static iterator_pair makeIteratorPair(
            const ChildList& first,
            const ChildList& next)
        {
            return std::make_pair(
                iterator(&first, first.begin(), &next),
                iterator(&next, next.end(), NULL));
        }

        static const_iterator_pair makeConstIteratorPair(
            const ChildList& first,
            const ChildList& next)
        {
            return std::make_pair(
                const_iterator(iterator(&first, first.begin(), &next)),
                const_iterator(iterator(&next, next.end(), NULL)));
        }

    private:
        // The tree is responsible for the assignment of the IDs that are later
        // used in the State implementation. Tree is a friend so that IDs can
        // be modified only from there. The common advice is that the friend
        // class should be in the same header file as this class, but we break
        // that advice to allow the Tree declaration to be excluded from the
        // Doxygen documentation.
        friend class Tree;

        void setID(int id) { _id = id; }

    private:
        // Frames should not be copied.
        Frame(const Frame&);
        Frame& operator=(const Frame&);
    };

    /**
       @brief Streaming operator.
    */
    std::ostream& operator<<(std::ostream& out, const Frame& frame);

    /*@}*/
}} // end namespaces

#endif // end include guard
