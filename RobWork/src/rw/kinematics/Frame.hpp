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


#ifndef RW_KINEMATICS_FRAME_HPP
#define RW_KINEMATICS_FRAME_HPP

/**
 * @file Frame.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/ConcatVectorIterator.hpp>
#include <vector>
#include <set>
#include "StateData.hpp"

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
    class Frame : public StateData
    {
        typedef std::vector<Frame*> ChildList;

    public:

		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<Frame> Ptr;

        /**
         * @brief Destructor for the frame.
         */
        virtual ~Frame() { }


        /**
         * @brief Post-multiply the transform of the frame to the parent transform.
         *
         * The transform is calculated for the joint values of \b state.
         *
         * The exact implementation of getTransform() depends on the type of
         * frame. See for example RevoluteJoint and PrismaticJoint.
         *
         * @param parent [in] The world transform of the parent frame.
         * @param state [in] Joint values for the forward kinematics tree.
         * @param result [in] The transform of the frame in the world frame.
         */
        void multiplyTransform(const math::Transform3D<>& parent,
                               const State& state,
                               math::Transform3D<>& result) const {
            doMultiplyTransform(parent, state, result);
        }

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
        math::Transform3D<> getTransform(const State& state) const {
            return doGetTransform(state);
        }

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
         * part of RobWork.
         *
         * @return The property map of the frame.
         */
        const common::PropertyMap& getPropertyMap() const { return _propertyMap; }

        /**
         * @copydoc getPropertyMap
         */
        common::PropertyMap& getPropertyMap() { return _propertyMap; }


        /**
         * @brief The number of degrees of freedom (dof) of the frame.
         *
         * The dof is the number of joint values that are used for controlling
         * the frame.
         *
         * Given a set joint values of type State, the getDof() number of joint
         * values for the frame can be read and written with State::getQ() and
         * State::setQ().
         *
         * @return The number of degrees of freedom of the frame.
         */
        int getDOF() const { return size(); }


        // The parents

        /**
         * @brief The parent of the frame or NULL if the frame is a DAF.
         */
        const Frame* getParent() const { return _parent; }

        /**
         * @brief The parent of the frame or NULL if the frame is a DAF.
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
        void attachTo(Frame* parent, State& state);

        /**
         * @brief Test if this frame is a Dynamically Attachable Frame
         * @return true if this frame is a DAF, false otherwise
         */
        bool isDAF();


        rw::math::Transform3D<> wTf(const rw::kinematics::State& state) const;
        rw::math::Transform3D<> fTf(Frame* to, const rw::kinematics::State& state) const;

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
         * @param dof [in] The number of degrees of freedom of the frame.
         *
         * @param name [in] The name of the frame.
         */
        Frame(int dof, const std::string& name);

    protected:
        /**
         * @brief Subclass implementation of the getTransform() method.
         */
        virtual void doMultiplyTransform(const math::Transform3D<>& parent,
                                         const State& state,
                                         math::Transform3D<>& result) const = 0;

        /**
         * brief Subclass implementation of the multiplyTransform() method
         */
        virtual math::Transform3D<> doGetTransform(const State& state) const = 0;

    private:
        friend class StateStructure;

        void setParent(Frame *frame){
            _parent = frame;
        }

        void removeChild(Frame *frame){
            for (ChildList::iterator it = _children.begin(); it != _children.end(); ++it)
                if ((*it) == frame) {
                    _children.erase(it);
                    return;
                }
        }

    private:
        // Various attributes stored for the frame. Users and constructors of
        // frame are free to use this value as they please.
        common::PropertyMap _propertyMap;

        // static connected parent and children
        Frame* _parent;
        ChildList _children;

        void addChild(Frame* child) {
        	/*BOOST_FOREACH(Frame* cchild, _children){
        		if(cchild==child)
        			RW_THROW("The frame: \"" << child->getName() << "\" is allready child of \"" << this->getName() << "\"");
        	}
			*/
            _children.push_back(child);
        }

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
			if (first.size() == 0) 
				return std::make_pair(
					const_iterator(iterator(&next, next.begin(), &next)),
					const_iterator(iterator(&next, next.end(), NULL)));
			else
				return std::make_pair(
					const_iterator(iterator(&first, first.begin(), &next)),
					const_iterator(iterator(&next, next.end(), NULL)));
        }

    private:
        // Frames should not be copied.
        Frame(const Frame&);
        Frame& operator=(const Frame&);
    };

    /** 
       @brief A pair of frames
     */
    typedef std::pair<Frame*, Frame*> FramePair;

    /** 
       @brief A pair of constant frames
     */
    typedef std::pair<const Frame*, const Frame*> ConstFramePair;

	/**
	   @brief A set of frames.
	*/
	typedef std::set<kinematics::Frame*> FrameSet;

	/**
	   @brief A list of frames
	*/
	typedef std::vector<kinematics::Frame*> FrameList;

	/**
	   @brief A set of frame pairs.
	*/
	typedef std::set<kinematics::FramePair> FramePairSet;

	/**
	 * @brief A list of frame pairs
	 */
	typedef std::vector<kinematics::FramePair> FramePairList;

    /**
       @brief Streaming operator.
    */
    std::ostream& operator<<(std::ostream& out, const Frame& frame);

    /*@}*/
}} // end namespaces

#endif // end include guard
