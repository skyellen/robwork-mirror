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


#ifndef RW_GRAPHICS_DRAWABLENODECLONE_HPP
#define RW_GRAPHICS_DRAWABLENODECLONE_HPP

/**
 * @file DrawableNodeClone.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>

#include "DrawableNode.hpp"

namespace rw { namespace graphics {

    //! @addtogroup graphics
	// @{

	/**
	 * @brief Abstract base class for all drawable classes
	 *
	 * Classes that are able to draw them self, may inherit from this class.
	 *
	 * The drawable class use a draw mask to distinguish between different
	 * groups to draw. E.g. when taking snapshots with a simulated camera
	 * virtual objects such as the red laser vector or the lines showing
	 * the camera view angle is should not be renered. Hence objects that
	 * are virtual should be set to virtual.
	 *
	 * A call to draw enabling Physical and User1 defined objects look like:
	 * \code
	 * drawable->draw(DrawableNode::Physical | DrawableNode::User1);
	 * \endcode
	 */
    class DrawableNodeClone: public DrawableNode {
    public:

        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<DrawableNodeClone> Ptr;

        /**
         * @brief constructor
         * @param name [in] the name of the node.
         * @param drawable [in] the drawable to clone - it will not make an actual clone of data, only of the visualization.
         */
        DrawableNodeClone(const std::string& name, DrawableNode::Ptr drawable):DrawableNode(name),_drawable(drawable){
        	_highlightedState = false;
            _highlighted = _drawable->isHighlighted();
            _visibleState = false;
            _visible = _drawable->isVisible();
            _transform = _drawable->getTransform();
            _scaleState = 0;
            _scale = _drawable->getScale();
            _maskState = 0;
            _mask = _drawable->getMask();
            _alphaState = 0;
            _alpha = _drawable->getTransparency();
        }

        //! destructor
    	virtual ~DrawableNodeClone(){}

        /**
         * @brief draws the object.
         */
        void draw(const DrawableNode::RenderInfo& info = DrawableNode::RenderInfo()) const{
            saveState();
            setState();
            _drawable->draw(info);
            restoreState();
        }

        /**
         * @brief enables or disables highlighting of the drawable class
         *
         * @param b [in] a if true highlight is enabled if false disabled
         */
        virtual void setHighlighted(bool b){ _highlighted = b; }

        /**
         * @brief Returns whether the DrawableNode is highlighted
         *
         * @return true/false
         */
        virtual bool isHighlighted() const{ return _highlighted; }

        /**
         * @brief Sets the DrawType
         *
         * @param drawType [in] the DrawType to be used
         */
        virtual void setDrawType(DrawType drawType){  }

        /**
         * @brief Sets up the color alpha value.
         *
         * @param alpha [in] \f$ 0.0 \f$ corresponds to fully transparent and
         * \f$1.0\f$ to completely solid.
         */
        virtual void setTransparency(float alpha){ _alpha = alpha; }

        /**
         * @brief Gets the color alpha value.
         * @return alpha value in the interval \f$ [0.0;1.0] \f$
         */
        virtual float getTransparency(){ return _alpha; }

        /**
         * @brief Specifies the scale of the object
         * @param scale [in] the scale
         */
        virtual void setScale(float scale){ _scale = scale; }

        /**
         * @brief gets the scale of the object
         * @return scale [in] the scale
         */
        virtual float getScale() const{ return _scale; }

        /**
         * @brief enable or disable this drawable. When disabled the drawable
         * will not render anything.
         */
        virtual void setVisible(bool enable){ _visible=enable; }

        /**
         * @brief checks if this drawable is enabled
         */
        virtual bool isVisible() { return  _visible; }

        /**
         * @brief gets the transformation of the drawable object
         * @return transform of the drawable object
         */
        virtual const rw::math::Transform3D<>& getTransform() const{ return  _transform; }

        /**
         * @brief Sets the transformation of the drawable object
         * @param t3d [in] transform of drawable object
         */
        virtual void setTransform(const rw::math::Transform3D<>& t3d) { _transform = t3d; }

        /**
         * @brief the group(s) that this drawable belong to
         * @param mask [in] drawable mask
         */
        virtual void setMask(unsigned int mask){ _mask=mask; }

        /**
         * @brief Get the DrawableTypeMask for the node.
         * @return the type mask.
         */
        virtual unsigned int getMask() const { return  _mask; }

    protected:
        //! saves state of drawable
        void saveState() const {
            _highlightedState = _drawable->isHighlighted();
            _visibleState = _drawable->isVisible();
            _transformState = _drawable->getTransform();
            _alphaState = _drawable->getTransparency();
            _maskState = _drawable->getMask();
            _scaleState = _drawable->getScale();
        }
        //! sets state of drawable
        void setState() const {
            _drawable->setHighlighted(_highlighted);
            _drawable->setVisible(_visible);
            _drawable->setTransform(_transform);
            _drawable->setTransparency(_alpha);
            _drawable->setMask(_mask);
            _drawable->setScale(_scale);
        }
        //! restore the drawable state
        void restoreState() const {
            _drawable->setHighlighted(_highlightedState);
            _drawable->setVisible(_visibleState);
            _drawable->setTransform(_transformState);
            _drawable->setTransparency(_alphaState);
            _drawable->setMask(_maskState);
            _drawable->setScale(_scaleState);
        }
    private:

    	DrawableNodeClone();
        DrawableNodeClone(const DrawableNodeClone&);
        DrawableNodeClone& operator=(const DrawableNodeClone&);

    private:
        mutable DrawableNode::Ptr _drawable;
        // the drawables saved states
        mutable bool _highlightedState, _highlighted,
                     _visibleState, _visible;
        mutable rw::math::Transform3D<> _transformState, _transform;
        mutable float _scaleState, _scale;
        mutable int _maskState, _mask;
        mutable float _alphaState, _alpha;


    };

    /*@}*/
}} // end namespaces

#endif // end include guard
