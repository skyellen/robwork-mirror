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


#ifndef RWLIBS_OPENGL_DRAWABLE_HPP
#define RWLIBS_OPENGL_DRAWABLE_HPP

/**
 * @file Drawable.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>
#include <rwlibs/os/rwgl.hpp>
#include <rw/graphics/DrawableNode.hpp>

namespace rw { namespace graphics { class Render; } }

namespace rwlibs { namespace opengl {

    //! @addtogroup opengl
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
	 * drawable->draw(Drawable::Physical | Drawable::User1);
	 * \endcode
	 */
    class Drawable: public rw::graphics::DrawableNode {
    public:

        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<Drawable> Ptr;

    	/**
    	 * @brief draw mask is used to filter which drawables to exclude from rendering.
    	 *
    	typedef enum{Physical=1, //! A physical object in the scene
    				 Virtual=2,  //! A virtual object, e.g. lines showing camera view angle
    				 DrawableObject=4,//! An object that is "just" a drawable
    				 CollisionObject=8,  //! An object that is also a CollisionObject
    				 User1=1024, //! User defined group 1...
    		    	 User2=2048,//!< User2
    		    	 User3=4096,//!< User3
    		    	 User4=8096, //!< User4
    		    	 ALL=0xFFFFFFFF
    	} DrawableTypeMask;
    	 */

        /**
         * @brief Constructer for Drawable
         *
         * @param name [in] Name of drawable
         * @param dmask [in] Type of the Drawable. Default value is Physical
         */
        Drawable(
            const std::string& name,
            unsigned int dmask = Physical);

        /**
         * @brief Constructer for Drawable
         *
         * @param render [in] Render to be used by the drawable
		 * @param name [in] Name of drawable
         * @param dmask [in] Type of the Drawable. Default value is Physical
         */
        Drawable(
            rw::common::Ptr<rw::graphics::Render> render,
            const std::string& name = "",
			unsigned int dmask = DrawableObject);

        /**
         * @brief Virtual destructor
         */
        virtual ~Drawable();

        /**
         * @brief draws the object.
         */
        virtual void draw(const rw::graphics::DrawableNode::RenderInfo& info = RenderInfo()) const;

        /**
         * @brief enables or disables highlighting of the drawable class
         *
         * @param b [in] a if true highlight is enabled if false disabled
         */
        void setHighlighted(bool b);

        /**
         * @brief Returns whether the Drawable is highlighted
         *
         * @return true/false
         */
        bool isHighlighted() const;

        /**
         * @brief Sets the DrawType
         *
         * @param drawType [in] the DrawType to be used
         */
        void setDrawType(rw::graphics::DrawableNode::DrawType drawType);

        /**
         * @brief Sets up the color alpha value.
         *
         * @param alpha [in] \f$ 0.0 \f$ corresponds to fully transparent and
         * \f$1.0\f$ to completely solid.
         */
        void setTransparency(float alpha);

        /**
         * @brief Gets the color alpha value.
         * @return alpha value in the interval \f$ [0.0;1.0] \f$
         */
        float getTransparency();

        /**
         * @brief Specifies the scale of the object
         * @param scale [in] the scale
         */
        void setScale(float scale);

        /**
         * @brief gets the scale of the object
         * @return scale [in] the scale
         */
        float getScale() const;

        /**
         * @brief enable or disable this drawable. When disabled the drawable
         * will not render anything.
         */
        void setVisible(bool enable) { _enable = enable; }

        /**
         * @brief checks if this drawable is enabled
         */
        bool isVisible() { return _enable; }

        /**
         * @brief gets the transformation of the drawable object
         * @return transform of the drawable object
         */
        const rw::math::Transform3D<>& getTransform() const;

        /**
         * @brief Sets the transformation of the drawable object
         * @param t3d [in] transform of drawable object
         */
        void setTransform(const rw::math::Transform3D<>& t3d);

        /**
         * @brief the group(s) that this drawable belong to
         * @param mask [in] drawable mask
         */
        void setMask(unsigned int mask){ _dmask = mask; }

        /**
         * @brief Get the rendering groups that this drawable belongs to.
         * @return the drawable type mask (see DrawableNode::DrawableTypeMask).
         */
        unsigned int getMask() const { return _dmask;}

        /**
         * @brief Get this drawables Render object
         */
        std::vector<rw::common::Ptr<rw::graphics::Render> > getRenders() const{
        	return _renders;
        }

        /**
         * @brief Add a render to this drawable.
         * @param render [in] the render.
         */
        void addRender( rw::common::Ptr<rw::graphics::Render> render ){
            _renders.push_back(render);
        }

    protected:
    	/**
    	 * @brief constructor
    	 */
    	//Drawable() {}

        /**
         * @brief The renderer that is used to render and draw the
         * drawable
         */
    	std::vector<rw::common::Ptr<rw::graphics::Render> > _renders;

        /**
         * @brief drawType specified how the Drawable should be visualized
         */
    	rw::graphics::DrawableNode::DrawType _drawType;

        /**
         * @brief alpha represents the color alpha value for which, \f$ 0.0 \f$
         * corresponds to fully transparent and \f$1.0\f$ to completely solid.
         * Default is \f$1.0\f$.
         */
        float _alpha;

        /**
         * @brief highlighted specifies whether the drawable should be highlighted.
         * Default is false
         */
        bool _highlighted;

        /**
         * @brief The scale of the object
         */
        float _scale;

        /**
         * @brief True if drawable is enabled/visible, false otherwise
         */
        bool _enable;

        /**
         * @brief The transformation that is applied to the drawable object.
         */
        rw::math::Transform3D<> _t3d;

    private:
    	GLfloat gltrans[16];
    	unsigned int _dmask;
        Drawable(const Drawable&);
        Drawable& operator=(const Drawable&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
