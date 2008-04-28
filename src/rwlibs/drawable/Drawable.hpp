/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef rwlibs_drawable_Drawable_HPP
#define rwlibs_drawable_Drawable_HPP

/**
 * @file Drawable.hpp
 */

#include "Render.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <rwlibs/os/rwgl.hpp>

#include <boost/shared_ptr.hpp>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Abstract base class for all drawable classes
     *
     * Classes that are able to draw them self, may inherit from this class.
     */
    class Drawable {
    public:
        /**
         * @brief Constructer for Drawable
         *
         * @param render [in] Render to be used by the drawable
         * @param drawType [in] DrawType of the Drawable. Default value is SOLID
         *
         * @param alpha [in] The color alpha value. \f$ 0.0 \f$
         * corresponds to fully transparent and \f$1.0\f$ to completely
         * solid. Default is \f$1.0\f$.
         */
        Drawable(boost::shared_ptr<Render> render,
        		 Render::DrawType drawType = Render::SOLID,
        		 float alpha = 1.0f);

        /**
         * @brief Virtual destructor
         */
        virtual ~Drawable();

        /**
         * @brief draws the object.
         */
        virtual void draw() const;

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
        void setDrawType(Render::DrawType drawType);

        /**
         * @brief Sets up the color alpha value.
         *
         * @param alpha [in] \f$ 0.0 \f$ corresponds to fully transparent and
         * \f$1.0\f$ to completely solid.
         */
        void setAlpha(float alpha);

        /**
         * @brief Gets the color alpha value.
         * @return alpha value in the interval \f$ [0.0;1.0] \f$
         */
        float getAlpha(float alpha);

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
        void setEnabled(bool enable) { _enable = enable; }

        /**
         * @brief checks if this drawable is enabled
         */
        bool isEnabled() { return _enable; }

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
         * @brief Get this drawables Render object
         */
        boost::shared_ptr<Render> getRender() const{
        	return _render;
        }

    protected:
    	/**
    	 * @brief constructor
    	 */
    	Drawable() {}

        /**
         * @brief The renderer that is used to render and draw the
         * drawable
         */
        boost::shared_ptr<Render> _render;

        /**
         * @brief drawType specified how the Drawable should be visualized
         */
        Render::DrawType _drawType;

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

        Drawable(const Drawable&);
        Drawable& operator=(const Drawable&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
