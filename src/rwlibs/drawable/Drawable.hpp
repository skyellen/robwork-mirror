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

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <rwlibs/os/rwgl.hpp>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Abstract base class for all drawable classes
     *
     * Classes that are able to draw itself may inherit from this class.
     */
    class Drawable {
    public:
        /**
         * @brief when calling draw on a drawable object the draw mode or type
         * can be specified.
         */
        enum DrawType {
            SOLID, //! draw the drawable in solid
            WIRE, //! draw the drawable in wireframe
            OUTLINE //! draw the drawable in both solid and wireframe
        };

        /**
         * @brief Constructer for Drawable
         *
         * @param drawType [in] DrawType of the Drawable. Default value is SOLID
         *
         * @param alpha [in] The color alpha value. \f$ 0.0 \f$
         * corresponds to fully transparent and \f$1.0\f$ to completely
         * solid. Default is \f$1.0\f$.
         */
        Drawable(DrawType drawType = SOLID, float alpha = 1.0f);

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
        void setDrawType(DrawType drawType);

        /**
         * @brief Sets up the color alpha value.
         *
         * @param alpha [in] \f$ 0.0 \f$ corresponds to fully transparent and
         * \f$1.0\f$ to completely solid.
         */
        void setAlpha(float alpha);

        /**
         * @brief Sets the highlight color
         *
         * @param r [in] Red component \f$ r\in[0,1]\f$
         *
         * @param g [in] Green component \f$ g\in[0,1]\f$
         *
         * @param b [in] Blue component \f$ b\in[0,1]\f$
         */
        void setHighlightColor(float r, float g, float b);

        /**
         * @brief Specifies the scale of the object
         * @param scale [in] the scale
         */
        void setScale(float scale);

    protected:
        /**
         * @todo jimmy: document this
         */
        enum UpdateType {
            //! DrawType changed
            DRAWTYPE = 0,
            //! Highlight state changed
            HIGHLIGHT,
            //! Alpha value changed
            ALPHA,
            //! Custom change
            CUSTOM };

        /**
         * @brief All classes inheriting from Drawable should implement update, which
         * is called whenever one of the properties of the drawable changes.
         */
        virtual void update(UpdateType type) = 0;

        /**
         * _drawType specified how the Drawable should be visualized
         */
        DrawType _drawType;

        /**
         * _alpha represents the color alpha value for which, \f$ 0.0 \f$
         * corresponds to fully transparent and \f$1.0\f$ to completely solid.
         * Default is \f$1.0\f$.
         */
        float _alpha;

        /**
         * _highlighted specifies whether the drawable should be highlighted.
         * Default is false
         */
        bool _highlighted;

        /**
         * @todo jimmy: document this
         */
        rw::math::Vector3D<float> _highlightColor;

        /**
         * @todo jimmy: document this
         */
        GLuint _displayListId;

        /**
         * The scale of the object
         */
        float _scale;

    private:
        Drawable(const Drawable&);
        Drawable& operator=(const Drawable&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
