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

#ifndef rwlibs_drawable_DrawableFrame_HPP
#define rwlibs_drawable_DrawableFrame_HPP

/**
 * @file DrawableFrame.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include "Drawable.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief DrawableFrame makes a visualization of a frame
     */
    class DrawableFrame : public Drawable
    {
    private:
        float _size;
        GLUquadricObj *_quadratic;
        GLuint _displaylist;
        void update(UpdateType type);

    public:
        /* Functions inherited from Drawable */
        /**
         * @copydoc Drawable::draw
         */
        void draw() const;

        /**
         * @copydoc Drawable::setHighlighted
         */
        void setHighlighted(bool b);

        /**
         * @brief Constructs a DrawableFrame
         * @param size [in] size of the frame coordinate system
         */
        DrawableFrame(float size=1);

        /**
         * Destroys DrawableFrame
         */
        virtual ~DrawableFrame(){};
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
