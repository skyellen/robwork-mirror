/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RWLIBS_DRAWABLE_RENDERFRAME_HPP
#define RWLIBS_DRAWABLE_RENDERFRAME_HPP

/**
 * @file RenderFrame.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include "Render.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief RenderFrame makes a visualization of a frame
     */
    class RenderFrame : public Render
    {
    private:
        float _size;
        GLUquadricObj *_quadratic;
        GLuint _displayListId;

        mutable float _green[4];
        mutable float _red[4];
        mutable float _blue[4];
    public:

        /**
         * @brief Constructs a RenderFrame
         * @param size [in] size of the frame coordinate system
         */
        RenderFrame(float size=1);

        /**
         * @brief Destructor
         */
        virtual ~RenderFrame(){
        	glDeleteLists(_displayListId,1);
        };

    	/* Functions inherited from Render */

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
