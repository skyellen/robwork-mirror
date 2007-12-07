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

#ifndef rwlibs_drawable_DrawableSTL_HPP
#define rwlibs_drawable_DrawableSTL_HPP

/**
 * @file DrawableSTL.hpp
 */

#include "Drawable.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <rw/geometry/GeometrySTL.hpp>
#include <vector>


namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads geometry from an ASCII or binary STL
     * (stereolithography) file format.
     */
    class DrawableSTL : public Drawable
    {
    public:
        /**
         * @brief Creates object
         * @param filename [in] filename of .stl file
         * @param r [in] red value
         * @param g [in] green value
         * @param b [in] blue value
         */
        DrawableSTL(
            const std::string &filename,
            float r = 0.55,
            float g = 1,
            float b = 0.6);

    /**
     * @brief Creates DrawableSTL without any triangles in
     */
    DrawableSTL(float r = 0.55, float g = 1, float b = 0.6);

        /**
         * @brief Sets the color to be used for the STL-file
         * @param r [un] red component \f$r\in [0,1] \f$
         * @param g [un] red component \f$g\in [0,1] \f$
         * @param b [un] red component \f$b\in [0,1] \f$
         */
        void setColor(float r, float g, float b);

    /**
     * @brief Sets the faces
     * @param faces [in] vector with faces
     */
    void setFaces(const std::vector<rw::geometry::Face<float> >& faces);

    protected:
        /**
         * @copydoc Drawable::update
         */
        void update(UpdateType type);

    private:
        std::vector<rw::geometry::Face<float> > _vfaces;
        GLfloat _r, _g, _b;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
