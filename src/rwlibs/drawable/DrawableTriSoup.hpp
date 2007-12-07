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

#ifndef rwlibs_drawable_DrawableTriSoup_HPP
#define rwlibs_drawable_DrawableTriSoup_HPP

/**
 * @file DrawableTriSoup.hpp
 */

#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>

#include <vector>

#include <rwlibs/os/rwgl.hpp>

#include <rw/geometry/Face.hpp>

#include "Drawable.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads geometry from an simple
     * custom file format.
     */
    class DrawableTriSoup : public Drawable
    {
    public:
        /**
         * @brief Constructor which loads file
         * @param filename [in] file to load
         */
        DrawableTriSoup(const std::string &filename);

        /**
         * @brief default constructor
         */
        DrawableTriSoup();

        /**
         * @brief Adds faces to triangles soup
         * @param faces [in] list of faces to add
         * @param r [in] red color component
         * @param g [in] green color component
         * @param b [in] blue color component
         */
        void addFaces(
            const std::vector<rw::geometry::Face<float> >& faces,
            float r,
            float g,
            float b);

        /**
         * @brief Clears the list of faces
         */
        void clearFaces();

    protected:
        /**
         * @copydoc Drawable::update
         */
        void update(UpdateType type);

    private:
        // The RGB name collides with a macro!
        struct Rgb {
            float val[3];

            Rgb(float r, float g, float b) { val[0] = r; val[1] = g; val[2] = b; }
        };

        struct Vertex {
            float val[3];

            Vertex(float x,float y,float z){ val[0] = x; val[1] = y; val[2] = z; }
        };

        typedef Vertex Normal;

        std::vector<Vertex> _vertexArray;
        std::vector<Normal> _normalArray;
        std::vector<Rgb> _rgbArray;
        std::vector<int> _rgbToVertexMap;

    private:
        void loadTriFile(const std::string& filename);
        void render();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
