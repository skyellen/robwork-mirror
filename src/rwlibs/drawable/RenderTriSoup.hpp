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


#ifndef RWLIBS_DRAWABLE_RENDERTRISOUP_HPP
#define RWLIBS_DRAWABLE_RENDERTRISOUP_HPP

/**
 * @file RenderTriSoup.hpp
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

#include "Render.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads geometry from an simple
     * custom file format.
     */
    class RenderTriSoup : public Render
    {
    public:

        /**
         * @brief Constructor
         */
        RenderTriSoup();

    	/**
         * @brief Constructor which loads file
         * @param filename [in] file to load
         */
        RenderTriSoup(const std::string &filename);

        /**
         * @brief Destructor
         */
        virtual ~RenderTriSoup();

        /**
         * @brief Adds faces to triangles soup
         * @param faces [in] list of faces to add
         * @param r [in] red color component
         * @param g [in] green color component
         * @param b [in] blue color component
         */
        void addFaces(
            const std::vector<rw::geometry::Face<float> >& faces,
            double r,
            double g,
            double b);

        /**
         * @brief Clears the list of faces
         */
        void clearFaces();

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

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

        std::string _id;
        GLuint _displayListId;

    private:
        void loadTriFile(const std::string& filename);
        void rerender();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
