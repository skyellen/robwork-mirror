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


#ifndef RWLIBS_DRAWABLE_RENDERIVG_HPP
#define RWLIBS_DRAWABLE_RENDERIVG_HPP

/**
 * @file RenderIVG.hpp
 */

#include "Render.hpp"
#include "IVGReader.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <vector>


namespace rwlibs { namespace drawable {

// Move into RenderIVG as private class (to avoid warning on GCC 4.3)
/*
namespace
{
    struct ColorFace
    {
    public:
        float _normal1[3];
        float _normal2[3];
        float _normal3[3];
        float _vertex1[3];
        float _vertex2[3];
        float _vertex3[3];
        GLfloat _r, _g, _b, _alpha;

        ColorFace(GLfloat r, GLfloat g, GLfloat b, GLfloat alpha)
        : _r(r), _g(g), _b(b), _alpha(alpha)
        {}

        ColorFace(const ColorFace &cf)
        {
            for(int i=0; i<3; i++)
            {
                _normal1[i] = cf._normal1[i];
                _normal2[i] = cf._normal2[i];
                _normal3[i] = cf._normal3[i];

                _vertex1[i] = cf._vertex1[i];
                _vertex2[i] = cf._vertex2[i];
                _vertex3[i] = cf._vertex3[i];
            }
            _r = cf._r;
            _g = cf._g;
            _b = cf._b;
            _alpha = cf._alpha;
        }
    };
}*/

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads geometry from an IVG file format
     */
    class RenderIVG : public Render
    {
    public:
        /**
         * @brief Creates object
         * @param filename [in] filename of .ivg file
         * @param r [in] red value
         * @param g [in] green value
         * @param b [in] blue value
         */
        RenderIVG(
            const std::string &filename,
            float r = 0.55,
            float g = 1,
            float b = 0.6);

	    /**
	     * @brief Creates RenderIVG without any triangles in
	     */
	    RenderIVG(float r = 0.55, float g = 1, float b = 0.6);

        /**
         * @brief Sets the color to be used for the IVG-file
         * @param r [un] red component \f$r\in [0,1] \f$
         * @param g [un] red component \f$g\in [0,1] \f$
         * @param b [un] red component \f$b\in [0,1] \f$
         */
        void setColor(float r, float g, float b);

	    /**
	     * @brief Sets the faces
	     * @param faces [in] vector with faces
	     */
	    void setTriMesh(const rw::geometry::TriMesh& faces);

        /**
         * @copydoc Render::draw
         */
	    void draw(DrawType type, double alpha) const{
	    	glCallList(_displayListId);
	    }

    private:

        struct ColorFace
        {
        public:
            float _normal1[3];
            float _normal2[3];
            float _normal3[3];
            float _vertex1[3];
            float _vertex2[3];
            float _vertex3[3];
            GLfloat _r, _g, _b, _alpha;

            ColorFace(GLfloat r, GLfloat g, GLfloat b, GLfloat alpha)
            : _r(r), _g(g), _b(b), _alpha(alpha)
            {}

            ColorFace(const ColorFace &cf)
            {
                for(int i=0; i<3; i++)
                {
                    _normal1[i] = cf._normal1[i];
                    _normal2[i] = cf._normal2[i];
                    _normal3[i] = cf._normal3[i];

                    _vertex1[i] = cf._vertex1[i];
                    _vertex2[i] = cf._vertex2[i];
                    _vertex3[i] = cf._vertex3[i];
                }
                _r = cf._r;
                _g = cf._g;
                _b = cf._b;
                _alpha = cf._alpha;
            }
        };

        //std::vector<rw::geometry::Face<float> > _vfaces;
        std::vector<ColorFace> _vfaces;
        GLfloat _r, _g, _b;
        GLuint _displayListId;

        void readIVG(const std::string &filename);

        void drawFace(const ColorFace& cface);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
