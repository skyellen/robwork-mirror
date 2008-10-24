/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RWLIBS_DRAWABLE_RENDERIVG_HPP
#define RWLIBS_DRAWABLE_RENDERIVG_HPP

/**
 * @file RenderIVG.hpp
 */

#include "Render.hpp"
#include "IVGReader.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <rw/geometry/Face.hpp>
#include <vector>


namespace rwlibs { namespace drawable {

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
}

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
	    void setFaces(const std::vector<rw::geometry::Face<float> >& faces);

        /**
         * @copydoc Render::draw
         */
	    void draw(DrawType type, double alpha) const{
	    	glCallList(_displayListId);
	    }

    private:
        //std::vector<rw::geometry::Face<float> > _vfaces;
        std::vector<ColorFace> _vfaces;
        GLfloat _r, _g, _b;
        GLuint _displayListId;
        void ReadIVG(const std::string &filename);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
