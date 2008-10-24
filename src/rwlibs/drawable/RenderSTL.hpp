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

#ifndef RWLIBS_DRAWABLE_RENDERSTL_HPP
#define RWLIBS_DRAWABLE_RENDERSTL_HPP

/**
 * @file RenderSTL.hpp
 */

#include "Render.hpp"
#include "RenderGeometry.hpp"


#include <rwlibs/os/rwgl.hpp>
#include <rw/geometry/GeometrySTL.hpp>
#include <vector>
#include <list>


namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads geometry from an ASCII or binary STL
     * (stereolithography) file format.
     */
    class RenderSTL : public Render
    {
    public:
        /**
         * @brief Creates object
         * @param filename [in] filename of .stl file
         */
        RenderSTL(const std::string &filename);

        /**
         * @brief Destructor
         */
	    virtual ~RenderSTL(){
	    	glDeleteLists(_displayListId, 1);
	    }

        /**
         * @brief Sets the color to be used for the STL-file
         * @param r [un] red component \f$r\in [0,1] \f$
         * @param g [un] red component \f$g\in [0,1] \f$
         * @param b [un] red component \f$b\in [0,1] \f$
         */
        void setColor(float r, float g, float b){
        	//_renderer->setColor(r,g,b);
        	_r = r;
        	_g = g;
        	_b = b;
        }

	    /**
	     * @brief Sets the faces
	     * @param faces [in] vector with faces
	     */
	    void setFaces(const std::vector<rw::geometry::Face<float> >& faces);

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    private:
    	//class FaceArrayGeometry _geom;
    	//RenderGeometry *_renderer;
    	std::vector<rw::geometry::Face<float> > _faces;
    	mutable GLfloat _diffuse[4];
    	GLfloat _ambient[4];
    	GLfloat _emission[4];
    	GLfloat _specular[4];
    	GLfloat _shininess[1];

    	GLuint _displayListId;
        float _r, _g, _b;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
