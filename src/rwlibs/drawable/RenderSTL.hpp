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
	     * @brief Returns the faces associated to the render
	     * @return Reference to vector with faces
	     */
	    const std::vector<rw::geometry::Face<float> >& getFaces() const;

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
