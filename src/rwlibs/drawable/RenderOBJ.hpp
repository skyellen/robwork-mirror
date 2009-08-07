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


#ifndef RWLIBS_DRAWABLE_RENDEROBJ_HPP
#define RWLIBS_DRAWABLE_RENDEROBJ_HPP

/**
 * @file rwlibs/drawable/RenderOBJ.hpp
 */

#include <rwlibs/drawable/Render.hpp>
#include "OBJReader.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <rw/geometry/Face.hpp>
#include <vector>


namespace rwlibs { namespace drawable {

	/** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads geometry from a OBJ file format
     */
    class RenderOBJ : public Render
    {
    public:
        /**
         * @brief Creates object
         * @param filename [in] filename of .ivg file
         */
        RenderOBJ(const std::string &filename);

        /**
         * @copydoc Render::draw
         */
	    void draw(DrawType type, double alpha) const
		{
			glCallList(_displayListId);
			//GLuint displayListId;
			//Render(alpha, displayListId);
			//glCallList(displayListId);
		}

    private:
		OBJReader _reader;
        GLuint _displayListId;

		void render(float alpha, GLuint &displayListId) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
