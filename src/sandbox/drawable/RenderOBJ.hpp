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

#ifndef RWLIBS_DRAWABLE_RENDEROBJ_HPP
#define RWLIBS_DRAWABLE_RENDEROBJ_HPP

/**
 * @file RenderOBJ.hpp
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
