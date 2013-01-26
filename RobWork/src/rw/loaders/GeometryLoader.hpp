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

#ifndef RW_LOADERS_GEOMETRYLOADER_HPP_
#define RW_LOADERS_GEOMETRYLOADER_HPP_

//! @file GeometryLoader.hpp

#include <rw/geometry/Geometry.hpp>

namespace rw {
namespace loaders {

	//! @addtogroup loaders
	// @{


	/**
	 * @brief interface for classes that are able to load 3d models
	 */
    class GeometryLoader {
    public:

    	//! destructor
        virtual ~GeometryLoader(){};

    	/**
         * @brief load a Geometry from file \b filename
         * @param filename [in] name of file to load
         * @return a model3d if loaded successfully else NULL (or exception)
         */
        virtual rw::geometry::Geometry::Ptr loadGeomtry(const std::string& filename) = 0;


    };
    //! @}
}
}
#endif /* RW_GRAPHICS_MODEL3DLOADER_HPP_ */
