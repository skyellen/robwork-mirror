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

#ifndef RW_GRAPHICS_MODEL3DLOADER_HPP_
#define RW_GRAPHICS_MODEL3DLOADER_HPP_

//! @file Model3DLoader.hpp

#include "Model3D.hpp"

namespace rw {
namespace graphics {

	//! @addtogroup graphics
	// @{


	/**
	 * @brief interface for classes that are able to load 3d models
	 */
    class Model3DLoader {
    public:
        //! destructor
        virtual ~Model3DLoader(){};
        /**
         * @brief load a Model3D from file \b filename
         * @param filename [in] name of file to load
         * @return a model3d if loaded successfully else NULL (or exception)
         */
        virtual Model3D::Ptr load(const std::string& filename) = 0;

        //virtual void save(Model3DPtr model, const std::string& filename) = 0;

    };
    //! @}
}
}
#endif /* RW_GRAPHICS_MODEL3DLOADER_HPP_ */
