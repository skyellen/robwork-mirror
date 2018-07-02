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

#ifndef RW_GRAPHICS_LOADER3DS_HPP_
#define RW_GRAPHICS_LOADER3DS_HPP_

//! @file Loader3DS.hpp

#include <rw/graphics/Model3D.hpp>
#include "../Model3DLoader.hpp"

#include <string>

namespace rw {
namespace loaders {

    //! @addtogroup graphics
	// @{

    /**
     * @brief This class loads 3DS geometry into a Model3D object.
     */
	class Loader3DS: public Model3DLoader
	{
	public:

        /**
         * @brief constructor
         */
		Loader3DS(){};

		/**
		 * @brief destructor
		 */
		virtual ~Loader3DS(){};

		//! @copydoc Model3DLoader::load
		rw::graphics::Model3D::Ptr load(const std::string& filename); // Loads a model

	private:
		std::string _path; // The path of the model

	};

	//! @}
}
}
#endif // RW_DRAWABLE_LOADER_3DS_H
