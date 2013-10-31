/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_LOADERS_LOADERASSIMP_HPP_
#define RW_LOADERS_LOADERASSIMP_HPP_

/**
 * @file LoaderAssimp.hpp
 *
 * \copydoc rw::loaders::LoaderAssimp
 */

#include <rw/loaders/Model3DLoader.hpp>

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Load 3D models using the Open Asset Import Library (Assimp).
 *
 * For further information on Assimp and supported formats, see http://assimp.sourceforge.net
 *
 * So far the loader has been tested for .dae files (Collada).
 *
 * Note that the RobWork loader for Assimp is still work in progress.
 */
class LoaderAssimp: public rw::loaders::Model3DLoader {
public:
	//! @brief Constructor
	LoaderAssimp();

	//! @brief Destructor
	virtual ~LoaderAssimp();

    //! @copydoc Model3DLoader::load
	rw::graphics::Model3D::Ptr load(const std::string& filename);
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_LOADERASSIMP_HPP_ */
