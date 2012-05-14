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


#ifndef RW_LOADERS_TGALOADER_HPP
#define RW_LOADERS_TGALOADER_HPP

#include <rw/sensor/Image.hpp>

namespace rw { namespace loaders {
	/**
	 * @brief
	 */
	class TGALoader
	{
	public:
		static rw::sensor::ImagePtr load(const std::string& filename);
	};
}}

#endif /*RW_LOADERS_PGMLOADER_HPP*/
