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


#ifndef RW_LOADERS_IMAGE_RGBLOADER_HPP
#define RW_LOADERS_IMAGE_RGBLOADER_HPP

#include <rw/sensor/Image.hpp>
#include "../ImageLoader.hpp"

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Loads images in Silicon Graphics RGB format.
     *
     */
	class RGBLoader : public ImageLoader
	{
	public:

		RGBLoader(){};

		virtual ~RGBLoader(){};

        //! @copydoc ImageLoader::loadImage
        rw::sensor::Image::Ptr loadImage(const std::string& filename);

        //! @copydoc ImageLoader::getImageFormats
        std::vector<std::string> getImageFormats();

        /**
         * @param filename [in] name of the file that is to be loaded.
         * @return if loaded successfully a pointer to the image is returned else NULL
         */
		static rw::sensor::Image::Ptr load(const std::string& filename);

		/**
         * @param img [in] the image that is to be saved.
         * @param filename [in] name of the file where the image is to be saved.
         * @return if loaded successfully a pointer to the image is returned else NULL
         */
		//static void save(rw::sensor::Image::Ptr img, const std::string& filename);

	};
	/*@}*/
}}

#endif /*RW_LOADERS_PGMLOADER_HPP*/
