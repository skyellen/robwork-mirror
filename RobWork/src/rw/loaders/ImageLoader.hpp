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

#ifndef RW_LOADERS_IMAGE_IMAGELOADER_HPP
#define RW_LOADERS_IMAGE_IMAGELOADER_HPP

#include <rw/sensor/Image.hpp>
#include <rw/common/ExtensionPoint.hpp>

namespace rw { namespace loaders {

    /** @addtogroup loaders */
	//! @{

    /**
     * @brief Image loader interface
     *
     */
	class ImageLoader
	{
	public:
		//! smart pointer type
	    typedef rw::common::Ptr<ImageLoader> Ptr;

	    //! destructor
	    virtual ~ImageLoader(){}

        /**
         * @param filename [in] name of the file that is to be loaded.
         * @return if loaded successfully a pointer to the image is returned else NULL
         */
		virtual rw::sensor::Image::Ptr loadImage(const std::string& filename) = 0;

		/**
		 * @brief get the list of supported image formats (as extensions)
		 * example (PNG,PGM,GIF)
		 * @return
		 */
		virtual std::vector<std::string> getImageFormats() = 0;

		/**
		 *
		 * @param format
		 * @return
		 */
		virtual bool isImageSupported(const std::string& format);


    	/**
    	 * @addtogroup extensionpoints
    	 * @extensionpoint{rw::loaders::ImageLoader::Factory,rw::loaders::ImageLoader,rw.loaders.ImageLoader}
 	 	 */

		/**
		 * @brief a factory for ImageLoader. This factory also defines an
		 * extension point for image loaders.
		 */
	    class Factory: public rw::common::ExtensionPoint<ImageLoader> {
	    public:
	    	//! constructor
	        Factory():rw::common::ExtensionPoint<ImageLoader>("rw.loaders.ImageLoader", "Example extension point"){};

	        /**
	         * @brief get an image loader for a specific file format
	         * @param format [in] image format eg. png, jpeg, ...
	         * @return
	         */
	        static rw::common::Ptr<ImageLoader> getImageLoader(const std::string& format);

	        /**
	         * @brief test if a imageloader for a specific fileformat exists.
	         * @param format [in] image format eg. png, jpeg ...
	         * @return
	         */
	        static bool hasImageLoader(const std::string& format);

	        /**
	         * @brief get a list of supported formats
	         * @return
	         */
	        static std::vector<std::string> getSupportedFormats();

	        ///// FOR BACKWARD COMPATIBILITY

	        /**
	         * @brief Loads/imports an image from a file.
	         *
	         * An exception is thrown if the file can't be loaded.
	         *
	         * @param filename [in] name of image file.
	         */
			static rw::sensor::Image::Ptr load(const std::string& filename);

	    };

	};

	// for backwards compatibility
	/**
	 * @brief Shortcut type for the ImageLoader::Factory
	 * @deprecated Please use ImageLoader::Factory instead.
	 */
	typedef ImageLoader::Factory ImageFactory;
	/*@}*/
}}

#endif /*RW_LOADERS_PGMLOADER_HPP*/
