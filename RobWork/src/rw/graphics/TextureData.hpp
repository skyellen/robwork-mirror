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


#ifndef RW_GRAPHICS_TEXTUREDATA_HPP_
#define RW_GRAPHICS_TEXTUREDATA_HPP_

#include <rw/sensor/Image.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace graphics {

	/**
	 * @brief container for storing texture data.
	 */
	class TextureData {
	public:
		rw::common::Ptr<TextureData> Ptr;

		//! constructor
		TextureData():_name(""),_imageData(NULL){};

		/**
		 * constructor
		 * @param name [in] texture id
		 * @param img [in] texture data
		 */
		TextureData(const std::string& name, rw::sensor::Image::Ptr img):_name(name),_imageData(img){}

		/**
		 * constructor
		 * @param name [in] texture id
		 * @param r [in] red value [0:1]
		 * @param g [in] green value [0:1]
		 * @param b [in] blue value [0:1]
		 */
		TextureData(const std::string& name, float r, float g, float b):
			_name(name),_imageData(NULL)
		{
		 _rgb[0] = r;
		 _rgb[1] = g;
		 _rgb[2] = b;
		}

		/**
		 * @brief check if this texture has image data
		 * @return true if it has image data, false otherwise
		 */
		bool hasImageData() const { return _imageData!=NULL; };

		/**
		 * @brief get image data
		 * @return
		 */
		rw::sensor::Image::Ptr getImageData() const { return _imageData; }

		/**
		 * @brief get RGB data
		 * @return
		 */
		rw::math::Vector3D<float> getRGBData() const {
			return rw::math::Vector3D<float>(_rgb[0],_rgb[1],_rgb[2]);
		}

		/**
		 * @brief get id of texture
		 * @return
		 */
		const std::string& getName() const { return _name; }

	private:
		std::string _name;
		rw::sensor::Image::Ptr _imageData;
		float _rgb[3];
	};

}
}

#endif /* TEXTUREDATA_HPP_ */
