/*
 * TextureData.hpp
 *
 *  Created on: 10/01/2011
 *      Author: jimali
 */

#ifndef RW_GRAPHICS_TEXTUREDATA_HPP_
#define RW_GRAPHICS_TEXTUREDATA_HPP_

#include <rw/sensor/Image.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace graphics {

class TextureData {
public:
    TextureData():_name(""),_imageData(NULL){};
    TextureData(const std::string& name, rw::sensor::Image::Ptr img):_name(name),_imageData(img){}
    TextureData(const std::string& name, float r, float g, float b):
        _name(name),_imageData(NULL)
    {
     _rgb[0] = r;
     _rgb[1] = g;
     _rgb[2] = b;
    }

    bool hasImageData(){ return _imageData!=NULL; };
    rw::sensor::Image::Ptr getImageData(){ return _imageData; }

    rw::math::Vector3D<float> getRGBData(){
        return rw::math::Vector3D<float>(_rgb[0],_rgb[1],_rgb[2]);
    }

    const std::string& getName() const { return _name; }

private:
    std::string _name;
    rw::sensor::Image::Ptr _imageData;
    float _rgb[3];
};

}
}

#endif /* TEXTUREDATA_HPP_ */
