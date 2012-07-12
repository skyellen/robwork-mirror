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


#include "Image25D.hpp"

#include <boost/foreach.hpp>

using namespace rw::sensor;
using namespace rw::common;
using namespace rw::math;

Image25D::~Image25D(){

}

Image::Ptr Image25D::asImage() const {
    float min = 10000000;
    float max = -100000000;
    BOOST_FOREACH(const Vector3D<float>& p, _data){
        min = std::min(min, p(2));
        max = std::max(max, p(2));
    }
    if(min>max-0.001)
        max+=0.001;
    //std::cout << min << " " << max << std::endl;
    return asImage(min, max);
}

Image::Ptr Image25D::asImage(float min, float max) const {

    Image::Ptr outImg = ownedPtr( new Image(_width,_height, Image::GRAY, Image::Depth16U));
    float offset = min;
    float scale = 1.0/(max - offset);
    for(unsigned int i = 0; i < _width; i++) {
        for(unsigned int j = 0; j < _height; j++) {
            outImg->setPixel16U(i,j, (uint16_t)((1.0-((_data[j*_width+i](2))-offset)*scale)*65400) );
            //std::cout << (uint16_t)((1.0-((_data[j*_width+i](2))-offset)*scale)*65400) << " --- "
            //        << _data[j*_width+i](2) << "\n";

        }
    }
    return outImg;
}
