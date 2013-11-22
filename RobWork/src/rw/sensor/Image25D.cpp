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

#include <iostream>
#include <fstream>

using namespace rw::sensor;
using namespace rw::common;
using namespace rw::math;

Image25D::~Image25D(){

}

Image::Ptr Image25D::asImage() const {
    float min = 10000000;
    float max = -100000000;
    BOOST_FOREACH(const Vector3D<float>& p, _data){
        min = std::min(min, -p(2));
        max = std::max(max, -p(2));
    }
    if(min>max-0.001)
        max+=0.001f;
    //std::cout << min << " " << max << std::endl;
    return asImage(min, max);
}

Image::Ptr Image25D::asImage(float min, float max) const {

    Image::Ptr outImg = ownedPtr( new Image(_width,_height, Image::GRAY, Image::Depth8U));
    float offset = min;
    float scale = 1.0f/(max - offset);
    for(unsigned int i = 0; i < _width; i++) {
        for(unsigned int j = 0; j < _height; j++) {

            float val = -(_data[j*_width+i](2));
            val = std::max(min, val);
            val = std::min(max, val);
            val -= offset;
            val *= scale;
            // the value should now be between 0 and 1
            RW_ASSERT(val>=0.0);
            RW_ASSERT(val<=1.0);
            uint8_t ival = (uint8_t)((1-val)*255.0);
            outImg->setPixel8U(i,j, ival);
            //std::cout << (uint16_t)((1.0-((_data[j*_width+i](2))-offset)*scale)*65400) << " --- "
            //        << _data[j*_width+i](2) << "\n";

        }
    }

    return outImg;
}

void Image25D::save(const Image25D& img, const std::string& filename, const rw::math::Transform3D<float>& t3d){
    std::ofstream output(filename.c_str());
    output << "# .PCD v.5 - Point Cloud Data file format\n";
    output << "FIELDS x y z\n";
    output << "SIZE 4 4 4\n";
    output << "TYPE F F F\n";
    output << "WIDTH " << img.getWidth() << "\n";
    output << "HEIGHT " << img.getHeight() << "\n";
    output << "POINTS " << img.getData().size() << "\n";
    output << "DATA ascii\n";
    BOOST_FOREACH(const rw::math::Vector3D<float>& p_tmp, img.getData()){
        Vector3D<float> p = t3d*p_tmp;
        output << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
    output.close();
}

Image25D::Ptr Image25D::load(const std::string& filename ){
    char line[500];
    int width, height, nr_points;
    std::ifstream input(filename.c_str());
    input.getline(line, 500); // output << "# .PCD v.5 - Point Cloud Data file format\n";
    input.getline(line, 500); // output << "FIELDS x y z\n";
    input.getline(line, 500); // output << "SIZE 4 4 4\n";
    input.getline(line, 500); //output << "TYPE F F F\n";
    std::string tmp;
    input >> tmp >> width;
    input.getline(line, 500);
    input >> tmp >> height;
    input.getline(line, 500);
    input >> tmp >> nr_points;
    input.getline(line, 500);
    std::cout << "w:" << width << " h:" << height << " p:" << nr_points << std::endl;
    rw::sensor::Image25D::Ptr img = rw::common::ownedPtr( new rw::sensor::Image25D(width,height) );
    input.getline(line, 500); // output << "DATA ascii\n";

    for(int i=0;i<nr_points;i++){
        Vector3D<float> p;
        input.getline(line, 500);
        sscanf(line, "%f %f %f", &p[0], &p[1], &p[2]);
        img->getData()[i] = p;
    }
    return img;
}

