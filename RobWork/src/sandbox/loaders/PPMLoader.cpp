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



#include "PPMLoader.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::sensor;

rw::sensor::ImagePtr PPMLoader::load(const std::string& fileName)
{
    // open file as binary
    FILE *imagefile = fopen(fileName.c_str(), "rb");

    if (imagefile == NULL) {
        RW_THROW("Can't open image file for writing!");
        return false;
    }
    // P6 :read magic number
    // # : comments
    // width height
    // maxvalue
    // binary data from top to bottom

    if (getBitsPerPixel() == 8) {
        fprintf(imagefile,"P6\n");
        fprintf(imagefile,"# CREATOR: RobWork - www.robwork.dk \n");
        fprintf(imagefile,"%u %u \n", _width, _height);
        fprintf(imagefile,"255\n");

        for(int y=_height-1; y>=0 ;y--){
            unsigned int idx = y*_widthStep;
            fwrite(&(*_imageData)[idx], 1, _widthStep, imagefile);
        }

        //fwrite(&(*_imageData)[0], 1, _imageData->size(), imagefile);
        fclose(imagefile);
    } else {
        fclose(imagefile);
        RW_THROW("Image depth not supported!");
        return false;
    }

    Image::ColorCode coding = Image::GRAY;
    Image::PixelDepth depth = Image::Depth16U;
    if(p.maxgrayval<256)
    	depth = Image::Depth8U;

    std::vector<unsigned char> *data =
        (std::vector<unsigned char>*)output.release();

    return ownedPtr(
        new Image(data, p.width, p.height, coding, depth));

}
