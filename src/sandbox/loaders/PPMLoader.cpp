/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/


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
