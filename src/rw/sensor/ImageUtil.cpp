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

#include "ImageUtil.hpp"

using namespace rw::sensor;

void ImageUtil::convertToGrayscale(const Image& src, Image& dst){
    if( src.getColorEncoding()!=Image::RGB24 )
        RW_THROW("Source image is not an RGB24 image!");

    // initialize dst if its not in the right size and format
    if(dst.getWidth()!=src.getWidth() ||
       dst.getHeight()!=src.getHeight() ||
       dst.getColorEncoding()!=Image::MONO8)
    {
        dst.resize(src.getWidth(),src.getHeight(),Image::MONO8);
    }

    const unsigned char* srcData = src.getImageData();
    unsigned char* dstData = dst.getImageData();

    for(size_t i=0,j=0; i<src.getDataSize();i+=3,j++){
        const unsigned char r = srcData[i+0];
        const unsigned char g = srcData[i+1];
        const unsigned char b = srcData[i+2];
        const unsigned char luma = (unsigned char)(r*0.3 + g*0.59+ b*0.11);
        dstData[j] = luma;
    }
}
