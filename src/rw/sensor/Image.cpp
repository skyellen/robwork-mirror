/*********************************************************************
 * RobWork Version 0.2
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

#include "Image.hpp"
#include <rw/common/macros.hpp>

#include <iostream>

using namespace rw::sensor;

Image::Image(
    int width,
    int height,
    ColorCode colorCode)
    :
    _width(width),
    _height(height),
    _colorCode(colorCode),
    _imageData( new std::vector<unsigned char>(
        _width * _height * getBitsPerPixel() / 8))
{}

Image::Image(
	std::vector<unsigned char> *image,
    int width,
    int height,
    ColorCode colorCode)
    :
    _width(width),
    _height(height),
    _colorCode(colorCode),
    _imageData(image)
{}

size_t Image::getDataSize() const
{
    return _imageData->size();
}

void Image::resize(int width, int height, ColorCode encoding){
    _width = width;
    _height = height;
    _colorCode = encoding;
    
    if(_imageData==NULL){
        _imageData = new std::vector<unsigned char>(_width * _height * getBitsPerPixel() / 8);
    } else {
        _imageData->resize(_width * _height * getBitsPerPixel() / 8);
    }        
}

unsigned char* Image::getImageData()
{
    return &(*_imageData)[0];
}

const unsigned char* Image::getImageData() const
{
    return &(*_imageData)[0];
}

std::pair<int,int> Image::getImageDimension()
{
    return std::make_pair(_width, _height);
}

unsigned int Image::getWidth() const
{
    return _width;
}

unsigned int Image::getHeight() const
{
    return _height;
}

unsigned int Image::getBitsPerPixel() const
{
    int bitsPerPixel = 8; // the width of the colorcode for 4 pixels
    switch(_colorCode){
    case(MONO8):
    case(RGB8):
    case(RAW8):
        bitsPerPixel = 8;
        break;
    case(YUV422): // (4xY + 2xU + 2V)
    case(MONO16):
    case(RGB16):
    case(MONO16S):
    case(RGB16S):
    case(RAW16):
        bitsPerPixel = 16;
        break;
    case(RGB24):
        bitsPerPixel = 24;
        break;
    case(YUV411): // not currently supported
    case(YUV444):
    default:
        RW_ASSERT(0);
    }
    return bitsPerPixel;
}

bool Image::saveAsPGM(const std::string& fileName) const
{
    FILE *imagefile = fopen(fileName.c_str(), "w");

    if (imagefile == NULL) {
        perror( "Can't create img_file_name");
        return false;
    }

    if (getBitsPerPixel() == 8) {
        fprintf(imagefile,"P5\n%u %u 255\n", _width, _height);
        fwrite(&(*_imageData)[0], 1, _imageData->size(), imagefile);
        fclose(imagefile);
        printf("wrote: img_file_name\n");

    } else {
        perror( "Bad image format!!");
        fclose(imagefile);
        return false;
    }

    return true;
}
