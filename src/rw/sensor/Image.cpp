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


#include "Image.hpp"
#include <rw/common/macros.hpp>

#include <iostream>

using namespace rw::sensor;

namespace {

    unsigned int toNrOfChannels(Image::ColorCode ccode){
        switch(ccode){
        case(Image::GRAY):
            return 1;
        case(Image::RGB):
        case(Image::BGR):
            return 3;
        case(Image::RGBA):
        case(Image::BGRA):
            return 4;
        default:
            RW_ASSERT(0);
        }
    }

}

Image::Image():
    _width(0),
    _height(0),
    _colorCode(GRAY),
    _depth(Depth8U),
    _nrChannels(toNrOfChannels(_colorCode)),
    _widthStep(_width*_nrChannels),
    _imageData(NULL)
{};


Image::Image(
    int width,
    int height,
    ColorCode colorCode,
    PixelDepth depth)
    :
    _width(width),
    _height(height),
    _colorCode(colorCode),
    _depth(depth),
    _nrChannels(toNrOfChannels(colorCode)),
    _widthStep(width*_nrChannels),
    _imageData( new std::vector<unsigned char>(
        _width * _height * _nrChannels * getBitsPerPixel() / 8))

{}

Image::Image(
	std::vector<unsigned char> *image,
    int width,
    int height,
    ColorCode colorCode,
    PixelDepth depth)
    :
    _width(width),
    _height(height),
    _colorCode(colorCode),
    _depth(depth),
    _nrChannels(toNrOfChannels(colorCode)),
    _widthStep(width*_nrChannels),
    _imageData(image)
{}

size_t Image::getDataSize() const
{
    return _imageData->size();
}

void Image::resize(int width, int height){
    _width = width;
    _height = height;

    if(_imageData==NULL){
        _imageData = new std::vector<unsigned char>(_width * _height * _nrChannels * getBitsPerPixel() / 8);
    } else {
        _imageData->resize(_width * _height * _nrChannels * getBitsPerPixel() / 8);
    }
    _widthStep = _width*_nrChannels;
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
    /*switch(_colorCode){
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
        break;
    case(RGB32):
    case(MONO32):
    case(MONO32S):
        bitsPerPixel = 32;
    default:
        RW_ASSERT(0);
    }*/

    switch(_depth){
    case(Depth8U):
    case(Depth8S):
        bitsPerPixel = 8;
    break;
    case(Depth16U):
    case(Depth16S):
        bitsPerPixel = 16;
    break;
    case(Depth32S):
    case(Depth32F):
        bitsPerPixel = 32;
    break;
    default:
        RW_WARN("Unsupported pixel depth!");
        RW_ASSERT(0);
    }
    return bitsPerPixel;
}

bool Image::saveAsPGM(const std::string& fileName) const
{
    if(_colorCode!=GRAY){
        RW_THROW("Image must be of type GRAY!");
    }

    // open file as binary file
    FILE *imagefile = fopen(fileName.c_str(), "wb");

    if (imagefile == NULL) {
        perror( "Can't create img_file_name");
        return false;
    }
/*
    unsigned int maxColorVal = 1<<getBitsPerPixel();
    double scale = 1;
    if( maxColorVal>=65536 ){
        scale = 65536/maxColorVal;
    }
    */

    if (getBitsPerPixel() == 8) {
        fprintf(imagefile,"P5\n");
        fprintf(imagefile,"# CREATOR: RobWork - www.robwork.dk \n");
        fprintf(imagefile,"%u %u \n", _width, _height);
        fprintf(imagefile,"255\n");

        // now print all row in reverse order
        for(size_t y=0;y<_height;y++){
            unsigned int idx = y*_widthStep;
            std::cout << y << " " << idx << std::endl;
            fwrite(&(*_imageData)[idx], 1, _width, imagefile);
        }
        std::cout << "Closing image file" << std::endl;
        fclose(imagefile);
        printf("wrote: img_file_name\n");

    } else {
        perror( "Bad image format!!");
        fclose(imagefile);
        return false;
    }

    return true;
}

bool Image::saveAsPGMAscii(const std::string& fileName) const {
    if(_colorCode!=GRAY){
        RW_THROW("Image must be of type GRAY!");
    }
    // open file as ascii
    FILE *imagefile = fopen(fileName.c_str(), "w");

    if (imagefile == NULL) {
        perror( "Can't create img_file_name");
        return false;
    }
/*
    unsigned int maxColorVal = 1<<getBitsPerPixel();
    double scale = 1;
    if( maxColorVal>=65536 ){
        scale = 65536/maxColorVal;
    }
    */

    if (getBitsPerPixel() == 8) {
        fprintf(imagefile,"P2\n");
        fprintf(imagefile,"# CREATOR: RobWork - www.robwork.dk \n");
        fprintf(imagefile,"%u %u \n", _width, _height);
        fprintf(imagefile,"255\n");

        // now print all row in reverse order
        for(size_t y=0;y<_height;y++){
            unsigned int idx = y*_widthStep;
            for(size_t x=0;x<_width;x++){
                fprintf(imagefile,"%u ", (*_imageData)[idx+x]);
            }
            fprintf(imagefile,"\n");
        }
        fclose(imagefile);
        printf("wrote: img_file_name\n");

    } else {
        perror( "Bad image format!!");
        fclose(imagefile);
        return false;
    }

    return true;
}


bool Image::saveAsPPM(const std::string& fileName) const
{
    // open file as binary
    FILE *imagefile = fopen(fileName.c_str(), "wb");

    if (imagefile == NULL) {
        RW_THROW("Can't open image file for writing!");
        return false;
    }

    if (getBitsPerPixel() == 8) {
        fprintf(imagefile,"P6\n");
        fprintf(imagefile,"# CREATOR: RobWork - www.robwork.dk \n");
        fprintf(imagefile,"%u %u \n", _width, _height);
        fprintf(imagefile,"255\n");

        for(size_t y=0;y<_height;y++){
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

    return true;
}
