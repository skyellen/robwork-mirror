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
#include <stdio.h>

using namespace rw::sensor;
using namespace rw::common;

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
            RW_THROW(0);
        }
    }

    size_t getStride(Image::PixelDepth depth){
        switch(depth){
        case(Image::Depth8U):
        case(Image::Depth8S):
            return 1;
        break;
        case(Image::Depth16U):
        case(Image::Depth16S):
        	return 2;
        break;
        case(Image::Depth32S):
        case(Image::Depth32F):
        	return 4;
        break;
        default:
            RW_THROW("Unsupported pixel depth!");
        }
        
    }

    unsigned int calcMask(Image::PixelDepth depth){
        switch(depth){
        case(Image::Depth8U):
        case(Image::Depth8S):
            return 0xFF;
        break;
        case(Image::Depth16U):
        case(Image::Depth16S):
        	return 0xFFFF;
        break;
        case(Image::Depth32S):
        case(Image::Depth32F):
        	return 0xFFFFFFFF;
        break;
        default:
            RW_THROW("Unsupported pixel depth!");
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
    _arrSize(0),
    _imageData(NULL),
    _stride(getStride(Depth8U)),
    _valueMask(calcMask(Depth8U))
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
    _arrSize(_width * _height * _nrChannels * getBitsPerPixel() / 8),
    _imageData(new char[_arrSize]),
    _stride(getStride(depth)),
    _valueMask(calcMask(depth))
{}

Image::Image(
	char *imageData,
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
    _arrSize(_width * _height * _nrChannels * getBitsPerPixel() / 8),
    _imageData(imageData),
    _stride(getStride(depth)),
    _valueMask(calcMask(depth))
{}

size_t Image::getDataSize() const
{
    return _arrSize;
}

Pixel4f Image::getPixel(size_t x, size_t y) const {
	const size_t idx = y*_widthStep + x*_nrChannels;

	// convert idx to point into char array
	const size_t cidx = idx*_stride;
	RW_ASSERT(cidx<_arrSize);

	// now if representation is float then we can set it directly
	if(_depth == Image::Depth32F){
		Pixel4f p((float)_imageData[cidx], 0, 0, 0);
		for(size_t i=1;i<_nrChannels;i++)
			p.ch[i] = (float)_imageData[cidx+i*_stride];
		return p;
	}
	// is an int so we need to convert it to float
	Pixel4f p((float)(_imageData[cidx]&_valueMask), 0, 0, 0);
	for(size_t i=1;i<_nrChannels;i++)
		p.ch[i] = (float)(_imageData[cidx+i*_stride]&_valueMask);
	return p;
}

float Image::getPixelValue(size_t x, size_t y, size_t channel) const {
	const size_t idx = y*_widthStep + x*_nrChannels;

	// convert idx to point into char array
	const size_t cidx = idx*_stride;
	RW_ASSERT(cidx<_arrSize);

	if(_depth == Image::Depth32F){
		return (float)_imageData[cidx+channel*_stride];
	} else {
		// is in int so we need to convert it to float
		return (float)(_imageData[cidx+channel*_stride]&_valueMask);
	}
}

void Image::safeDeleteData(){
    if(_imageData==NULL )
        return;
    _arrSize = 0;
    _widthStep = 0;
    _width = 0;
    _height = 0;
    delete[] _imageData;
}

void Image::resize(int width, int height){
    if(width==_width && _height==height)
        return;
    safeDeleteData();

    _width = width;
    _height = height;
    size_t arrSize = _width * _height * _nrChannels * getBitsPerPixel() / 8;
    _imageData = new char[arrSize];
    _widthStep = _width*_nrChannels;
}

char* Image::getImageData()
{
    return _imageData;
}

const char* Image::getImageData() const
{
    return _imageData;
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
            //std::cout << y << " " << idx << std::endl;

            fwrite((unsigned char*)&(_imageData[idx]), 1, _width, imagefile);
        }
        //std::cout << "Closing image file" << std::endl;
        fclose(imagefile);
        //printf("wrote: img_file_name\n");

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
            size_t idx = y*_widthStep;
            for(size_t x=0;x<_width;x++){
            	unsigned char *arr = (unsigned char*) _imageData;
                fprintf(imagefile,"%u ", arr[idx+x]);
            }
            fprintf(imagefile,"\n");
        }
        fclose(imagefile);
        //printf("wrote: img_file_name\n");

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
            fwrite(&_imageData[idx], 1, _widthStep, imagefile);
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
