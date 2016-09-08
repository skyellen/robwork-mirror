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

#include <stdio.h>
#include <string.h>
#include <cmath>

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
            break;
        }
        return 0;
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
            break;
        }
        return 0;
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
            break;
        }
        return 0;
    }

}
void Image::initFloatConversion(){
    switch(_depth){
    case(Depth8U):
    case(Depth16U):{
        _toFloat = 1.0f/std::pow(2.0f,(float)getBitsPerPixel());
        _fromFloat = std::pow(2.0f,(float)getBitsPerPixel());
    }
    break;
    case(Depth8S):
    case(Depth16S):{
        _toFloat = 1.0f/std::pow(2.0f,(float)(getBitsPerPixel()-1));
        _fromFloat = std::pow(2.0f,(float)(getBitsPerPixel()-1));
    }
    break;
    case(Depth32S):
    case(Depth32F):
    default:
        _toFloat = 1.0f;
        _fromFloat = 1.0f;
    }

    //std::cout << "toFloat   " << _toFloat << std::endl;
    //std::cout << "fromFloat " << _fromFloat << std::endl;
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
{
    _widthStepByte = _widthStep*getBitsPerPixel()/8;
    initFloatConversion();

}


Image::Image(
    unsigned int width,
    unsigned int height,
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
{
    _widthStepByte = _widthStep*getBitsPerPixel()/8;
    initFloatConversion();
}

Image::Image(
	char *imageData,
    unsigned int width,
    unsigned int height,
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
{
    _widthStepByte = _widthStep*getBitsPerPixel()/8;
    initFloatConversion();
}

size_t Image::getDataSize() const
{
    return _arrSize;
}

Pixel4f Image::getPixelf(size_t x, size_t y) const {
	const size_t idx = y*_widthStep + x*_nrChannels;

	// convert idx to point into char array
	const size_t cidx = idx*_stride;
	RW_ASSERT(cidx<_arrSize);

	// now if representation is float then we can set it directly
	if(_depth == Image::Depth32F){
		Pixel4f p(_imageData[cidx], 0, 0, 0);
		for(size_t i=1;i<_nrChannels;i++)
			p.ch[i] = _imageData[cidx+i*_stride];
		return p;
	}
	// is an int so we need to convert it to float
	Pixel4f p(( *((int*)&_imageData[cidx]) &_valueMask)*_toFloat, 0, 0, 0);
	for(size_t i=1;i<_nrChannels;i++)
		p.ch[i] = (  *((int*)&_imageData[cidx+i*_stride]) &_valueMask)*_toFloat;
	return p;
}

void Image::getPixel(size_t x, size_t y, Pixel4f& dst) const {
    const size_t idx = y*_widthStep + x*_nrChannels;

    // convert idx to point into char array
    const size_t cidx = idx*_stride;
    RW_ASSERT(cidx<_arrSize);

    // now if representation is float then we can set it directly
    if(_depth == Image::Depth32F){
        dst.ch[0] = _imageData[cidx];
        for(size_t i=1;i<_nrChannels;i++)
            dst.ch[i] = _imageData[cidx+i*_stride];
    }
    // is an int so we need to convert it to float
    dst.ch[0] = ( *((int*)&_imageData[cidx]) &_valueMask)*_toFloat;
    for(size_t i=1;i<_nrChannels;i++)
        dst.ch[i] = (  *((int*)&_imageData[cidx+i*_stride]) &_valueMask)*_toFloat;
}


Image::Pixel4i Image::getPixeli(size_t x, size_t y) const {
    const size_t idx = y*_widthStep + x*_nrChannels;

    // convert idx to point into char array
    const size_t cidx = idx*_stride;
    RW_ASSERT(cidx<_arrSize);

    // now if representation is float then we can set it directly
    if(_depth == Image::Depth32F){
        Pixel4i p((int)(_imageData[cidx]*_fromFloat), 0, 0, 0);
        for(size_t i=1;i<_nrChannels;i++)
            p.ch[i] = (int)(_imageData[cidx+i*_stride]*_fromFloat);
        return p;
    }
    // is an int so we need to convert it to float
    Pixel4i p(( *((int*)&_imageData[cidx]) &_valueMask), 0, 0, 0);
    for(size_t i=1;i<_nrChannels;i++)
        p.ch[i] = (  *((int*)&_imageData[cidx+i*_stride]) &_valueMask);
    return p;
}

void Image::getPixel(size_t x, size_t y, Pixel4i& dst) const {
    const size_t idx = y*_widthStep + x*_nrChannels;

    // convert idx to point into char array
    const size_t cidx = idx*_stride;
    RW_ASSERT(cidx<_arrSize);

    // now if representation is float then we can set it directly
    if(_depth == Image::Depth32F){
        dst.ch[0] = (int)(_imageData[cidx]*_fromFloat);
        for(size_t i=1;i<_nrChannels;i++)
            dst.ch[i] = (int)(_imageData[cidx+i*_stride] *_fromFloat);
    }
    // is an int so we need to convert it to float
    dst.ch[0] = ( *((int*)&_imageData[cidx]) &_valueMask);
    for(size_t i=1;i<_nrChannels;i++)
        dst.ch[i] = (  *((int*)&_imageData[cidx+i*_stride]) &_valueMask);
}



void Image::setPixel(size_t x, size_t y, const Pixel4f& value) {
    const size_t idx = y*_widthStep + x*_nrChannels;

    // convert idx to point into char array
    const size_t cidx = idx*_stride;
    RW_ASSERT(cidx<_arrSize);

    // now if representation is float then we can set it directly
    if(_depth == Image::Depth32F || _depth == Image::Depth32S){
        ((float*)&_imageData[cidx])[0] = value.ch[0];
        for(size_t i=1;i<_nrChannels;i++)
            ((float*)&_imageData[cidx])[i] = value.ch[i];
    } else if( _depth == Image::Depth16S || _depth == Image::Depth16U) {
        for(size_t i=0;i<_nrChannels;i++)
            ((int16_t*)&_imageData[cidx])[i] = (int16_t)(value.ch[i]*_fromFloat);
    } else if( _depth == Image::Depth8S || _depth == Image::Depth8U) {
        for(size_t i=0;i<_nrChannels;i++)
            ((int8_t*)&_imageData[cidx])[i] = (int8_t)(value.ch[i]*_fromFloat);
    }
}

float Image::getPixelValuef(size_t x, size_t y, size_t channel) const {
	const size_t idx = y*_widthStep + x*_nrChannels;

	// convert idx to point into char array
	const size_t cidx = idx*_stride;
	RW_ASSERT(cidx<_arrSize);

	if(_depth == Image::Depth32F){
        char *valuePtr = &_imageData[cidx+channel*_stride];
        return ( *((int*)valuePtr)&_valueMask)*_toFloat;
	} else {
		// is in int so we need to convert it to float
	    char *valuePtr = &_imageData[cidx+channel*_stride];
		return ( *((unsigned int*)valuePtr)&_valueMask)*_toFloat;
	}
}

int Image::getPixelValuei(size_t x, size_t y, size_t channel) const{
    const size_t idx = y*_widthStep + x*_nrChannels;

    // convert idx to point into char array
    const size_t cidx = idx*_stride;
    RW_ASSERT(cidx<_arrSize);

    if(_depth == Image::Depth32F){
        char *valuePtr = &_imageData[cidx+channel*_stride];
        return (int)  (( *((float*)valuePtr))*_fromFloat);
    } else {
        // is in int so we need to convert it to float
        char *valuePtr = &_imageData[cidx+channel*_stride];
        return (int)  (( *((int*)valuePtr)&_valueMask));
    }
}


void Image::safeDeleteData(){
    if(_imageData==NULL )
        return;
    _arrSize = 0;
    _widthStep = 0;
    _widthStepByte = 0;
    _width = 0;
    _height = 0;
    delete[] _imageData;
}

void Image::resize(unsigned int width, unsigned int height){
    if(width==_width && _height==height)
        return;
    safeDeleteData();

    _width = width;
    _height = height;
    size_t arrSize = _width * _height * _nrChannels * getBitsPerPixel() / 8;
    _imageData = new char[arrSize];
    _widthStep = _width*_nrChannels;
    _widthStepByte = _widthStep*getBitsPerPixel()/8;
}

char* Image::getImageData()
{
    return _imageData;
}

const char* Image::getImageData() const
{
    return _imageData;
}

std::pair<unsigned int, unsigned int> Image::getImageDimension()
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
            unsigned int idx = (unsigned int)y*_widthStep;
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
            unsigned int idx = (unsigned int)y*_widthStep;
            fwrite(&_imageData[idx], 1, _widthStep, imagefile);
        }

        //fwrite(&(*_imageData)[0], 1, _imageData->size(), imagefile);
        fclose(imagefile);
    } else if(getBitsPerPixel() == 16) {

        fprintf(imagefile,"P6\n");
        fprintf(imagefile,"# CREATOR: RobWork - www.robwork.dk \n");
        fprintf(imagefile,"%u %u \n", _width, _height);
        fprintf(imagefile,"65535\n");

        for(size_t y=0;y<_height;y++){
            unsigned int idx = (unsigned int)y*_widthStep;
            fwrite(&_imageData[idx], 2, _widthStep, imagefile);
        }

        //fwrite(&(*_imageData)[0], 1, _imageData->size(), imagefile);
        fclose(imagefile);

    } else {
        fclose(imagefile);
        RW_THROW("Image depth of " << getBitsPerPixel() << " is not supported!");
        return false;
    }

    return true;
}



Image::Ptr Image::copyFlip(bool horizontal, bool vertical) const {
    Image::Ptr dstImg = ownedPtr( new Image(_width,_height,_colorCode,_depth) );
    char *dstData = dstImg->getImageData();
    if(horizontal && vertical){
        for(size_t y=0;y<_height;y++){
            for(size_t x=0;x<_width;x++){
                Pixel4f val = getPixel(x,y);
                dstImg->setPixel(_width-1-x,_height-1-y,val);
            }
        }
    } else if( horizontal ){
        for(size_t y=0;y<_height;y++){
            memcpy(&dstData[(_height-1-y)*_widthStepByte], &_imageData[y*_widthStepByte], _widthStepByte);
        }
    } else if( vertical ){
        for(size_t y=0;y<_height;y++){
            for(size_t x=0;x<_width;x++){
                Pixel4f val = getPixel(x,y);
                dstImg->setPixel(_width-1-x,y,val);
            }
        }
    } else {
        memcpy(dstData, _imageData, _arrSize);
    }
    return dstImg;
}
