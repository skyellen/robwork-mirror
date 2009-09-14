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


#include "ImageUtil.hpp"

#include <limits.h>

using namespace rw::sensor;

namespace {

    // only works for integer images
    double calcScale(const Image& src, const Image &dst){
        double maxValSrc = 1<<src.getBitsPerPixel();
        double maxValDst = 1<<dst.getBitsPerPixel();
        // dst = src[idx]*maxDst/maxSrc
        return maxValDst/maxValSrc;
    }

    /**
     * @brief copies one image \b src into another image dst. Both
     * images must have same nr of channels but may differ in depth.
     * @param src [in] source image
     * @param dst [in] destination image
     */
    template<class SRCTYPE, class DSTTYPE>
    void copy(const Image& src, Image &dst, double scale){
        RW_ASSERT(src.getNrOfChannels()==dst.getNrOfChannels());

        unsigned int nrChannels = src.getNrOfChannels();
        const unsigned char *srcData = src.getImageData();
        unsigned int srcWidthStep = src.getWidthStep();

        DSTTYPE* dstData = (DSTTYPE*)dst.getImageData();
        unsigned int dstWidthStep = dst.getWidthStep();

        if(srcWidthStep==dstWidthStep){
            for(int y=0;y<src.getHeight();y++){
                unsigned int idx = y*srcWidthStep;
                SRCTYPE *srcDataRow = (SRCTYPE*)&srcData[idx];
                DSTTYPE *dstDataRow = (DSTTYPE*)&dstData[idx];
                for(int x=0;x<src.getWidth()*nrChannels;x++){
                    dstData[x] = scale * (DSTTYPE) (srcDataRow[x]);
                }
            }
        } else {
            for(int y=0;y<src.getHeight();y++){
                SRCTYPE *srcDataRow = (SRCTYPE*)&srcData[y*srcWidthStep];
                DSTTYPE *dstDataRow = (DSTTYPE*)&dstData[y*dstWidthStep];
                for(int x=0;x<src.getWidth()*nrChannels;x++){
                    dstData[x] = scale * (DSTTYPE) (srcDataRow[x]);
                }
            }
        }
    }

    template<class SRCTYPE, class DSTTYPE>
    void copy(const Image& src, Image &dst){
        // calculate scale from nr of pixels per image
        if(src.getBitsPerPixel() != dst.getBitsPerPixel()){
            copy<SRCTYPE,DSTTYPE>(src,dst,calcScale(src,dst));
        } else {
            RW_ASSERT(src.getNrOfChannels()==dst.getNrOfChannels());

            unsigned int nrChannels = src.getNrOfChannels();
            const unsigned char *srcData = src.getImageData();
            unsigned int srcWidthStep = src.getWidthStep();

            DSTTYPE* dstData = (DSTTYPE*)dst.getImageData();
            unsigned int dstWidthStep = dst.getWidthStep();

            if(srcWidthStep==dstWidthStep){
                for(int y=0;y<src.getHeight();y++){
                    unsigned int idx = y*srcWidthStep;
                    SRCTYPE *srcDataRow = (SRCTYPE*)&srcData[idx];
                    DSTTYPE *dstDataRow = (DSTTYPE*)&dstData[idx];
                    for(int x=0;x<src.getWidth()*nrChannels;x++){
                        dstData[x] = (DSTTYPE) (srcDataRow[x]);
                    }
                }
            } else {
                for(int y=0;y<src.getHeight();y++){
                    SRCTYPE *srcDataRow = (SRCTYPE*)&srcData[y*srcWidthStep];
                    DSTTYPE *dstDataRow = (DSTTYPE*)&dstData[y*dstWidthStep];
                    for(int x=0;x<src.getWidth()*nrChannels;x++){
                        dstData[x] = (DSTTYPE) (srcDataRow[x]);
                    }
                }
            }
        }
    }

    template<class SRCTYPE, class DSTTYPE>
    void convertRGB2GRAY(const Image& src, Image& dst, float w[3]){
        unsigned int nrChannels = src.getNrOfChannels();
        double scale = calcScale(src, dst);
        const char *srcData = src.getImageData();
        //unsigned int srcWidthStep = src.getWidthStep();
        unsigned int srcWidthStep = src.getWidth()*src.getNrOfChannels();//dst.getWidthStep();

        DSTTYPE* dstData = (DSTTYPE*)dst.getImageData();
        unsigned int dstWidthStep = dst.getWidth()*dst.getNrOfChannels();//dst.getWidthStep();

        //std::cout << "src.getHeight(): " << src.getHeight() << std::endl;
        //std::cout << "src.getWidth(): " << src.getWidth() << std::endl;
        //std::cout << "srcWidthStep: " << srcWidthStep << std::endl;
        //std::cout << "dst.getHeight(): " << dst.getHeight() << std::endl;
        //std::cout << "dst.getWidth(): " << dst.getWidth() << std::endl;
        //std::cout << "dstWidthStep: " << dstWidthStep << std::endl;

        for(size_t y=0;y<src.getHeight();y++){
            //std::cout << "y: " << y << std::endl;

            SRCTYPE *srcDataRow = (SRCTYPE*)&srcData[y*srcWidthStep];
            DSTTYPE *dstDataRow = (DSTTYPE*)&dstData[y*dstWidthStep];
            for(size_t x=0, x_gray=0;x<src.getWidth()*nrChannels;x+=nrChannels,x_gray++){
                //std::cout << "x_gray: " << x_gray << std::endl;
                //std::cout << "x: " << x << std::endl;
                const SRCTYPE r = srcDataRow[x+0];
                const SRCTYPE g = srcDataRow[x+1];
                const SRCTYPE b = srcDataRow[x+2];
                const DSTTYPE luma = (DSTTYPE)(scale*(r*w[0] + g*w[1]+ b*w[2]));
                dstDataRow[x_gray] = luma;
            }
        }
    }

    template<class SRCTYPE>
    void convertRGB2GRAY(const Image& src, Image& dst, float weights[3]){
        //std::cout << "dst.getPixelDepth()" << dst.getPixelDepth() << std::endl;
        switch(dst.getPixelDepth()){
        case(Image::Depth8U):
            convertRGB2GRAY<SRCTYPE, unsigned char>(src, dst, weights); break;
        case(Image::Depth8S):
            RW_ASSERT(0);
        case(Image::Depth16U):
            convertRGB2GRAY<SRCTYPE,unsigned short>(src, dst, weights); break;
        case(Image::Depth16S):
            RW_ASSERT(0);
        case(Image::Depth32S):
            RW_ASSERT(0);
        case(Image::Depth32F):
            RW_ASSERT(0);
            //convertRGB2GRAY<SRCTYPE,float>(src, dst, weights); break;
        default:
            RW_ASSERT(0);
        }
    }

}

void ImageUtil::RGB2GRAY(const Image& src, Image& dst){
    if( src.getColorEncoding() != Image::RGB )
        RW_THROW("Source image is not an RGB image!");
    if( dst.getColorEncoding() != Image::GRAY )
        RW_THROW("Destination image is not a GRAY image!");

    // initialize dst if its not in the right size and format
    //std::cout << "Check resize" << std::endl;
    if(dst.getWidth()!=src.getWidth() || dst.getHeight()!= src.getHeight()){
        dst.resize( src.getWidth(), src.getHeight() );
    }

    float weights[3];
    weights[0] = 0.3;
    weights[1] = 0.59;
    weights[2] = 0.11;

    //std::cout << "conv" << src.getPixelDepth() << std::endl;
    switch(src.getPixelDepth()){
    case(Image::Depth8U):
        convertRGB2GRAY<unsigned char>(src, dst, weights); break;
    case(Image::Depth8S):
        RW_ASSERT(0);
    case(Image::Depth16U):
        convertRGB2GRAY<unsigned short>(src, dst, weights); break;
    case(Image::Depth16S):
        RW_ASSERT(0);
    case(Image::Depth32S):
        RW_ASSERT(0);
    case(Image::Depth32F):
        RW_ASSERT(0);
        //convertRGB2GRAY<SRCTYPE,float>(src, dst, weights); break;
    default:
        RW_ASSERT(0);
    }
}

void ImageUtil::reset(Image& src, int color){
	char* srcData = src.getImageData();
    for(size_t i=0; i<src.getDataSize();i++){
        srcData[i] = color;
    }
}


void ImageUtil::flipY(Image& img){
    // the image is mirrored in the Y-axis
    int nrOfChannels = img.getNrOfChannels();
    int width = img.getWidth();
    int height = img.getHeight();
    unsigned char *data = (unsigned char*)img.getImageData();

    // this actually only works for images with depth 8
    for(int y=0;y<height;y++){
        for(int x=0;x<width/2;x++){
            for(int c=0;c<nrOfChannels;c++){
                int idx = (y*width+x)*nrOfChannels;
                int idxback = (y*width+width-1-x)*nrOfChannels;
                unsigned char tmp = data[idx+c];
                data[idx+c] = data[idxback+c];
                data[idxback+c] = tmp;
            }
        }
    }
}

void ImageUtil::flipX(Image& img){
    // the image is mirrored in the x-axis
    int nrOfChannels = img.getNrOfChannels();
    int width = img.getWidth();
    int height = img.getHeight();
    unsigned char *data = (unsigned char*)img.getImageData();

    // this actually only works for images with depth 8
    for(int y=0;y<height/2;y++){
        for(int x=0;x<width;x++){
            for(int c=0;c<nrOfChannels;c++){
                int idx = (y*width+x)*nrOfChannels;
                int idxback = ((height-1-y)*width+x)*nrOfChannels;
                unsigned char tmp = data[idx+c];
                data[idx+c] = data[idxback+c];
                data[idxback+c] = tmp;
            }
        }
    }
}

