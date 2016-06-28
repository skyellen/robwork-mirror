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



#include "RGBLoader.hpp"

#include <rw/common/macros.hpp>
#include <stdio.h>

using namespace rw::loaders;
using namespace rw::sensor;

namespace {

    #ifndef SEEK_SET
    #  define SEEK_SET 0
    #endif

    #define GLuint unsigned int
    #define GLint int
    #define GL_FALSE (0)
    #define GL_TRUE (!GL_FALSE)

    #define ALPHA_NONE   (0x0000)       /* no alpha info */
    #define ALPHA_OPAQUE (0x0001<<0)    /* alpha = 1 */
    #define ALPHA_INVIS  (0x0001<<1)    /* alpha = 0 */
    #define ALPHA_TRANSP (0x0004<<2)    /* 0 < alpha < 1 */


    /******************************************************************************/

    typedef struct ACImage_t
    {
        unsigned short width, height, depth;
        void *data;
        int index;
        char *name;
        int amask;
        char *origname; /** do not set - set automatically in texture_read function **/

    } ACImage;


    typedef struct _rawImageRec {
        unsigned short imagic;
        unsigned short type;
        unsigned short dim;
        unsigned short sizeX, sizeY, sizeZ;
        unsigned long min, max;
        unsigned long wasteBytes;
        char name[80];
        unsigned long colorMap;
        FILE *file;
        unsigned char *tmp, *tmpR, *tmpG, *tmpB, *tmpA;
        unsigned long rleEnd;
        GLuint *rowStart;
        GLint *rowSize;
    } rawImageRec;


    /******************************************************************************/

    void ConvertShort(unsigned short *array, long length)
    {
        unsigned long b1, b2;
        unsigned char *ptr;

        ptr = (unsigned char *)array;
        while (length--) {
        b1 = *ptr++;
        b2 = *ptr++;
        *array++ = (unsigned short)((b1 << 8) | (b2));
        }
    }

    void ConvertLong(GLuint *array, long length)
    {
        unsigned long b1, b2, b3, b4;
        unsigned char *ptr;

        ptr = (unsigned char *)array;
        while (length--) {
        b1 = *ptr++;
        b2 = *ptr++;
        b3 = *ptr++;
        b4 = *ptr++;
        *array++ = (b1 << 24) | (b2 << 16) | (b3 << 8) | (b4);
        }
    }

    rawImageRec *RawImageOpen(const char *fileName)
    {
        union
        {
            int testWord;
            char testByte[4];
        } endianTest;

        rawImageRec *raw;
        int swapFlag;
        int x;

        endianTest.testWord = 1;
        if (endianTest.testByte[0] == 1) {
            swapFlag = GL_TRUE;
        } else {
            swapFlag = GL_FALSE;
        }

        raw = (rawImageRec *) malloc(sizeof(rawImageRec));
        if (raw == NULL) {
            fprintf(stderr, "Out of memory!\n");
            return (NULL);
        }
        if ((raw->file = fopen(fileName, "rb")) == NULL) {
            perror(fileName);
            return (NULL);

        }

        size_t stat = fread(raw, 1, 12, raw->file);
        if (stat != 12) {
            RW_THROW("Reading error");
        }

        if (swapFlag) {
            ConvertShort(&raw->imagic, 6);
        }

        raw->tmp = (unsigned char *) malloc(raw->sizeX * 256);
        raw->tmpR = (unsigned char *) malloc(raw->sizeX * 256);
        raw->tmpG = (unsigned char *) malloc(raw->sizeX * 256);
        raw->tmpB = (unsigned char *) malloc(raw->sizeX * 256);
        raw->tmpA = (unsigned char *) malloc(raw->sizeX * 256);
        if (raw->tmp == NULL || raw->tmpR == NULL || raw->tmpG == NULL
                || raw->tmpB == NULL) {
            fprintf(stderr, "Out of memory!\n");
            return (NULL);
        }

        if ((raw->type & 0xFF00) == 0x0100) {
            x = raw->sizeY * raw->sizeZ * sizeof(GLuint);
            raw->rowStart = (GLuint *) malloc(x);
            raw->rowSize = (GLint *) malloc(x);
            if (raw->rowStart == NULL || raw->rowSize == NULL) {
                fprintf(stderr, "Out of memory!\n");
                return (NULL);
            }
            raw->rleEnd = 512 + (2 * x);
            fseek(raw->file, 512, SEEK_SET);
            stat = fread(raw->rowStart, 1, x, raw->file);
            stat = fread(raw->rowSize, 1, x, raw->file);
            if (swapFlag) {
                ConvertLong(raw->rowStart, x / sizeof(GLuint));
                ConvertLong((GLuint *) raw->rowSize, x / sizeof(GLint));
            }
        }
        return raw;
    }

    void RawImageClose(rawImageRec *raw)
    {

        fclose(raw->file);
        free(raw->tmp);
        free(raw->tmpR);
        free(raw->tmpG);
        free(raw->tmpB);
        free(raw);
    }

    void RawImageGetRow(rawImageRec *raw, unsigned char *buf, int y,
                                int z)
    {
        unsigned char *iPtr, *oPtr, pixel;
        int count;
        size_t stat;
        if ((raw->type & 0xFF00) == 0x0100) {
            fseek(raw->file, raw->rowStart[y + z * raw->sizeY], SEEK_SET);
            stat = fread(raw->tmp, 1, (unsigned int) raw->rowSize[y + z * raw->sizeY],
                  raw->file);
            if (stat != (size_t)raw->rowSize[y + z * raw->sizeY]) {RW_THROW("Reading error");}

            iPtr = raw->tmp;
            oPtr = buf;
            while (1) {
                pixel = *iPtr++;
                count = (int) (pixel & 0x7F);
                if (!count) {
                    return;
                }
                if (pixel & 0x80) {
                    while (count--) {
                        *oPtr++ = *iPtr++;
                    }
                } else {
                    pixel = *iPtr++;
                    while (count--) {
                        *oPtr++ = pixel;
                    }
                }
            }
        } else {
            fseek(raw->file, 512 + (y * raw->sizeX) + (z * raw->sizeX
                    * raw->sizeY), SEEK_SET);
            stat = fread(buf, 1, raw->sizeX, raw->file);
            if (stat != raw->sizeX) {RW_THROW("Reading error");}
        }
    }

    void RawImageGetData(rawImageRec *raw, ACImage *final)
    {
        unsigned char *ptr;
        int i, j;

        final->data = (unsigned char *) malloc((raw->sizeX + 1) * (raw->sizeY+1) * raw->sizeZ);
        if (final->data == NULL) {
            fprintf(stderr, "Out of memory!\n");
            return;
        }

        ptr = (unsigned char *) final->data;

        /*
         debugf("raw image depth %d", raw->sizeZ);
         */
        if (raw->sizeZ == 1) {
            for (i = 0; i < raw->sizeY; i++) {
                RawImageGetRow(raw, raw->tmpR, i, 0);
                for (j = 0; j < raw->sizeX; j++) { /* packing */
                    *ptr++ = *(raw->tmpR + j);
                    /**ptr++ = *(raw->tmpR + j);
                     *ptr++ = *(raw->tmpR + j);
                     *ptr++ = 255;*/
                }
            }
        }
        if (raw->sizeZ == 2) {
            for (i = 0; i < raw->sizeY; i++) {
                RawImageGetRow(raw, raw->tmpR, i, 0);
                RawImageGetRow(raw, raw->tmpA, i, 1);
                for (j = 0; j < raw->sizeX; j++) { /* packing */
                    *ptr++ = *(raw->tmpR + j);
                    /**ptr++ = *(raw->tmpR + j);
                     *ptr++ = *(raw->tmpR + j);*/
                    *ptr++ = *(raw->tmpA + j);

                    final->amask |= ((*(raw->tmpA + j) == 255) ? ALPHA_OPAQUE
                            : 0);
                    final->amask |= ((*(raw->tmpA + j) == 0) ? ALPHA_INVIS : 0);
                    final->amask |= (((*(raw->tmpA + j) > 0) && (*(raw->tmpA
                            + j) < 255)) ? ALPHA_TRANSP : 0);
                }
            }
        } else if (raw->sizeZ == 3) {
            for (i = 0; i < raw->sizeY; i++) {
                RawImageGetRow(raw, raw->tmpR, i, 0);
                RawImageGetRow(raw, raw->tmpG, i, 1);
                RawImageGetRow(raw, raw->tmpB, i, 2);
                for (j = 0; j < raw->sizeX; j++) { /* packing */
                    *ptr++ = *(raw->tmpR + j);
                    *ptr++ = *(raw->tmpG + j);
                    *ptr++ = *(raw->tmpB + j);
                    /**ptr++ = 255;*/
                }
            }
        } else if (raw->sizeZ == 4) {
            for (i = 0; i < raw->sizeY; i++) {
                RawImageGetRow(raw, raw->tmpR, i, 0);
                RawImageGetRow(raw, raw->tmpG, i, 1);
                RawImageGetRow(raw, raw->tmpB, i, 2);
                RawImageGetRow(raw, raw->tmpA, i, 3);
                for (j = 0; j < raw->sizeX; j++) { /* packing */
                    *ptr++ = *(raw->tmpR + j);
                    *ptr++ = *(raw->tmpG + j);
                    *ptr++ = *(raw->tmpB + j);
                    *ptr++ = *(raw->tmpA + j);

                    final->amask |= ((*(raw->tmpA + j) == 255) ? ALPHA_OPAQUE
                            : 0);
                    final->amask |= ((*(raw->tmpA + j) == 0) ? ALPHA_INVIS : 0);
                    final->amask |= (((*(raw->tmpA + j) > 0) && (*(raw->tmpA
                            + j) < 255)) ? ALPHA_TRANSP : 0);
                }
            }
        }
    }

}

namespace {
/*
    template<class SRCTYPE, class DSTTYPE>
    class PixelAccessor {

        DSTTYPE get(unsigned int idx);

        DSTTYPE get(unsigned int x, unsigned int y);

        SRCTYPE* getRow(unsigned int y);

        void set(DSTTYPE)
    };
*/
}

rw::sensor::Image::Ptr RGBLoader::loadImage(const std::string& fname){
    return RGBLoader::load(fname);
}

std::vector<std::string> RGBLoader::getImageFormats() {
	std::vector<std::string> formats;
	formats.push_back("RGB");
	return formats;
}

rw::sensor::Image::Ptr RGBLoader::load(const std::string& fname){
    const char *fileName = fname.c_str();
    rawImageRec *raw;
	Image::Ptr img;
    ACImage *final = new ACImage();

    //printf("Loading texture: %s\n", fileName);

    raw = RawImageOpen(fileName);
    if (raw == NULL)
    {
        fprintf(stderr, "error opening rgb file\n");
        return img;
    }

    final->width = raw->sizeX;
    final->height = raw->sizeY;
    final->depth = raw->sizeZ;

    RawImageGetData(raw, final);
    RawImageClose(raw);

    //printf("loaded RGB image %dx%d (%d)\n", final->width, final->height, final->depth);

    Image::PixelDepth pdepth = Image::Depth8U;
    Image::ColorCode ccode = Image::RGB;
    if(final->depth==3){
        ccode = Image::RGB;
    }else if(final->depth==4){
        ccode = Image::RGBA;
    }else if(final->depth==1){
        // R
    }else if(final->depth==2){
        // RA
    } else {
        RW_ASSERT(0);
    }

    //std::cout << "Copy data to the image" << std::endl;
    //std::cout << "final->width: " << final->width << std::endl;
    //std::cout << "final->height: " << final->height << std::endl;
    //std::cout << "final->depth: " << final->depth << std::endl;

    unsigned int widthStep = final->width * final->depth;

    img = new Image(final->width, final->height, ccode, pdepth);
    char *imgData = img->getImageData();
    for(size_t y=0;y<final->height; y++){
        unsigned int rowidx = (int)y*widthStep;
        unsigned char *dstrow = (unsigned char *) &(imgData[rowidx]);
        unsigned char *srcrow = &(((unsigned char*)final->data)[rowidx]);
        for(int x=0;x<final->width*final->depth; x++){
            dstrow[x] = srcrow[x];
        }
    }
    return img;
}
