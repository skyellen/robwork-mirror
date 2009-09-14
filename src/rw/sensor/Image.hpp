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


#ifndef RW_SENSOR_IMAGE_HPP
#define RW_SENSOR_IMAGE_HPP

/**
 * @file Image.hpp
 */

#include <utility>
#include <string>
#include <vector>
#include <rw/common/Ptr.hpp>

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief The image class is a simple wrapper around a char data array.
     * This Image wrapper contain information of width, height and encoding.
     *
     * The image class is somewhat inspired by the IplImage of opencv.
     */
    class Image
    {
    public:
        /**
         * @brief The color encodings that the image can use. This also defines the number
         * channels that an image has.
         */
        typedef enum {
            GRAY, //!< Grayscale image
            RGB,  //!< 3-channel color image (Standard opengl)
            RGBA, //!< 4-channel color image with alpha channel
            BGR,  //!< 3-channel color image (Standard OpenCV)
            BGRA, //!< 4-channel color image with alpha channel
            BayerBG,
            Luv,
            Lab,
            HLS,
            User
        } ColorCode;

        /**
         * @brief The pixeldepth determines how many bits that are used per pixel per channel
         */
        typedef enum {
            Depth8U, //!< Depth8U
            Depth8S, //!< Depth8S
            Depth16U,//!< Depth16U
            Depth16S,//!< Depth16S
            Depth32S,//!< Depth32S
            Depth32F //!< Depth32F
        } PixelDepth;

    public:
        /**
         * @brief default constructor
         */
        Image();

        /**
         * @brief constructor
         * @param width [in] width of the image
         * @param height [in] height of the image
         * @param encoding [in] the colorCode of this Image
         * @param depth [in] the pixel depth in bits per channel
         */
        Image(
            int width,
            int height,
            ColorCode encoding,
            PixelDepth depth);

        /**
         * @brief constructor
         * @param imgData [in] char pointer that points to an array of chars with
         * length width*height*(bitsPerPixel/8)
         * @param width [in] width of the image
         * @param height [in] height of the image
         * @param encoding [in] the colorCode of this Image
         * @param depth [in] the pixel depth in bits per channel
         */
        Image(char *imgData,
              int width, int height,
              ColorCode encoding,
              PixelDepth depth);

        /**
         * @brief destructor
         */
        virtual ~Image(){}

        /**
         * @brief resizes the current image.
         * @param width [in] width in pixels
         * @param height [in] height in pixels
         */
        void resize(int width, int height);

        /**
         * @brief returns a char pointer to the image data
         * @return char pointer to the image data
         */
        char* getImageData();

        /**
         * @brief returns a char pointer to the image data
         * @return const char pointer to the image data
         */
        const char* getImageData() const;

        /**
         * @brief sets the data array of this image. Make sure to
         * change the height and width accordingly.
         */
        void setImageData(char* data){
            if(!_imageData)
                delete _imageData;
            _imageData = data;
        };

        /**
         * @brief returns the size of the char data array
         * @return size of char data array
         */
        size_t getDataSize() const;

        /**
         * @brief returns the dimensions (width and height) of this image
         * @return a pair of integers where first is the width and second
         * is the height
         */
        std::pair<int,int> getImageDimension();

        /**
         * @brief returns the width of this image
         * @return image width
         */
        unsigned int getWidth() const;

        /**
         * @brief returns the height of this image
         * @return image height
         */
        unsigned int getHeight() const;

        /**
         * @brief returns color encoding/type of this image
         * @return ColorCode of this image
         */
        ColorCode getColorEncoding() const
        {
            return _colorCode;
        }

        /**
         * @brief returns the number of bits per pixel. This is the number
         * of bits used per pixel per channel.
         * @return number of bits per pixel
         */
        unsigned int getBitsPerPixel() const;

        /**
         * @brief saves this image to a file in the PGM (grayscale) format
         * @param fileName [in] the name of the file that is to be created
         *
         * @return true if save was succesfull, false otherwise
         */
        bool saveAsPGM(const std::string& fileName) const;

        /**
         * @brief saves this image to a file in the ascii PGM (grayscale) format
         * @param fileName [in] the name of the file that is to be created
         * @return true if save was succesfull, false otherwise
         */
        bool saveAsPGMAscii(const std::string& fileName) const;

        /**
         * @brief saves this image to a file in the PPM (color) format
         * @param fileName [in] the name of the file that is to be created
         * @return true if save was succesfull, false otherwise
         */
        bool saveAsPPM(const std::string& fileName) const;

        /**
         * @brief the size of an aligned image row in bytes. This may not be
         * the same as the width if extra bytes are padded to each row for
         * alignment purposes.
         * @return size of aligned image row
         */
        unsigned int getWidthStep() const {return _widthStep;};

        /**
         * @brief bits per pixel encoded as a PixelDepth type.
         * @return the pixel depth
         */
        inline PixelDepth getPixelDepth() const {return _depth;};

        /**
         * @brief The number of channels that this image has.
         * @return nr of channels
         */
        inline unsigned int getNrOfChannels() const { return _nrChannels;};

    private:
        void safeDeleteData();
    private:

        unsigned int _width, _height;
        ColorCode _colorCode;
        PixelDepth _depth;
        unsigned int _nrChannels;
        unsigned int _widthStep;
    protected:

        size_t _arrSize;
        /**
         * @brief Char array of image data
         */
        char* _imageData;

    };

    typedef rw::common::Ptr<Image> ImagePtr;

    /* @} */
}} // end namespaces

#endif // end include guard
