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

#ifndef rw_sensor_Image_HPP
#define rw_sensor_Image_HPP

/**
 * @file Image.hpp
 */

#include <utility>
#include <string>
#include <vector>

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief The image class is a simple wrapper around a char data array.
     * This Image wrapper contain information of width, height and encoding.
     *
     * Images can be copied and assigned freely via the compiler provided copy
     * constructor and assignment operator.
     */
    class Image
    {
    public:
        //! @brief The color encodings that the image can use
        typedef enum {
            MONO8, YUV411, YUV422, YUV444, RGB8, 
            MONO16, RGB16, MONO16S, RGB16S, RAW8, 
            RAW16, RGB24
        } ColorCode;

    public:

        /**
         * @brief constructor
         * @param width [in] width of the image
         * @param height [in] height of the image
         * @param encoding [in] the colorCode of this Image
         */
        Image(
            unsigned int width,
            unsigned int height,
            ColorCode encoding);

        /*
         * @brief constructor
         * @param imgData [in] char pointer that points to an array of chars with
         * length width*height*(bitsPerPixel/8)
         * @param width [in] width of the image
         * @param height [in] height of the image
         * @param encoding [in] the colorCode of this Image
         */
        Image(std::vector<char> &imgData,
              unsigned int width,
              unsigned int height,
              ColorCode encoding);
        
        virtual ~Image(){
        	delete _imageData;
        }

        /* Not implemented.
         *
         * @brief resizes the current image.
         * @param width
         * @param height
         * @param bitsPerPixel
         */
/*        void resize(unsigned int width,
          unsigned int height,

          unsigned int bitsPerPixel);*/

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
         * @brief returns the size of the char data array
         * @return size of char data array
         */
        size_t getDataSize();

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
        ColorCode getColorEncoding()
        {
            return _colorCode;
        }

        /**
         * @brief returns the number of bits per pixel
         * @return number of bits per pixel
         */
        unsigned int getBitsPerPixel() const;

        /**
         * @brief saves this image to a file in the PGM format
         * @return true if save was succesfull, false otherwise
         */
        bool saveAsPGM(const std::string& fileName) const;

        /// for SDTV not HDTV
        /*

        friend void RGB24ToYUV422(unsigned char rgb1[3],
        unsigned char rgb2[3],
        unsigned char yuv[4]){

        }

        friend void YUV422ToRGB24(unsigned char yuv[4],unsigned char rgb1[3], unsigned char rgb2[3]){
        int u,y1,v,y2;
        u  = yuv[0];
        y1 = yuv[1];
        v  = yuv[2];
        y2 = yuv[3];

        // Conversion
        int r = y1 + 1.370705 * v;
        int g = y1 - 0.698001 * v - 0.337633 * u;
        int b = y1 + 1.732446 * u;

        // Clamp to 0..1
        if (r < 0) r = 0;
        if (g < 0) g = 0;
        if (b < 0) b = 0;
        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;

        rgb1[0] = r&0xFF;
        rgb1[1] = g&0xFF;
        rgb1[2] = b&0xFF;

        // Conversion
        r = y2 + 1.370705 * v;
        g = y2 - 0.698001 * v - 0.337633 * u;
        b = y2 + 1.732446 * u;

        // Clamp to 0..1
        if (r < 0) r = 0;
        if (g < 0) g = 0;
        if (b < 0) b = 0;
        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;

        rgb2[0] = r&0xFF;
        rgb2[1] = g&0xFF;
        rgb2[2] = b&0xFF;
        }

        */

    private:
        unsigned int _width, _height;
        ColorCode _colorCode;

        Image(){};
        
    protected:
        /**
         * @brief Char array of image data
         */
        std::vector<char> *_imageData;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
