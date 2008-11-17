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
         * @brief default constructor
         */
        Image():
            _width(0),
            _height(0),
            _colorCode(MONO8),
            _imageData(NULL)
        {};

        /**
         * @brief constructor
         * @param width [in] width of the image
         * @param height [in] height of the image
         * @param encoding [in] the colorCode of this Image
         */
        Image(
            int width,
            int height,
            ColorCode encoding);

        /*
         * @brief constructor
         * @param imgData [in] char pointer that points to an array of chars with
         * length width*height*(bitsPerPixel/8)
         * @param width [in] width of the image
         * @param height [in] height of the image
         * @param encoding [in] the colorCode of this Image
         */
        Image(std::vector<unsigned char> *imgData,
              int width,int height,ColorCode encoding);

        /**
         * @brief destructor
         *
         */
        virtual ~Image(){
            delete _imageData;
        }

        /*
         * @brief resizes the current image.
         * @param width
         * @param height
         * @param bitsPerPixel
         */
        void resize(int width, int height, ColorCode encoding);

        /**
         * @brief returns a char pointer to the image data
         * @return char pointer to the image data
         */
        unsigned char* getImageData();

        /**
         * @brief returns a char pointer to the image data
         * @return const char pointer to the image data
         */
        const unsigned char* getImageData() const;

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
         * @brief returns the number of bits per pixel
         * @return number of bits per pixel
         */
        unsigned int getBitsPerPixel() const;

        /**
         * @brief saves this image to a file in the PGM format
         * @return true if save was succesfull, false otherwise
         */
        bool saveAsPGM(const std::string& fileName) const;



    private:
        unsigned int _width, _height;
        ColorCode _colorCode;

    protected:
        /**
         * @brief Char array of image data
         */
        std::vector<unsigned char> *_imageData;
    };

    typedef rw::common::Ptr<Image> ImagePtr;

    /* @} */
}} // end namespaces

#endif // end include guard
