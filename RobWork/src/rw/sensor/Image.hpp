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
 * @file rw/sensor/Image.hpp
 *
 * @copydoc rw::sensor::Image
 */

#include <string>
#include <rw/common/Ptr.hpp>
#include <boost/mpl/equal_to.hpp>
#include <boost/mpl/int.hpp>
#include <rw/common/types.hpp>
#include <rw/common/macros.hpp>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/* @{ */

namespace mpl {
	using namespace boost::mpl;
}


/**
 * @brief
 */
struct Pixel4f
{
    Pixel4f(float v0, float v1, float v2, float v3)
    {
        ch[0] = v0;
        ch[1] = v1;
        ch[2] = v2;
        ch[3] = v3;
    }

    float ch[4]; //! up to four channels
};

/**
 * @brief The image class is a simple wrapper around a char data array.
 * This Image wrapper contain information of width, height and encoding.
 *
 * The image class is somewhat inspired by the IplImage of opencv.
 *
 * The coordinate system has its origin located at the top-left position, where from X increases to
 * the left and Y-increases downwards.
 *
 * setting pixel values in an efficient manner has been enabled using some template joggling.
 * It requires that the user know what type of image he/she is working with.
 */
class Image
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Image> Ptr;

	//! @brief
	struct Pixel4i
	{
	    Pixel4i(int v0, int v1, int v2, int v3)
	    {
	        ch[0] = v0;
	        ch[1] = v1;
	        ch[2] = v2;
	        ch[3] = v3;
	    }

	    int ch[4]; //! up to four channels
	};

    /**
     * @brief The color encodings that the image can use. This also defines the number
     * channels that an image has.
     */
    typedef enum
    {
        GRAY, //!< Grayscale image 1-channel
        RGB, //!< 3-channel color image (Standard opengl)
        RGBA, //!< 4-channel color image with alpha channel
        BGR, //!< 3-channel color image (Standard OpenCV)
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
    typedef enum
    {
        Depth8U, //!< Depth8U
        Depth8S, //!< Depth8S
        Depth16U,//!< Depth16U
        Depth16S,//!< Depth16S
        Depth32S,//!< Depth32S
        Depth32F
    //!< Depth32F
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
    Image(unsigned int width, unsigned int height, ColorCode encoding, PixelDepth depth);

    /**
     * @brief constructor
     * @param imgData [in] char pointer that points to an array of chars with
     * length width*height*(bitsPerPixel/8)
     * @param width [in] width of the image
     * @param height [in] height of the image
     * @param encoding [in] the colorCode of this Image
     * @param depth [in] the pixel depth in bits per channel
     */
    Image(char *imgData, unsigned int width, unsigned int height, ColorCode encoding, PixelDepth depth);

    /**
     * @brief destructor
     */
    virtual ~Image()
    {
        safeDeleteData();
    }
    ;

    /**
     * @brief resizes the current image.
     * @param width [in] width in pixels
     * @param height [in] height in pixels
     */
    void resize(unsigned int width, unsigned int height);

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
    void setImageData(char* data)
    {
        if (!_imageData) delete _imageData;
        _imageData = data;
    }
    ;

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
    std::pair<unsigned int, unsigned int> getImageDimension();

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
    unsigned int getWidthStep() const
    {
        return _widthStepByte;
    }
    ;

    /**
     * @brief bits per pixel encoded as a PixelDepth type.
     * @return the pixel depth
     */
    inline PixelDepth getPixelDepth() const
    {
        return _depth;
    }
    ;

    /**
     * @brief The number of channels that this image has.
     * @return nr of channels
     */
    inline unsigned int getNrOfChannels() const
    {
        return _nrChannels;
    }
    ;

    // Here comes all the getPixel operations

    /**
     * @brief generic but inefficient access to pixel information. The float
     * value is between [0;1] which means non float images are scaled according to
     * their pixel depth (bits per pixel).
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @return up to 4 pixels (depends on nr of channels) in a float format
     */
    Pixel4f getPixel(size_t x, size_t y) const{ return getPixelf(x,y); };
    Pixel4f getPixelf(size_t x, size_t y) const;

    /**
     * @brief generic but inefficient access to pixel information. The float
     * value is between [0;1] which means non float images are scaled according to
     * their pixel depth (bits per pixel).
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @param dst [out] up to 4 pixels (depends on nr of channels) in a float format
     */
    void getPixel(size_t x, size_t y, Pixel4f& dst) const;

    /**
     * @brief generic access to pixel information, however user must take care of the pixel
     * depth himself. If image is a Depth8U then the maximum value is 254. Also float images
     * are scaled accordingly.
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @return up to 4 pixels (depends on nr of channels) as ints
     */
    Pixel4i getPixeli(size_t x, size_t y) const;

    /**
     * @brief generic access to pixel information, however user must take care of the pixel
     * depth himself. If image is a Depth8U then the maximum value is 254. Also float images
     * are scaled accordingly.
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @param dst [out] up to 4 pixels (depends on nr of channels) in a float format
     */
    void getPixel(size_t x, size_t y, Pixel4i& dst) const;

    /**
     * @brief generic but inefficient access to a specific channel of
     * a pixel.
     * @param x [in]
     * @param y [in]
     * @return
     */
    float getPixelValue(size_t x, size_t y, size_t channel) const{ return getPixelValuef(x,y,channel); };
    float getPixelValuef(size_t x, size_t y, size_t channel) const;

    int getPixelValuei(size_t x, size_t y, size_t channel) const;
    template<typename T>
    void getPixelValue(size_t x, size_t y, size_t channel, T &dst) const{
        const size_t idx = y*_widthStep + x*_nrChannels;

        // convert idx to point into char array
        const size_t cidx = idx*_stride;

        dst = *((T*) &_imageData[cidx+channel*_stride] );
    }


    // Here comes all the setPixel operations
    void setPixel(size_t x, size_t y, const Pixel4f& value);


    /**
     * @brief sets the gray tone in a 1-channel gray tone image with
     * @param x
     * @param y
     * @param value
     */
    inline void setPixel8U(const int x, const int y, uint8_t value){
        const size_t idx = y*_widthStep + x;
        _imageData[idx] = value;
    }

    inline void setPixel8U(const int x, const int y, uint8_t ch0, uint8_t ch1, uint8_t ch2){
        const size_t idx = y*_widthStep + x*3;
        RW_ASSERT_MSG(idx<_arrSize, idx << "<" << _arrSize);
        _imageData[idx] = ch0;
        _imageData[idx+1] = ch1;
        _imageData[idx+2] = ch2;
    }

    inline void setPixel8U(const int x, const int y, uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3){
        const size_t idx = y*_widthStep + x*4;
        _imageData[idx] = ch0;
        _imageData[idx+1] = ch1;
        _imageData[idx+2] = ch2;
        _imageData[idx+3] = ch3;
    }

    inline void setPixel8S(const int x, const int y, int8_t value){
        const size_t idx = y*_widthStep + x;
        _imageData[idx] = value;
    }

    inline void setPixel8S(const int x, const int y, int8_t ch0, int8_t ch1, int8_t ch2){
        const size_t idx = y*_widthStep + x*3;
        _imageData[idx] = ch0;
        _imageData[idx+1] = ch1;
        _imageData[idx+2] = ch2;
    }

    inline void setPixel8S(const int x, const int y, int8_t ch0, int8_t ch1, int8_t ch2, int8_t ch3){
        const size_t idx = y*_widthStep + x*4;
        _imageData[idx] = ch0;
        _imageData[idx+1] = ch1;
        _imageData[idx+2] = ch2;
        _imageData[idx+3] = ch3;
    }

    inline void setPixel16U(const int x, const int y, uint16_t value){
        const size_t idx = y*_widthStep + x;
        uint16_t *ref = (uint16_t*)&_imageData[idx*2];
        *ref = value;
    }

    inline void setPixel16U(const int x, const int y, uint16_t ch0, uint16_t ch1, uint16_t ch2){
        const size_t idx = y*_widthStep + x*3;
        uint16_t *ref = (uint16_t*)&_imageData[idx*2];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
    }

    inline void setPixel16U(const int x, const int y, uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3){
        const size_t idx = y*_widthStep + x*4;
        uint16_t *ref = (uint16_t*)&_imageData[idx*2];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
        ref[3] = ch3;
    }

    inline void setPixel16S(const int x, const int y, int16_t value){
        const size_t idx = y*_widthStep + x;
        int16_t *ref = (int16_t*)&_imageData[idx*2];
        *ref = value;
    }

    inline void setPixel16S(const int x, const int y, int16_t ch0, int16_t ch1, int16_t ch2){
        const size_t idx = y*_widthStep + x*3;
        int16_t *ref = (int16_t*)&_imageData[idx*2];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
    }

    inline void setPixel16S(const int x, const int y, int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3){
        const size_t idx = y*_widthStep + x*4;
        int16_t *ref = (int16_t*)&_imageData[idx*2];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
        ref[3] = ch3;
    }

    inline void setPixel32S(const int x, const int y, int32_t value){
        const size_t idx = y*_widthStep + x;
        int32_t *ref = (int32_t*)&_imageData[idx*4];
        *ref = value;
    }

    inline void setPixel32S(const int x, const int y, int32_t ch0, int32_t ch1, int32_t ch2){
        const size_t idx = y*_widthStep + x*3;
        int32_t *ref = (int32_t*)&_imageData[idx*4];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
    }

    inline void setPixel32S(const int x, const int y, int32_t ch0, int32_t ch1, int32_t ch2, int32_t ch3){
        const size_t idx = y*_widthStep + x*4;
        int32_t *ref = (int32_t*)&_imageData[idx*4];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
        ref[3] = ch3;
    }

    inline void setPixel32F(const int x, const int y, float value){
        const size_t idx = y*_widthStep + x;
        float *ref = (float*)&_imageData[idx*4];
        *ref = value;
    }

    inline void setPixel32F(const int x, const int y, float ch0, float ch1, float ch2){
        const size_t idx = y*_widthStep + x*3;
        float *ref = (float*)&_imageData[idx*4];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
    }

    inline void setPixel32F(const int x, const int y, float ch0, float ch1, float ch2, float ch3){
        const size_t idx = y*_widthStep + x*4;
        float *ref = (float*)&_imageData[idx*4];
        ref[0] = ch0;
        ref[1] = ch1;
        ref[2] = ch2;
        ref[3] = ch3;
    }

    template <PixelDepth DT>
    inline void setPixel(const int x, const int y, int value){
		if( mpl::equal_to<mpl::int_<DT>, mpl::int_<Depth8U> >::value ){
            setPixel8U(x, y, (uint8_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8S> >::value ){
            setPixel8S(x, y, (int8_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16U> >::value ){
            setPixel16U(x, y, (uint16_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16S> >::value ){
            setPixel16S(x, y, (int16_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32S> >::value ){
            setPixel32S(x, y, (int32_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32F> >::value ){
            setPixel32F(x, y, (float)value);
        } else {
            RW_THROW("Unknown pixel depth!");
        }
    }

    template <PixelDepth DT>
    inline void setPixel(const int x, const int y, float value){
        // these boost ifs should be discarded by compiler when the template argument is known        
		if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8U> >::value ){
            setPixel8U(x, y, (uint8_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8S> >::value ){
            setPixel8S(x, y, (int8_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16U> >::value ){
            setPixel16U(x, y, (uint16_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16S> >::value ){
            setPixel16S(x, y, (int16_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32S> >::value ){
            setPixel32S(x, y, (int32_t)value);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32F> >::value ){
            setPixel32F(x, y, value);
        } else {
            RW_THROW("Unknown pixel depth!");
        }
    }

    // and now for the multi-channel formats
    template <PixelDepth DT>
    inline void setPixel(const int x, const int y, const int ch0, const int ch1, const int ch2){
        // these boost ifs should be discarded by compiler when the template argument is known
        if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8U> >::value ){
            setPixel8U(x, y, (uint8_t)ch0, (uint8_t)ch1, (uint8_t)ch2);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8S> >::value ){
            setPixel8S(x, y, (int8_t)ch0, (int8_t)ch1, (int8_t)ch2);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16U> >::value ){
            setPixel16U(x, y, (uint16_t)ch0, (uint16_t)ch1, (uint16_t)ch2);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16S> >::value ){
            setPixel16S(x, y, (int16_t)ch0, (int16_t)ch1, (int16_t)ch2);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32S> >::value ){
            setPixel32S(x, y, (int32_t)ch0, (int32_t)ch1, (int32_t)ch2);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32F> >::value ){
            setPixel32F(x, y, (float)ch0, (float)ch1, (float)ch2);
        } else {
            RW_THROW("Unknown pixel depth!");
        }
    }

    template <PixelDepth DT>
    inline void setPixel(const int x, const int y, const int ch0, const int ch1, const int ch2, const int ch3){
        // these boost ifs should be discarded by compiler when the template argument is known
        if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8U> >::value ){
            setPixel8U(x, y, (uint8_t)ch0, (uint8_t)ch1, (uint8_t)ch2, (uint8_t)ch3);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth8S> >::value ){
            setPixel8S(x, y, (int8_t)ch0, (int8_t)ch1, (int8_t)ch2, (int8_t)ch3);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16U> >::value ){
            setPixel16U(x, y, (uint16_t)ch0, (uint16_t)ch1, (uint16_t)ch2, (uint16_t)ch3);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth16S> >::value ){
            setPixel16S(x, y, (int16_t)ch0, (int16_t)ch1, (int16_t)ch2, (int16_t)ch3);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32S> >::value ){
            setPixel32S(x, y, (int32_t)ch0, (int32_t)ch1, (int32_t)ch2, (int32_t)ch3);
        } else if( boost::mpl::equal_to<mpl::int_<DT>, boost::mpl::int_<Depth32F> >::value ){
            setPixel32F(x, y, (float)ch0, (float)ch1, (float)ch2, (float)ch3);
        } else {
            RW_THROW("Unknown pixel depth!");
        }
    }

    /**
     * @brief copies this image and flips it around horizontal or vertical axis or both.
     * @return
     */
    Image::Ptr copyFlip(bool horizontal, bool vertical) const ;

private:
    void safeDeleteData();
    void initFloatConversion();
private:

    unsigned int _width, _height;
    ColorCode _colorCode;
    PixelDepth _depth;
    unsigned int _nrChannels;
    unsigned int _widthStep;//! width of a row in channels eg. _width*_nrChannels
    unsigned int _widthStepByte;//! width of a row in bytes, _width*_nrChannels*(bytesPerChannel)


protected:

    size_t _arrSize;
    /**
     * @brief Char array of image data
     */
    char* _imageData;

    size_t _stride; //! the stride of a pixel value
    unsigned int _valueMask; //! true if float representation is used
    float _toFloat, _fromFloat;
    //bool _isFloat; //! true if float representation is used

};

#ifdef RW_USE_DEPRECATED
typedef rw::common::Ptr<Image> ImagePtr;
#endif
/* @} */
}
} // end namespaces

#endif // end include guard
