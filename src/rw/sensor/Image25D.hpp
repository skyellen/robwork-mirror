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

#ifndef RW_SENSOR_IMAGE25D_HPP
#define RW_SENSOR_IMAGE25D_HPP

namespace rw {
namespace sensor {

class Image25D {

public:
    /**
     * @brief default constructor
     */
    Image25D():
        _width(0),
        _height(0),
        _depthData(new std::vector<float>())
    {};

    /**
     * @brief constructor
     * @param width [in] width of the image
     * @param height [in] height of the image
     * @param encoding [in] the colorCode of this Image
     */
    Image25D(int width, int height):
        _width(width),
        _height(height),
        _depthData(new std::vector<float>())
    {}

    /**
     * @brief constructor
     * @param imgData [in] char pointer that points to an array of chars with
     * length width*height*(bitsPerPixel/8)
     * @param width [in] width of the image
     * @param height [in] height of the image
     */
    Image25D(std::vector<float> *imgData,int width,int height):
        _width(width),
        _height(height),
        _depthData(imgData)
    {}

    /**
     * @brief destructor
     *
     */
    virtual ~Image25D(){
        delete _depthData;
    }

    /**
     * @brief resizes the current image.
     * @param width
     * @param height
     */
    void resize(int width, int height){
        _width = width;
        _height = height;
        _depthData->resize(_width*_height);
    }

    /**
     * @brief returns a char pointer to the image data
     * @return char pointer to the image data
     */
    std::vector<float>& getImageData(){ return *_depthData; };

    /**
     * @brief returns a char pointer to the image data
     * @return const char pointer to the image data
     */
    const std::vector<float>& getImageData() const{ return *_depthData; };

    /**
     * @brief returns the width of this image
     * @return image width
     */
    unsigned int getWidth() const { return _width;};

    /**
     * @brief returns the height of this image
     * @return image height
     */
    unsigned int getHeight() const { return _height;};

private:
    unsigned int _width, _height;

protected:
    /**
     * @brief Float array of image data
     */
    std::vector<float> *_depthData;
};

}
}

#endif /*RW_SENSOR_IMAGE3D_HPP_*/
