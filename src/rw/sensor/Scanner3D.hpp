#ifndef RW_SENSOR_SCANNER3D_HPP_
#define RW_SENSOR_SCANNER3D_HPP_

#include "Scanner.hpp"
#include "Image3D.hpp"

namespace rw {
namespace sensor {

/**
 * @brief an interface describing a 3D scanner sensor. The scanner takes
 * pictures in the oposite direction of the z-axis of the frame that it is
 * attached to. The x-y plane forms the image plane such that the xy-origin is
 * located in the bottom left corner of the image.
 *
 */
class Scanner3D: public Scanner {

protected:

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner3D(rw::kinematics::Frame* frame, const std::string& name):
        Scanner(frame, name)
    {
    }

public:
    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner3D(){};

    /**
     * @brief gets the last acquired image
     *
     */
    virtual const Image3D& getImage() = 0;

    /**
     * @brief returns the framerate that this camera is setup with
     * @return the framerate in frames per second
     */
    virtual double getFrameRate() = 0;

    /**
     * @brief sets the framerate of this camera. If the framerate is not
     * supported the closest supported framerate is choosen.
     * @param framerate [in] the framerate
     */
    virtual void setFrameRate(double framerate) = 0;

    /**
     * @brief aquires an image from the camera. This method is not blocking.
     * Use  isImageReady to poll for completion of acquire.
     */
    virtual void acquire() = 0;

    /**
     * @brief tests whether a image has been acquired
     * @return true if an image has been acquired, false otherwise.
     */
    virtual bool isImageReady() = 0;

};

}
}

#endif /*SCANNER3D_HPP_*/
