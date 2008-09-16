#ifndef RW_SENSOR_SCANNER_HPP_
#define RW_SENSOR_SCANNER_HPP_

#include "Sensor.hpp"
#include <rw/kinematics/State.hpp>

namespace rw {
namespace sensor {

class Scanner: public Sensor {
protected:
    
    Scanner(rw::kinematics::Frame* frame, const std::string& name):
        Sensor(frame,name)
    {
    }
    
    
public:
    /**
     * @brief Opens connection to the scanner
     */
    virtual void open() = 0;
    
    /**
     * @brief Returns whether the scanner has been opened
     *
     * @return true if camera is opened
     */
    virtual bool isOpen() = 0;
    
    /**   
     * @brief Closes the connection to the scanner
     */
    virtual void close() = 0;
    
    /**
     * @brief Acquires data
     */
    virtual void acquire( const rw::kinematics::State& state ) = 0;
        
    /**
     * @brief tests whether an image has been acquired
     * @return true if an image has been acquired, false otherwise.
     */
    virtual bool isImageReady() = 0;
    
    /**
     * @brief Returns the min and max range of this Scanner3D
     * @return min and max range
     */    
    virtual std::pair<double,double> getRange() = 0;

    /**
     * @brief returns the framerate that this camera is setup with
     * @return the framerate in frames per second
     */
    virtual double getFrameRate() = 0;
    
};

}
}

#endif /*SCANNER_HPP_*/
