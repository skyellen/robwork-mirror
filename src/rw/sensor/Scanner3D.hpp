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
    
};

}
}

#endif /*SCANNER3D_HPP_*/
