#ifndef RW_SENSOR_SCANNER2D_HPP_
#define RW_SENSOR_SCANNER2D_HPP_

#include "Scanner.hpp"

#include "Scan2D.hpp"

namespace rw {
namespace sensor {

/**
 * @brief The Scanner2D sensor encapsulate the basic interface of a
 * 2 dimensional range scanning device.
 */

class Scanner2D: public Scanner {

protected:

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner2D(rw::kinematics::Frame* frame, const std::string& name):
        Scanner(frame, name)
    {
    }

public:
    /**
     * @brief destructor
     */
    virtual ~Scanner2D();

    /**
     * @brief
     */
    virtual bool acquire() = 0;

    /**
     * @brief gets the last acquired scan
     *
     */
    virtual const Scan2D& getData() = 0;


    virtual std::pair<double,double> getRange();

    /**
     * @brief gets the scanning resolution in radians
     * @return
     */
    virtual double getResolution();

};

}
}

#endif /*SCANNER2D_HPP_*/
