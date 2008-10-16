#ifndef RW_SENSOR_SCAN2D_HPP
#define RW_SENSOR_SCAN2D_HPP

#include "SensorData.hpp"

#include <vector>

namespace rw {
namespace sensor {


class Scan2D: public SensorData {
public:
    std::vector<float> _angle;
    std::vector<float> _depth;

};

} //end namespace sensor
} //end namespace rw

#endif /*RW_SENSOR_SCAN2D_HPP*/
