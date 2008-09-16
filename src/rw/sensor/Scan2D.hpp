#ifndef SCAN2D_HPP_
#define SCAN2D_HPP_

#include "SensorData.hpp"

class Scan2D: public SensorData {
public:

private:
    std::vector<float> _angle;
    std::vector<float> _depth;

};

#endif /*SCAN2D_HPP_*/
