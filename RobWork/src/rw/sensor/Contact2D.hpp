#ifndef RW_SENSOR_CONTACT2D_HPP_
#define RW_SENSOR_CONTACT2D_HPP_

#include <rw/math/Vector2D.hpp>

namespace rw{
namespace sensor {

class Contact2D {
public:
    rw::math::Vector2D<> p; // Contact position
    rw::math::Vector2D<> n; // Surface contact normal
    double curvature; // surface curvature
    double avgCurvature; // double moving average of the curvature
    double mu; // coulomb friction coefficient
};

}
}



#endif /*CONTACT_HPP_*/
