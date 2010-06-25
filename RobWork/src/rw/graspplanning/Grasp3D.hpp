#ifndef GRASP3D_HPP_
#define GRASP3D_HPP_

#include <rw/sensor/Contact3D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace graspplanning {


/**
 * @brief a grasp is a set of contacts between the object to be grasped and
 * the robot gripper.
 */

class Grasp3D {
public:

    Grasp3D(int nrOfContacts=1):
        contacts(nrOfContacts),
        approach(nrOfContacts)
    {}

    Grasp3D(const std::vector<rw::sensor::Contact3D> cons):
        contacts(cons),
        approach(cons.size())
    {}

    void scale(double clerance){
        for(size_t i=0;i<contacts.size();i++)
            contacts[i].p += normalize(approach[i])*clerance;
    }

    double phi,psi,quality;
    std::vector<rw::sensor::Contact3D> contacts;
    std::vector<rw::math::Vector3D<> > approach;
    rw::math::Vector3D<> center;


};
}
}
#endif /*GRASP_HPP_*/
