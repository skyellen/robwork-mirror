#ifndef GRASP2D_HPP_
#define GRASP2D_HPP_

#include <rw/sensor/Contact2D.hpp>
#include <rw/math/Vector2D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief a grasp is a set of contacts between the object to be grasped and 
 * the robot gripper.
 */

class Grasp2D {
public:

    Grasp2D(int nrOfContacts):
        contacts(nrOfContacts),
        approach(nrOfContacts)
    {}
    
    void scale(double clerance){
        for(int i=0;i<3;i++)
            contacts[i].p += normalize(approach[i])*clerance;
    }
    
    double phi,psi;
    rw::math::Vector2D<> center;
    std::vector<rw::sensor::Contact2D> contacts;
    std::vector<rw::math::Vector2D<> > approach;
    
};
}
}

#endif /*GRASP_HPP_*/
