/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef SEMIFORCECLOSUREFILTER_HPP_
#define SEMIFORCECLOSUREFILTER_HPP_

#include "Grasp3D.hpp"

namespace rw {
namespace graspplanning {

/**
 * @brief A conservative estimate of the force closure properties of the grasp are
 * used to indicate weather a grasp is valid or not.
 *
 * The method is described in "Grasping the Dice by Dicing the Grasp"
 */
class SemiForceClosureFilter {
public:

    SemiForceClosureFilter(size_t nrContacts):
        _nrContacts(nrContacts),_avgScale(1.0/nrContacts){};

    virtual ~SemiForceClosureFilter(){};

    bool isValid(const Grasp3D& grasp);

private:
    size_t _nrContacts;
    double _avgScale;
};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
