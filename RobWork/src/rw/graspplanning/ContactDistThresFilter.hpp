/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef CONTACTDISTTHRESFILTER_HPP_
#define CONTACTDISTTHRESFILTER_HPP_

#include "Grasp3D.hpp"
#include "GraspValidateFilter.hpp"

#include <rw/sensor/Contact3D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief tests if contact points in a grasp is too close to each other in respect to the
 * approach angle.
 *
 * Two points that are very close is not allowed unless they are approached from opposite
 * directions.
 *
 */
class ContactDistThresFilter: public GraspValidateFilter {
public:

    /**
     *
     * @param minDist
     * @param maxDist
     * @param allowCloseWhenOpposite
     * @return
     */
    ContactDistThresFilter(double minDist, double maxDist, bool allowCloseWhenOpposite = true):
        _minDist(minDist), _maxDist(maxDist),_allowCloseWhenOpposite(allowCloseWhenOpposite)
    {};

    /**
     * @brief destructor
     */
    virtual ~ContactDistThresFilter(){};

    /**
     * @copydoc GraspValidateFilter::isValid
     */
    bool isValid(const Grasp3D& grasp);

    bool isContactPairValid(const rw::sensor::Contact3D& c1, const rw::sensor::Contact3D& c2);

private:
    double _minDist;
    double _maxDist;
    bool _allowCloseWhenOpposite;
};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
