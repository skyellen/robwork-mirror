/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef PLANECLEARANCEFILTER_HPP_
#define PLANECLEARANCEFILTER_HPP_

#include "Grasp3D.hpp"
#include <rw/math/Transform3D.hpp>
#include <rw/sensor/Contact3D.hpp>

#include "GraspValidateFilter.hpp"
#include "ContactValidateFilter.hpp"

namespace rw {
namespace graspplanning {


/**
 * @brief tests if a grasp is valid in respect to the distance of each
 * contact point to some plane.
 *
 * The plane is defined by the xy-plane in
 * a transform that is specified relative to the frame that the object
 * is specified relative to.
 */
class PlaneClearanceFilter : public GraspValidateFilter, public ContactValidateFilter {
public:

    /**
     * Constructor
     * @param planeFrame [in] The transform of the plane relative to the frame
     * that the grasp contact points are described relative to.
     * @param clearance [in] The minimum distance between plane and
     * contact point for a valid grasp
     * @param minAngle [in] the minimum angle that is allowed in
     * a valid grasp. [-Pi/2,Pi/2]
     * @return
     */
    PlaneClearanceFilter(const rw::math::Transform3D<>& planeFrame,
                         double clearance,
                         double minAngle):
        _planeFrame(planeFrame),
        _clearance(clearance),
        _minAngle(minAngle)
   {};

    /**
     * @brief destructor
     */
    virtual ~PlaneClearanceFilter(){};

    /**
     * @brief tests if a grasp \b grasp is valid in regard to the settings
     * of this clearance filter.
     * @param grasp
     * @return
     */
    bool isValid(const Grasp3D& grasp);

    /**
     * @copydoc ContactValidateFilter::isValid
     */
    bool isValid(const rw::sensor::Contact3D& contact);

private:
    rw::math::Transform3D<> _planeFrame;
    double _clearance, _minAngle;
};

}
}

#endif /* PLANECLEARANCEFILTER_HPP_ */
