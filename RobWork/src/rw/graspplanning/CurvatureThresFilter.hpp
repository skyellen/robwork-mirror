/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef CURVATURETHRESFILTER_HPP_
#define CURVATURETHRESFILTER_HPP_

#include "Grasp3D.hpp"
#include "GraspValidateFilter.hpp"
#include "ContactValidateFilter.hpp"

namespace rw {
namespace graspplanning {

/**
 * @brief tests if a grasp is valid in respect to the curvature of the object
 * surface in and around the contact points.
 *
 * This class requires that the face in which the contact point is extracted is
 * registered in the Contact3D data.
 *
 */
class CurvatureThresFilter: public GraspValidateFilter, public ContactValidateFilter {
public:

    CurvatureThresFilter(double minCurvature, double maxCurvature):
        _minCurvature(minCurvature), _maxCurvature(maxCurvature)
    {};

    virtual ~CurvatureThresFilter(){};

    bool isValid(const Grasp3D& grasp);

    bool isValid(const Contact3D& contact);

private:
    double _minCurvature;
    double _maxCurvature;
};

}
}

#endif /* PLANECLEARANCEFILTER_HPP_ */
