/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef GRASPVALIDATEFILTER_HPP_
#define GRASPVALIDATEFILTER_HPP_

#include "Grasp3D.hpp"

namespace rw {
namespace graspplanning {

/**
 * @brief tests if a grasp is valid in respect to some criterias implemented
 * by a sub class.
 */
class GraspValidateFilter {
public:

    /**
     * @brief destructor
     */
    virtual ~GraspValidateFilter(){};

    /**
     * @brief tests if a grasp \b grasp is valid in regard to the criterias
     * of the class that implements this function.
     * @param grasp
     * @return
     */
    virtual bool isValid(const Grasp3D& grasp) = 0;

};

}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
