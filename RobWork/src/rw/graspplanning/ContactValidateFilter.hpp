/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef CONTACTVALIDATEFILTER_HPP_
#define CONTACTVALIDATEFILTER_HPP_

#include <rw/sensor/Contact3D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief tests if a grasp is valid in respect to some criterias implemented
 * by a sub class.
 */
class ContactValidateFilter {
public:

    /**
     * @brief destructor
     */
    virtual ~ContactValidateFilter(){};

    /**
     * @brief tests if a grasp \b grasp is valid in regard to the criterias
     * of the class that implements this function.
     * @param grasp
     * @return
     */
    virtual bool isValid(const rw::sensor::Contact3D& contact) = 0;

};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
