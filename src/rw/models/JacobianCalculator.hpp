/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_MODELS_JACOBIANCALCULATOR_HPP
#define RW_MODELS_JACOBIANCALCULATOR_HPP

#include <rw/common/Ptr.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKTable.hpp>

namespace rw {
namespace models {


/**
 * @brief JacobianCalculator provides an interface for obtaining a Jacobian
 *
 */
class JacobianCalculator
{
public:
    /**
     * @brief Destructor
     */
    virtual ~JacobianCalculator();

    /**
     * @brief Returns the Jacobian associated to \b state
     * @param state [in] State for which to calculate the Jacobian
     * @return Jacobian for \b state
     */
    virtual rw::math::Jacobian get(const rw::kinematics::State& state) const {
        rw::kinematics::FKTable fk(state);
        return get(fk);
    }

    /**
     * @brief Returns the Jacobian calculated based on the content of \b fk
     * @param fk [in] Forward kinematics table based on which to calculate the Jacobian
     * @return Jacobian for \b fk
     */
    virtual rw::math::Jacobian get(const rw::kinematics::FKTable& fk) const = 0;


};


typedef rw::common::Ptr<JacobianCalculator> JacobianCalculatorPtr;


} //end namespace models
} //end namespace rw


#endif //end include guard
