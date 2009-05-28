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

#ifndef RW_MODELS_JOINTDEVICEJACOBIANCALCULATOR_HPP
#define RW_MODELS_JOINTDEVICEJACOBIANCALCULATOR_HPP


#include <rw/kinematics/State.hpp>

#include "JacobianCalculator.hpp"
#include "Joint.hpp"
#include "JointDevice.hpp"

namespace rw {
namespace models {

class JointDeviceJacobianCalculator: public JacobianCalculator
{
public:
    JointDeviceJacobianCalculator(/*const std::vector<Joint*>& joints,*/
                                  JointDevicePtr device,
                                  const kinematics::Frame* base,
                                  const std::vector<kinematics::Frame*>& tcps,
                                  const kinematics::State& state);

    virtual ~JointDeviceJacobianCalculator();

    virtual math::Jacobian get(const rw::kinematics::FKTable& table) const;


private:
    const kinematics::Frame* _base;
    std::vector<kinematics::Frame*> _tcps;
    size_t _dof;
    std::vector<Joint*> _joints; //Ordered list with the joints to include
    typedef std::vector<std::pair<const Joint*, size_t> > JacobianSetup;

    std::vector<JacobianSetup> _jacobianSetups;



};

} //end namespace models
} //end namespace rw


#endif /* JOINTDEVICEJACOBIANCALCULATOR_HPP_ */
