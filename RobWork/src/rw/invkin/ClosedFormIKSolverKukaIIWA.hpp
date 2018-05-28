/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_INVKIN_CLOSEDFORMIKSOLVERKUKAIIWA_HPP_
#define RW_INVKIN_CLOSEDFORMIKSOLVERKUKAIIWA_HPP_

/**
 * @file ClosedFormIKSolverKukaIIWA.hpp
 *
 * \copydoc rw::invkin::ClosedFormIKSolverKukaIIWA
 */

#include <rw/invkin/ClosedFormIK.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/kinematics/FKRange.hpp>

namespace rw { namespace models { class SerialDevice; } }

namespace rw {
namespace invkin {
//! @addtogroup invkin

//! @{
/**
 * @brief Analytical inverse solver for the Kuka LBR IIWA 7 R800 robot.
 *
 * Notice that this is a 7 DOF robot and that there is an infinite number of solutions.
 * The extra DOF means that the middle joint of the robot is able to move in a circle.
 * This solver will choose a point on this circle randomly and return up to 8 possible solutions.
 */
class ClosedFormIKSolverKukaIIWA: public ClosedFormIK {
public:
	//! @brief Smart pointer type to ClosedFormIKSolverKukaIIWA
	typedef rw::common::Ptr<ClosedFormIKSolverKukaIIWA> Ptr;

	/**
	 * @brief Construct new closed form solver for a Kuka 7 DOF IIWA robot.
	 * @param device [in] the device.
	 * @param state [in] the state to get the frame structure and extract the dimensions from.
	 */
	ClosedFormIKSolverKukaIIWA(const rw::common::Ptr<const rw::models::SerialDevice> device, const rw::kinematics::State& state);

	//! @brief Destructor.
	virtual ~ClosedFormIKSolverKukaIIWA();

	//! @copydoc InvKinSolver::solve
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;

    /**
     * @brief Find inverse kinematic solutions deterministically by pulling joint 4 as much in the given direction as possible.
     * @param baseTend [in] Desired base to end transformation \f$\robabx{}{desired}{\mathbf{T}}\f$.
     * @param state [in] State of the device from which to start the iterations.
     * @param dir4 [in] unit vector giving the direction to pull joint 4 in (given in base coordinate system).
     * @return List of up to 8 solutions. Notice that the list may be empty.
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state, const rw::math::Vector3D<>& dir4) const;

	//! @copydoc InvKinSolver::setCheckJointLimits
    void setCheckJointLimits(bool check);

    /**
     * @copydoc InvKinSolver::getTCP
     */
    virtual rw::common::Ptr< const rw::kinematics::Frame > getTCP() const;            

private:
	std::pair<double,double> findBaseAngles(const rw::math::Vector2D<> &pos, const rw::kinematics::State& state) const;

	void addBaseAngleSolutions(const rw::math::Rotation3D<>& baseRend, const rw::math::Vector3D<>& baseP6, const rw::math::Vector3D<>& basePtarget4, rw::kinematics::State& state, double angle, std::vector<rw::math::Q>& res) const;
	void addOuterSolutions(const rw::math::Rotation3D<>& baseRend, const rw::math::Vector3D<>& baseP6, rw::kinematics::State& state, double angle1, double angle2, double angle3, std::vector<rw::math::Q>& res) const;
	void addRotationSolutions(const rw::math::Rotation3D<>& baseRend, rw::kinematics::State& state, double angle1, double angle2, double angle3, double angle4, double angle5, std::vector<rw::math::Q>& res) const;

	static rw::math::Vector3D<> randomPerpendicularVector(const rw::math::Vector3D<>& v);

private:
    const rw::common::Ptr<const rw::models::SerialDevice> _device;
    bool _checkJointLimits;
    std::vector<const rw::kinematics::Frame*> _frames;
    const rw::math::Q _qLow;
    const rw::math::Q _qHigh;

    double _lTcp;
    double _lJ3J4;
    double _lJ2J4;
    rw::math::Vector3D<> _baseP2;

	rw::kinematics::FKRange _fkRange2_0;
	rw::kinematics::FKRange _fkRange3_0;
	rw::kinematics::FKRange _fkRange4_0;
	rw::kinematics::FKRange _fkRange5_0;
	rw::kinematics::FKRange _fkRange6_0;
};
//! @}
} /* namespace invkin */
} /* namespace rw */

#endif /* RW_INVKIN_CLOSEDFORMIKSOLVERKUKAIIWA_HPP_ */
