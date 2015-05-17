/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef CLOSEDFORMURSOLVER_HPP_
#define CLOSEDFORMURSOLVER_HPP_

#include <rw/invkin/ClosedFormIK.hpp>
#include <rw/math/Vector2D.hpp>

namespace rw { namespace models { class SerialDevice; } }

class ClosedFormURSolver: public rw::invkin::ClosedFormIK {
public:
	//! @brief Smart pointer type to ClosedFormURSolver
	typedef rw::common::Ptr<ClosedFormURSolver> Ptr;

	/**
	 * @brief Construct new closed form solver for a Universal Robot.
	 * @note The dimensions will be automatically extracted from the device, using an arbitrary state.
	 * @param device [in] the device.
	 * @param state [in] the state to use to extract dimensions.
	 */
	ClosedFormURSolver(const rw::common::Ptr<const rw::models::SerialDevice> device, const rw::kinematics::State& state);

	//! @brief Destructor.
	virtual ~ClosedFormURSolver();

	//! @copydoc InvKinSolver::solve
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;

	//! @copydoc InvKinSolver::setCheckJointLimits
    void setCheckJointLimits(bool check);

private:
    rw::math::Q adjustJoints(const rw::math::Q &q) const;

    void addBaseAngleSolutions(const rw::math::Transform3D<>& baseTend, const rw::math::Vector3D<>& baseTdh5, const rw::kinematics::State& state, double angle, std::vector<rw::math::Q>& res) const;
    void addElbowSolutions(const rw::math::Transform3D<>& baseTend, const rw::math::Vector3D<>& baseTdh5, const rw::kinematics::State& state, double baseAngle, std::pair<double,double> elbow, std::vector<rw::math::Q>& res) const;

    // UR specific geometric functions
    std::pair<rw::math::Vector3D<>,rw::math::Vector3D<> > getJoint4Positions(const rw::math::Vector3D<> &baseTdh5, const rw::math::Vector3D<> &tcpZ, const rw::kinematics::State &state) const;
    std::pair<std::pair<double,double>,std::pair<double,double> > getElbowJoints(const rw::math::Vector3D<> &intersection, const rw::kinematics::State &state) const;
    rw::math::Q getOrientationJoints(const rw::math::Transform3D<> &baseTend, const rw::math::Vector3D<> &baseTdh5, const rw::kinematics::State &state) const;
	std::pair<double,double> findBaseAngle(const rw::math::Vector2D<> &pos) const;

	// Generic geometric functions
	static std::pair<rw::math::Vector3D<>,rw::math::Vector3D<> > findCirclePlaneIntersection(const rw::math::Vector3D<> &circleCenter, double radius, const rw::math::Vector3D<> &circleDir1, const rw::math::Vector3D<> &circleDir2, const rw::math::Vector3D<> &planeNormal);
	static std::pair<std::pair<double,double>,std::pair<double,double> > findTwoBarAngles(const rw::math::Vector2D<> &pos, double L1, double L2);
	static rw::math::Vector3D<> getPerpendicularVector(const rw::math::Vector3D<> &vec);

private:
    const rw::common::Ptr<const rw::models::SerialDevice> _device;
    bool _checkJointLimits;
    std::vector<const rw::kinematics::Frame*> _frames;
    double _lTcp, _baseRadius, _endCircleRadius, _l1, _l2, _lJ0J1;
};

#endif /* CLOSEDFORMURSOLVER_HPP_ */
