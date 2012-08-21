/*
 * ClosedFormURSolver.hpp
 *
 *  Created on: 26/07/2012
 *      Author: thomas
 */

#ifndef CLOSEDFORMURSOLVER_HPP_
#define CLOSEDFORMURSOLVER_HPP_

#include <rw/invkin/InvKinSolver.cpp>
#include <rw/invkin/ClosedFormIK.hpp>
#include <rw/math/Vector2D.hpp>

class ClosedFormURSolver: public rw::invkin::ClosedFormIK {
public:
	typedef rw::common::Ptr<ClosedFormURSolver> Ptr;

	ClosedFormURSolver(const rw::models::Device::Ptr device, const rw::kinematics::State& state);
	virtual ~ClosedFormURSolver();

    std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;
    void setCheckJointLimits(bool check);

private:
    rw::math::Q adjustJoints(const rw::math::Q &q) const;

    // UR specific geometric functions
    std::pair<rw::math::Vector3D<>,rw::math::Vector3D<> > getJoint4Positions(const rw::math::Vector3D<> &baseTdh5, const rw::math::Vector3D<> &tcpZ, const rw::kinematics::State &state) const;
    std::pair<std::pair<double,double>,std::pair<double,double> > getElbowJoints(const rw::math::Vector3D<> &intersection, const rw::kinematics::State &state) const;
    rw::math::Q getOrientationJoints(const rw::math::Transform3D<> &baseTend, const rw::math::Vector3D<> &baseTdh5, const rw::kinematics::State &state) const;
	std::pair<double,double> findBaseAngle(const rw::math::Vector2D<> &pos) const;

	// Generic geometric functions
	static std::pair<rw::math::Vector3D<>,rw::math::Vector3D<> > findCirclePlaneIntersection(const rw::math::Vector3D<> &circleCenter, double radius, const rw::math::Vector3D<> &circleDir1, const rw::math::Vector3D<> &circleDir2, const rw::math::Vector3D<> &planeNormal);
	static std::pair<std::pair<double,double>,std::pair<double,double> > findTwoBarAngles(const rw::math::Vector2D<> &pos, double L1, double L2);
	static rw::math::Vector3D<> getPerpendicularVector(const rw::math::Vector3D<> &vec);

    bool _checkJointLimits;
    const rw::models::Device::Ptr _device;
    std::vector<rw::kinematics::Frame*> _frames;
    double _lTcp, _baseRadius, _endCircleRadius, _l1, _l2, _lJ0J1;
};

#endif /* CLOSEDFORMURSOLVER_HPP_ */
