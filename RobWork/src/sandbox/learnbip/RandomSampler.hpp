/*
 * RandomSampler.hpp
 *
 *  Created on: 24/08/2012
 *      Author: thomas
 */

#ifndef RANDOMSAMPLER_HPP_
#define RANDOMSAMPLER_HPP_

#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>

class RandomSampler: public rw::pathplanning::QSampler {
public:
	typedef std::pair<double,double> Pair;
	RandomSampler(const rw::models::WorkCell::Ptr wc, const rw::models::Device::Ptr device, const rw::math::Transform3D<> endTtcp = rw::math::Transform3D<>::identity(),Pair radius = Pair(0.3,0.7), Pair x = std::pair<double,double>(-0.2,0.4), Pair y = std::pair<double,double>(-1.1,-0.5), double height = 0.45, double maxangleZ = 60*rw::math::Deg2Rad);
	virtual ~RandomSampler();

private:
	const rw::models::WorkCell::Ptr _wc;
	const rw::models::Device::Ptr _device;
	const rw::invkin::InvKinSolver *_iksolver;
	const rw::math::Transform3D<> _tcpTend;
	rw::proximity::CollisionDetector *_detector;

	virtual rw::math::Q doSample();
	bool doEmpty() const { return false; }

	bool inverseKin(std::vector<rw::math::Q>& sol, const rw::math::Transform3D<> &wTtarget, const rw::kinematics::State &state) const;

	Pair _radius;
	Pair _x;
	Pair _y;
	double _height;
	double _cosanglez;
};

#endif /* RANDOMSAMPLER_HPP_ */
