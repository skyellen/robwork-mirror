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
	RandomSampler(const rw::models::WorkCell::Ptr wc, const rw::models::Device::Ptr device);
	virtual ~RandomSampler();

private:
	const rw::models::WorkCell::Ptr _wc;
	const rw::models::Device::Ptr _device;
	const rw::invkin::InvKinSolver *_iksolver;
	rw::proximity::CollisionDetector *_detector;

	virtual rw::math::Q doSample();
	bool doEmpty() const { return false; }

	bool inverseKin(std::vector<rw::math::Q>& sol, const rw::math::Transform3D<> &target, const rw::kinematics::State &state) const;
};

#endif /* RANDOMSAMPLER_HPP_ */
