
#ifndef SUPPORTPOSE_HPP_
#define SUPPORTPOSE_HPP_

#include <rw/math/Vector3D.hpp>
#include <vector>


class SupportPose {
public:

	SupportPose(int degree, double prob):
		_degree(degree),
		_rotAxes(1),
		_rotAxesTable(1),
		_probability(prob)
	{};

	virtual ~SupportPose(){};

	// redundant, since length of _rotAxes is also the degree.
	// though its nice to have
	int _degree;
	std::vector< rw::math::Vector3D<> > _rotAxes; // relative to own coordinate frmae
	std::vector< rw::math::Vector3D<> > _rotAxesTable; // relative to supporting structures frame

	// each rotation axis can be valid in a number of angle intervals
	std::vector< std::vector<std::pair<double,double> > > _segments;

	//rw::math::Transform3D<> _trans;

	// the statistics
	double _probability;
	double _quality;
};

#endif
