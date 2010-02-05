/*
 * CircleModel.hpp
 *
 *  Created on: 10-03-2009
 *      Author: jimali
 */

#ifndef CIRCLEMODEL_HPP_
#define CIRCLEMODEL_HPP_

#include <rw/math/Vector3D.hpp>

#include <vector>

class CircleModel {
public:
	rw::math::Vector3D<> _center,_n;
	double _r;

	CircleModel(){};

	/**
	 * @brief create a circle from a number of points
	 */
	CircleModel(std::vector<rw::math::Vector3D<> >& points);

	CircleModel(rw::math::Vector3D<>& p1, rw::math::Vector3D<>& p2, rw::math::Vector3D<>& p3);

	void refit(std::vector<rw::math::Vector3D<> >& points);

	/**
	 * @brief tests if two circles are equal which is within some epsilon of
	 * each other
	 * @param circ
	 * @param epsilon
	 * @return
	 */
	bool isClose(const CircleModel& circ, double epsilon) const;

	bool isClose(const rw::math::Vector3D<>& p, double epsilon) const ;

	void print() const ;

};


#endif /* CIRCLEMODEL_HPP_ */
