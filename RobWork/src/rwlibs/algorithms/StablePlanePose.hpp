/*
 * StablePlanePose.hpp
 *
 *  Created on: 12/02/2010
 *      Author: jimali
 */

#ifndef STABLEPLANEPOSE_HPP_
#define STABLEPLANEPOSE_HPP_

/**
 * @brief calculate the stable poses of an object lying on a planar support
 * structure.
 */
class StablePlanePose {

	StablePlanePose();
	StablePlanePose(rw::geometry::GeometryPtr geom);
	StablePlanePose(rw::geometry::GeometryPtr geom, const rw::math::Vector3D<>& CM);

	void setGeometry(rw::geometry::GeometryPtr geom);
	void setGeometry(rw::geometry::GeometryPtr geom, const rw::math::Vector3D<>& CM);

	/**
	 * @brief calculate all stable poses for the geometry lying on a planar
	 * support structure.
	 * @param dist [in] the minimum distance from the edge of the supporting
	 * polygon to the point defined by CM projected onto the supporting polygon.
	 */
	std::vector<StablePose> calculatePoses(double dist);
};

#endif /* STABLEPLANEPOSE_HPP_ */
