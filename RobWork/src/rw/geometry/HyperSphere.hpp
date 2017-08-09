/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_HYPERSPHERE_HPP_
#define RW_GEOMETRY_HYPERSPHERE_HPP_

/**
 * @file HyperSphere.hpp
 *
 * \copydoc rw::geometry::HyperSphere
 */

#include <rw/common/Ptr.hpp>

#include <Eigen/Core>

#include <vector>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief A hyper-sphere of K dimensions.
 *
 * Functions are provided to create (almost) uniform distribution of points on a hyper-sphere as shown in [1].
 *
 * The distribution of points is illustrated below for 2 and 3 dimensional hyper-spheres.
 * Notice that the tessellation is best when \f$\delta\f$ is small.
 *
 * \image html geometry/hypersphere.gif "Distribution of points for K=2 and K=3."
 *
 * [1] Lovisolo, L., and E. A. B. Da Silva. "Uniform distribution of points on a hyper-sphere with applications to vector bit-plane encoding."
 * IEE Proceedings-Vision, Image and Signal Processing 148.3 (2001): 187-193.
 */
class HyperSphere {
public:
	//! @brief Smart pointer type for HyperSphere.
	typedef rw::common::Ptr<const HyperSphere> Ptr;

	/**
	 * @brief Construct a hyper-sphere of unit size.
	 * @param dimensions [in] the number of dimensions.
	 */
	HyperSphere(unsigned int dimensions);

	//! @brief Destructor.
	virtual ~HyperSphere();

	/**
	 * @brief Create a uniform distribution in Cartesian coordinates.
	 *
	 * This uses #uniformDistributionSpherical and maps the spherical coordinates to Cartesian coordinates.
	 * The mapping is documented in [1], section 2.1.
	 *
	 * @param delta [in] the resolution.
	 * @return unit vectors, \f$[x_1 x_2 \dots x_K]^T\f$, in Cartesian coordinates with dimension K.
	 * @note This function is only implemented for \f$2 \leq K \leq 6\f$.
	 */
	std::vector<Eigen::VectorXd> uniformDistributionCartesian(double delta) const;

	/**
	 * @brief Create a uniform distribution in spherical coordinates.
	 *
	 * This implements the algorithm in [1], section 2.1, for dimensions \f$2 \leq K \leq 6\f$.
	 *
	 * @param delta [in] the resolution.
	 * @return list of vectors, \f$[\theta_1 \theta_2 \dots \theta_{K-1}]^T\f$, in spherical coordinates with dimension K-1.
	 * @note This function is only implemented for \f$2 \leq K \leq 6\f$.
	 */
	std::vector<Eigen::VectorXd> uniformDistributionSpherical(double delta) const;

	/**
	 * @brief Get the number of dimensions of the hyper-sphere.
	 * @return the number of dimensions, \f$2 \leq K \leq 6\f$.
	 */
	unsigned int getDimensions() const;

	/**
	 * @brief Calculate the surface area of a hyper-sphere.
	 *
	 * Calculated for even dimensionality as \f$\frac{K \pi^{K/2}}{(K/2)!}\f$
	 *
	 * Calculated for odd dimensionality as \f$\frac{K 2^K \pi^{(K-1)/2}}{K!}\f$
	 *
	 * @return the surface area.
	 */
	double area() const;

	/**
	 * @brief The volume of a hyper-sphere.
	 *
	 * Calculated for even dimensionality as \f$\frac{\pi^{K/2}}{(K/2)!}\f$
	 *
	 * Calculated for odd dimensionality as \f$\frac{2 (2 \pi)^{(K-1)/2}}{K!!}\f$
	 * where the double factorial for odd K means \f$1 \cdot 3 \cdot 5 \dots K\f$
	 *
	 * @return the volume.
	 */
	double volume() const;

private:
	const unsigned int _dimensions;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_HYPERSPHERE_HPP_ */
