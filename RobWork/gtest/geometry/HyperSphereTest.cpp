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

#include <gtest/gtest.h>

#include <rw/math/Constants.hpp>
#include <rw/geometry/HyperSphere.hpp>

using rw::math::Pi;
using rw::geometry::HyperSphere;

TEST(HyperSphereTest, 1D) {
	const HyperSphere sphere(1);
	EXPECT_EQ(1,sphere.getDimensions());
	EXPECT_DOUBLE_EQ(2,sphere.area()); // two end-points
	EXPECT_DOUBLE_EQ(2,sphere.volume()); // length between end-points
}

// Test distribution on a circle
TEST(HyperSphereTest, 2D) {
	static const double delta = 0.1; // radians
	const HyperSphere sphere(2);
	EXPECT_EQ(2,sphere.getDimensions());
	const std::vector<Eigen::VectorXd> spherical = sphere.uniformDistributionSpherical(delta);
	const std::vector<Eigen::VectorXd> cartesian = sphere.uniformDistributionCartesian(delta);
	EXPECT_EQ(spherical.size(), cartesian.size());
	EXPECT_EQ(63,spherical.size());
	for (std::size_t i = 0; i < std::min(spherical.size(),cartesian.size()); i++) {
		EXPECT_EQ(1,spherical[i].size());
		EXPECT_EQ(2,cartesian[i].size());
		ASSERT_GE(spherical[i].size(),1);
		ASSERT_GE(cartesian[i].size(),2);
		EXPECT_NEAR(delta*(0.5+i),spherical[i][0],1e-14);
		EXPECT_NEAR(std::cos(delta*(0.5+i)),cartesian[i][0],1e-14);
		EXPECT_NEAR(std::sin(delta*(0.5+i)),cartesian[i][1],1e-14);
	}
	EXPECT_DOUBLE_EQ(Pi*2,sphere.area()); // circumference of circle
	EXPECT_DOUBLE_EQ(Pi,sphere.volume()); // area of circle
}

// Test distribution on a sphere
TEST(HyperSphereTest, 3D) {
	static const double delta = 0.1; // radians
	const HyperSphere sphere(3);
	EXPECT_EQ(3,sphere.getDimensions());
	const std::vector<Eigen::VectorXd> spherical = sphere.uniformDistributionSpherical(delta);
	const std::vector<Eigen::VectorXd> cartesian = sphere.uniformDistributionCartesian(delta);
	EXPECT_EQ(spherical.size(), cartesian.size());
	EXPECT_EQ(1258,spherical.size());
	for (std::size_t i = 0; i < std::min(spherical.size(),cartesian.size()); i++) {
		EXPECT_EQ(2,spherical[i].size());
		EXPECT_EQ(3,cartesian[i].size());
		ASSERT_GE(spherical[i].size(),2);
		ASSERT_GE(cartesian[i].size(),3);
		EXPECT_GE(spherical[i][0],delta/2);
		EXPECT_LE(spherical[i][0],Pi);
	}
	EXPECT_DOUBLE_EQ(Pi*4,sphere.area());
	EXPECT_DOUBLE_EQ(Pi*4/3,sphere.volume());
}

// Higher dimensions
TEST(HyperSphereTest, 4D) {
	static const double delta = 0.1; // radians
	const HyperSphere sphere(4);
	EXPECT_EQ(4,sphere.getDimensions());
	const std::vector<Eigen::VectorXd> spherical = sphere.uniformDistributionSpherical(delta);
	const std::vector<Eigen::VectorXd> cartesian = sphere.uniformDistributionCartesian(delta);
	EXPECT_EQ(spherical.size(), cartesian.size());
	EXPECT_EQ(19739,spherical.size());
	for (std::size_t i = 0; i < std::min(spherical.size(),cartesian.size()); i++) {
		EXPECT_EQ(3,spherical[i].size());
		EXPECT_EQ(4,cartesian[i].size());
		ASSERT_GE(spherical[i].size(),3);
		ASSERT_GE(cartesian[i].size(),4);
		EXPECT_GE(spherical[i][0],delta/2);
		for (unsigned int j = 0; j <= 1; j++) {
			EXPECT_GE(spherical[i][j],0);
			EXPECT_LE(spherical[i][j],Pi);
		}
	}
	EXPECT_DOUBLE_EQ(Pi*Pi*2,sphere.area());
	EXPECT_DOUBLE_EQ(Pi*Pi/2,sphere.volume());
}

TEST(HyperSphereTest, 5D) {
	static const double delta = 0.12; // radians
	const HyperSphere sphere(5);
	EXPECT_EQ(5,sphere.getDimensions());
	const std::vector<Eigen::VectorXd> spherical = sphere.uniformDistributionSpherical(delta);
	const std::vector<Eigen::VectorXd> cartesian = sphere.uniformDistributionCartesian(delta);
	EXPECT_EQ(spherical.size(), cartesian.size());
	EXPECT_EQ(126813,spherical.size());
	for (std::size_t i = 0; i < std::min(spherical.size(),cartesian.size()); i++) {
		EXPECT_EQ(4,spherical[i].size());
		EXPECT_EQ(5,cartesian[i].size());
		ASSERT_GE(spherical[i].size(),4);
		ASSERT_GE(cartesian[i].size(),5);
		EXPECT_GE(spherical[i][0],delta/2);
		for (unsigned int j = 0; j <= 2; j++) {
			EXPECT_GE(spherical[i][j],0);
			EXPECT_LE(spherical[i][j],Pi);
		}
	}
	EXPECT_DOUBLE_EQ(Pi*Pi*8/3,sphere.area());
	EXPECT_DOUBLE_EQ(Pi*Pi*8/15,sphere.volume());
}

TEST(HyperSphereTest, 6D) {
	static const double delta = 0.16; // radians
	const HyperSphere sphere(6);
	EXPECT_EQ(6,sphere.getDimensions());
	const std::vector<Eigen::VectorXd> spherical = sphere.uniformDistributionSpherical(delta);
	const std::vector<Eigen::VectorXd> cartesian = sphere.uniformDistributionCartesian(delta);
	EXPECT_EQ(spherical.size(), cartesian.size());
	EXPECT_EQ(295749,spherical.size());
	for (std::size_t i = 0; i < std::min(spherical.size(),cartesian.size()); i++) {
		EXPECT_EQ(5,spherical[i].size());
		EXPECT_EQ(6,cartesian[i].size());
		ASSERT_GE(spherical[i].size(),5);
		ASSERT_GE(cartesian[i].size(),6);
		EXPECT_GE(spherical[i][0],delta/2);
		for (unsigned int j = 0; j <= 3; j++) {
			EXPECT_GE(spherical[i][j],0);
			EXPECT_LE(spherical[i][j],Pi);
		}
	}
	EXPECT_DOUBLE_EQ(Pi*Pi*Pi,sphere.area());
	EXPECT_DOUBLE_EQ(Pi*Pi*Pi/6,sphere.volume());
}

TEST(HyperSphereTest, 7D) {
	const HyperSphere sphere(7);
	EXPECT_EQ(7,sphere.getDimensions());
	EXPECT_DOUBLE_EQ(Pi*Pi*Pi*16/15,sphere.area());
	EXPECT_DOUBLE_EQ(Pi*Pi*Pi*16/105,sphere.volume());
}
