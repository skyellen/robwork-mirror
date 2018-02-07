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

#include <rw/math/Statistics.hpp>

using namespace rw::math;

TEST(Statistics, Double) {
	Statistics<double> stats;
	stats.add(1.1);
	stats.add(0.2);
	stats.add(-0.065);
	stats.add(0.005);
	EXPECT_DOUBLE_EQ(0.31,stats.mean());
	EXPECT_DOUBLE_EQ(0.28995,stats.variance());
	EXPECT_DOUBLE_EQ(0.31,stats.meanAndVariance().first);
	EXPECT_DOUBLE_EQ(0.28995,stats.meanAndVariance().second);
	EXPECT_DOUBLE_EQ(0.1025,stats.median());
	EXPECT_DOUBLE_EQ(-0.065,stats.minValue());
	EXPECT_DOUBLE_EQ(1.1,stats.maxValue());
	EXPECT_DOUBLE_EQ(-0.065,stats.minAndMaxValue().first);
	EXPECT_DOUBLE_EQ(1.1,stats.minAndMaxValue().second);

	EXPECT_DOUBLE_EQ(0.2915800123437472,stats.angularMean());
	EXPECT_DOUBLE_EQ(0.2904023945936753,stats.angularVariance());
	EXPECT_DOUBLE_EQ(0.2915800123437472,stats.angularMeanAndVariance().first);
	EXPECT_DOUBLE_EQ(0.2904023945936753,stats.angularMeanAndVariance().second);

}

TEST(Statistics, Float) {
	Statistics<float> stats;
	stats.add(1.1f);
	stats.add(0.2f);
	stats.add(-0.065f);
	stats.add(0.005f);
	EXPECT_FLOAT_EQ(0.31f,stats.mean());
	EXPECT_FLOAT_EQ(0.28995f,stats.variance());
	EXPECT_FLOAT_EQ(0.31f,stats.meanAndVariance().first);
	EXPECT_FLOAT_EQ(0.28995f,stats.meanAndVariance().second);
	EXPECT_FLOAT_EQ(0.1025f,stats.median());
	EXPECT_FLOAT_EQ(-0.065f,stats.minValue());
	EXPECT_FLOAT_EQ(1.1f,stats.maxValue());
	EXPECT_FLOAT_EQ(-0.065f,stats.minAndMaxValue().first);
	EXPECT_FLOAT_EQ(1.1f,stats.minAndMaxValue().second);

	EXPECT_FLOAT_EQ(0.29158f,stats.angularMean());
	EXPECT_FLOAT_EQ(0.2904024f,stats.angularVariance());
	EXPECT_FLOAT_EQ(0.29158f,stats.angularMeanAndVariance().first);
	EXPECT_FLOAT_EQ(0.2904024f,stats.angularMeanAndVariance().second);
}
