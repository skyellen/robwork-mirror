/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/sensor/TactileArrayModel.cpp>

using rw::sensor::TactileArrayModel;

TEST(TactileArray, TactileArrayModel) {
    static const double cellWidth = 0.1;
    static const double cellHeight = 0.2;
    static const TactileArrayModel::ValueMatrix::Index height = 3;
    static const TactileArrayModel::ValueMatrix::Index width = 4;
    const TactileArrayModel::ValueMatrix heightMap = TactileArrayModel::ValueMatrix::Zero(height, width);
	const TactileArrayModel model("Model", NULL, Transform3D<>::identity(), heightMap, cellWidth, cellHeight);
	EXPECT_EQ("Model", model.getName());
	EXPECT_EQ(height-1, model.getHeight());
	EXPECT_EQ(width-1, model.getWidth());
	EXPECT_EQ(height-1, model.getCenters().shape()[1]);
	EXPECT_EQ(width-1, model.getCenters().shape()[0]);
	EXPECT_EQ(height-1, model.getNormals().shape()[1]);
	EXPECT_EQ(width-1, model.getNormals().shape()[0]);
	EXPECT_EQ(height, model.getVertexGrid().shape()[1]);
	EXPECT_EQ(width, model.getVertexGrid().shape()[0]);
}
