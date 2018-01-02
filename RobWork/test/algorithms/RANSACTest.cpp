/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "../TestSuiteConfig.hpp"

#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rwlibs/algorithms/LineModel.hpp>
#include <rwlibs/algorithms/StructuredLineModel.hpp>
#include <rwlibs/algorithms/PlaneModel.hpp>
#include <rwlibs/algorithms/StablePose1DModel.hpp>
#include <rwlibs/algorithms/StablePose0DModel.hpp>

#include <boost/foreach.hpp>

#include <vector>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using namespace rw::math;
using namespace rwlibs::algorithms;



vector<Transform3D<> > readData(istream& stream) {
	string line;
	vector<Transform3D<> > data;
	
	while (getline(stream, line)) {
		replace(line.begin(), line.end(), ',', ' ');
		stringstream sstr(line);
		
		// read position
		double m[3];
		sstr >> m[0] >> m[1] >> m[2];
		Vector3D<> pos(m[0], m[1], m[2]);
		
		// read rpy
		sstr >> m[0] >> m[1] >> m[2];
		RPY<> rpy(m[0], m[1], m[2]);
		
		Transform3D<> t(pos, rpy.toRotation3D());
		data.push_back(t);
	}
	
	return data;
}


BOOST_AUTO_TEST_CASE(RANSACLineTest) {
	/* Test whether RANSAC finds a line model in supplied data file.
	 * 
	 * Test file contains 10 points lying on a line with noise of sigma=0.1 added to their position.
	 * Test file also contains 5 random outliers.
	 */
	BOOST_TEST_MESSAGE("- Testing line fitting with RANSAC");
	
	// read data file
	string filePath = testFilePath() + "ransac/line_data.csv";
	ifstream inFile(filePath.c_str());
	vector<Transform3D<> > data = readData(inFile);
	inFile.close();
    BOOST_CHECK(data.size() == 15);

    // extract positions
    vector<Vector3D<> > pos;
    BOOST_FOREACH (const Transform3D<>& t, data) {
        pos.push_back(t.P());
    }

    int seed = 0;

    // Run the test for 100 iterations, to make sure it does not succeed based on a lucky seed.
    while (seed < 100)
    {
        std::srand(seed++);
        // find models
        vector<LineModel> models = LineModel::findModels(pos, 100, 5, 0.1, 0.001);
        LineModel bestModel = LineModel::bestModel(models);

        // check if any model found
        BOOST_CHECK(models.size() > 0);

        // check if the model is acceptably close to ground truth
        rw::geometry::Line referenceLine(
                    Vector3D<>(0.570924747488628, -0.796576799930771, 0.198772066742534),
                    Vector3D<>(0.989086430219537, -0.722790548702032, -0.706598794410653)
                    );
        LineModel referenceModel(referenceLine);

        BOOST_CHECK(bestModel.same(referenceModel, 0.01));
    }
}


BOOST_AUTO_TEST_CASE(RANSACStructuredLineTest) {
    /* Test whether RANSAC finds a structured line model in supplied data file.
     *
     * Test file contains 10 points lying on a line with noise of sigma=0.1 added to their position.
     * Test file also contains 5 random outliers.
     */
    BOOST_TEST_MESSAGE("- Testing structured line fitting with RANSAC");


    // read data file
    string filePath = testFilePath() + "ransac/sline_data.csv";
    ifstream inFile(filePath.c_str());
    vector<Transform3D<> > data = readData(inFile);
    inFile.close();
    BOOST_CHECK(data.size() == 15);

    // extract positions
    vector<Vector3D<> > pos;
    BOOST_FOREACH (const Transform3D<>& t, data) {
        pos.push_back(t.P());
    }


    int seed = 0;

    // Run the test for 100 iterations, to make sure it does not succeed based on a lucky seed.
    while (seed < 100)
    {
        std::srand(seed++);


        // find models
        vector<StructuredLineModel> models = StructuredLineModel::findModels(pos, 500, 5, 0.2, 0.001);
        StructuredLineModel bestModel = StructuredLineModel::bestModel(models);

        // check if any model found
        BOOST_CHECK(models.size() > 0);

        // check if the model is acceptably close to ground truth
        rw::geometry::Line referenceLine(
                    Vector3D<>(-0.00421171, 0.0240888, 0.023985),
                    Vector3D<>(0.677764, -0.428622, 0.598408)
                    );
        StructuredLineModel referenceModel(referenceLine, Vector3D<>(0.68698, -0.43474, 0.60617), 0.944152);

        BOOST_CHECK(bestModel.same(referenceModel, 0.03));
    }
}


BOOST_AUTO_TEST_CASE(RANSACPlaneTest) {
    /* Test whether RANSAC finds a plane model in provided data file.
     *
     * Test file contains 10 points lying on a plane.
     * Test file also contains 5 random outliers.
     */
    BOOST_TEST_MESSAGE("- Testing plane fitting with RANSAC");


    // read data file
    string filePath = testFilePath() + "ransac/plane_data.csv";
    ifstream inFile(filePath.c_str());
    vector<Transform3D<> > data = readData(inFile);
    inFile.close();
    BOOST_CHECK(data.size() == 15);

    // extract positions
    vector<Vector3D<> > pos;
    BOOST_FOREACH (const Transform3D<>& t, data) {
        pos.push_back(t.P());
    }


    int seed = 0;

    // Run the test 100 times, to ensure it does not pass based on luck!
    while (seed < 100)
    {
        std::srand(seed++);

        // find models
        vector<PlaneModel> models = PlaneModel::findModels(pos, 1000, 6, 0.01, 0.01);
        PlaneModel bestModel = PlaneModel::bestModel(models);

        // check if any model found
        BOOST_CHECK(models.size() > 0);

        // check if the model is acceptably close to ground truth
        rw::geometry::Plane referencePlane(
                    Vector3D<>(-0.25435, -0.962100, 0.098347),
                    0.25 // distance from plane to origo.
                    );
        PlaneModel referenceModel(referencePlane);
        BOOST_CHECK(referencePlane.d() == -referenceModel.d()); // Notice that the PlaneModel uses the distance from the origo to the plane (which is opposite of the Plane).

        BOOST_CHECK(bestModel.same(referenceModel, 0.01));
    }
}

BOOST_AUTO_TEST_CASE(RANSACStablePose1DTest) {
    /* Test whether RANSAC finds a stable pose with 1 dof model in provided data file.
     *
     * Test file contains 10 points with orientations matching a stable pose with noise of sigma=5deg.
     * Test file also contains 5 random outliers.
     */
    BOOST_TEST_MESSAGE("- Testing stable pose 1 dof fitting with RANSAC");


    // read data file
    string filePath = testFilePath() + "ransac/stablepose_data.csv";
    ifstream inFile(filePath.c_str());
    vector<Transform3D<> > data = readData(inFile);
    inFile.close();
    BOOST_CHECK(data.size() > 0);

    // extract orientations
    vector<Rotation3D<> > rot;
    BOOST_FOREACH (const Transform3D<>& t, data) {
        rot.push_back(t.R());
    }

    // Run the test with 1000 different seeds:
    int seed = 0;
    while (seed < 100)
    {
        std::srand(seed++);

        // find models
        vector<StablePose1DModel> models = StablePose1DModel::findModels(rot, 100, 5, 0.1, 0.01);
        StablePose1DModel bestModel = StablePose1DModel::bestModel(models);

        // check if any model found
        BOOST_CHECK(models.size() > 0);

        // check if the model is acceptably close to ground truth
        StablePose1DModel referenceModel(
                    Vector3D<>(0, 0, 1),
                    Vector3D<>(0, 0, 1)
                    );

        BOOST_CHECK(bestModel.same(referenceModel, 0.05));

        // check if stable pose defined by axes is the same as one defined by planes and a normal
        StablePose1DModel modelPlanes(
                    Vector3D<>(0.181716811988644, 0.619803060100149, -0.763428560463371),
                    Vector3D<>(0, 0, 1)
                    );

        StablePose1DModel modelAxes = StablePose1DModel::fromAxes(
                    Vector3D<>(0, 0, 1),
                    Vector3D<>(0.181716811988644, 0.619803060100149, -0.763428560463371)
                    );

        BOOST_CHECK(modelPlanes.same(modelAxes, 0.01));
    }
}

BOOST_AUTO_TEST_CASE(RANSACStablePose0DTest) {
    /* Test whether RANSAC finds a stable pose with 0 dof model in provided data file.
     *
     * Test file contains 10 points with orientations matching a stable pose with noise of sigma=5deg.
     * Test file also contains 5 random outliers.
     */
    BOOST_TEST_MESSAGE("- Testing stable pose 0 dof fitting with RANSAC");



    // read data file
    string filePath = testFilePath() + "ransac/stablepose0_data.csv";
    ifstream inFile(filePath.c_str());
    vector<Transform3D<> > data = readData(inFile);
    inFile.close();
    BOOST_CHECK(data.size() > 0);

    // extract orientations
    vector<Rotation3D<> > rot;
    BOOST_FOREACH (const Transform3D<>& t, data) {
        rot.push_back(t.R());
    }


    // Run the test with 1000 different seeds:
    int seed = 0;
    while (seed < 100)
    {
        std::srand(seed++);

        // find models
        vector<StablePose0DModel> models = StablePose0DModel::findModels(rot, 100, 5, 0.5, 0.5);
        StablePose0DModel bestModel = StablePose0DModel::bestModel(models);

        // check if any model found
        BOOST_CHECK(models.size() > 0);

        // check if the model is acceptably close to ground truth
        StablePose0DModel referenceModel(RPY<>(0.1, 0.45, 0.0).toRotation3D());

        BOOST_CHECK(bestModel.same(referenceModel, 1.0));
    }
}
