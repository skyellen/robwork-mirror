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

#include "../TestEnvironment.hpp"

#include <rw/geometry/Triangulate.hpp>

#include <vector>

using namespace rw::math;
using namespace rw::geometry;
using namespace std;

namespace {
class TriangulateTest : public ::testing::Test
{
public:
    int testint;
    vector<Vector2D<double>> contour_square;
    vector<Vector3D<double>> contour_square3D;
    vector<Vector2D<double>> contour_octagon;
    vector<Vector3D<double>> line;
    Triangulate tr;

    virtual void SetUp()
    {

        contour_square.push_back(Vector2D<double>(0.0,0.0));
        contour_square.push_back(Vector2D<double>(2.0,0.0));
        contour_square.push_back(Vector2D<double>(2.0,2.0));
        contour_square.push_back(Vector2D<double>(0.0,2.0));


        contour_square3D.push_back(Vector3D<double>(0.0,0.0,0.0));
        contour_square3D.push_back(Vector3D<double>(2.0,0.0,0.0));
        contour_square3D.push_back(Vector3D<double>(2.0,2.0,0.0));
        contour_square3D.push_back(Vector3D<double>(0.0,2.0,0.0));
        //contour_square3D.push_back(Vector3D<double>(1.0,1.0,1.0));
        //contour_square3D.push_back(Vector3D<double>(1.0,1.0,-1.0));

        line.push_back(Vector3D<double>(0.0,0.0,0.0));
        line.push_back(Vector3D<double>(1.0,1.0,0.0));


        contour_octagon.push_back(Vector2D<double>(1.0,0.0));
        contour_octagon.push_back(Vector2D<double>(2.0,0.0));
        contour_octagon.push_back(Vector2D<double>(3.0,1.0));
        contour_octagon.push_back(Vector2D<double>(3.0,2.0));
        contour_octagon.push_back(Vector2D<double>(2.0,3.0));
        contour_octagon.push_back(Vector2D<double>(1.0,3.0));
        contour_octagon.push_back(Vector2D<double>(0.0,2.0));
        contour_octagon.push_back(Vector2D<double>(0.0,1.0));
    }

};

TEST_F(TriangulateTest, utilityfunctions) {
    // Check area functions
    double area = tr.calcArea(contour_square);
    EXPECT_DOUBLE_EQ(area,4.0);
    area = tr.calcArea(contour_octagon);
    EXPECT_DOUBLE_EQ(area,7.0);

    // Check if points are inside a triangle:
    float Ax,Ay,Bx,By,Cx,Cy;
    Ax = 0;
    Ay = 0;
    Bx = 2;
    By = 0;
    Cx = 2;
    Cy = 2;

    float Insidex = 1.5, Insidey = 0.5;
    EXPECT_TRUE(tr.insideTriangle2D(Ax,Ay,Bx,By,Cx,Cy,Insidex,Insidey));
    Insidex = 0.5; Insidey = 0.5;
    EXPECT_TRUE(tr.insideTriangle2D(Ax,Ay,Bx,By,Cx,Cy,Insidex,Insidey));
    Insidey = 1.6;
    EXPECT_FALSE(tr.insideTriangle2D(Ax,Ay,Bx,By,Cx,Cy,Insidex,Insidey));


    /*// Check snip functions
    int *V = new int[4];
    for (int v=0; v<4; v++) {
        V[v] = v;
    }
    EXPECT_TRUE(tr.snip(contour_square,0,1,2,4,V));


    delete[] V;*/
}


TEST_F(TriangulateTest, processPoints) {
	line.push_back(Vector3D<double>(2.0,2.0,0.0));

    vector<int> result;
    // Triangulate the square:
    ASSERT_TRUE(tr.processPoints(contour_square,result));
    EXPECT_EQ(result.size(),6);


    // Triangle 1
    EXPECT_EQ(result[0],3);
    EXPECT_EQ(result[1],0);
    EXPECT_EQ(result[2],1);
    // Triangle 2
    EXPECT_EQ(result[3],1);
    EXPECT_EQ(result[4],2);
    EXPECT_EQ(result[5],3);

    result.clear();

    // Check that processPoints fails when too few points are used:
    contour_square.erase(contour_square.end());
    contour_square.erase(contour_square.end());
    ASSERT_FALSE(tr.processPoints(contour_square,result));

    // Triangulate the octagon:
    ASSERT_TRUE(tr.processPoints(contour_octagon,result));
    EXPECT_EQ(result.size(),18); // six triangles expected

    // Triangle 1
    EXPECT_EQ(result[0],7);
    EXPECT_EQ(result[1],0);
    EXPECT_EQ(result[2],1);

    // Triangle 2
    EXPECT_EQ(result[3],1);
    EXPECT_EQ(result[4],2);
    EXPECT_EQ(result[5],3);

    // Triangle 3
    EXPECT_EQ(result[6],3);
    EXPECT_EQ(result[7],4);
    EXPECT_EQ(result[8],5);

    // Triangle 4
    EXPECT_EQ(result[9],5);
    EXPECT_EQ(result[10],6);
    EXPECT_EQ(result[11],7);

    // Triangle 5
    EXPECT_EQ(result[12],7);
    EXPECT_EQ(result[13],1);
    EXPECT_EQ(result[14],3);

    // Triangle 6
    EXPECT_EQ(result[15],3);
    EXPECT_EQ(result[16],5);
    EXPECT_EQ(result[17],7);
}

TEST_F(TriangulateTest, processPoints3D) {
    vector<int> result;
    // Triangulate the square:
    ASSERT_TRUE(tr.processPoints(contour_square3D,result));
    EXPECT_EQ(result.size(),6);

    // Triangle 1
    EXPECT_EQ(result[0],3);
    EXPECT_EQ(result[1],0);
    EXPECT_EQ(result[2],1);
    // Triangle 2
    EXPECT_EQ(result[3],1);
    EXPECT_EQ(result[4],2);
    EXPECT_EQ(result[5],3);

    result.clear();

    // Check if processPoints fails when points are closer than precision
    ASSERT_THROW(tr.processPoints(contour_square3D,result,0.0005,10.0),rw::common::Exception);



    // Check if processPoints fails when too few points is supplied:
    ASSERT_FALSE(tr.processPoints(line,result));
    line.push_back(Vector3D<double>(2.0,2.0,0.0));

    // Check if processPoints fails when edges of the contour are colinear:
    ASSERT_THROW(tr.processPoints(line,result,1,0.0001),rw::common::Exception);

}
}

