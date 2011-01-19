/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/geometry.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Timer.hpp>
#include <string>
#include <fstream>

#include <rw/geometry/Covariance.hpp>

#include <boost/test/unit_test.hpp>

using namespace boost::unit_test;

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::geometry;

const int TRI_MESH_SIZE=10000;

BOOST_AUTO_TEST_CASE( CovarianceTest ){

    PlainTriMeshN1F::Ptr mesh;
    mesh = STLFile::load( testFilePath() + "geoms/FingerMid.stl" );

    IndexedTriMeshN0<float, uint32_t>::Ptr imeshf32 = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float, uint32_t> >(*mesh);

    {
        Covariance<float> covar;
        covar.initialize( imeshf32->getVertices() );
        std::cout << covar.getMatrix() << std::endl;
    }

    Geometry::Ptr geom = GeometryFactory::getGeometry(testFilePath() + "geoms/FingerMid.stl");

    {
        Covariance<float> covar;
        covar.initialize( *geom );
        std::cout << covar.getMatrix() << std::endl;
    }

}

