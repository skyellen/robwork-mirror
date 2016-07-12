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

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Cone.hpp>
#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Pyramid.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Tube.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/State.hpp>

#if RW_HAVE_ASSIMP
#include <rw/loaders/model3d/LoaderAssimp.hpp>
using rw::graphics::Model3D;
using rw::loaders::LoaderAssimp;
#else
#include <rw/loaders/model3d/STLFile.hpp>
using rw::loaders::STLFile;
#endif

using rw::common::ownedPtr;
using namespace rw::geometry;
using rw::kinematics::State;
using namespace rw::math;

BOOST_AUTO_TEST_CASE( EstimateVolumeTest ){
	{
		// Sphere test
		const double radius = 0.02;
		const Sphere sphere(radius);
		const double volEst = GeometryUtil::estimateVolume(*sphere.createMesh(0));
		BOOST_CHECK_CLOSE(volEst,4.*Pi*radius*radius*radius/3.,0.2);
	}
	{
		// Box test
		const double x = 0.02;
		const double y = 0.03;
		const double z = 0.04;
		const Box box(x,y,z);
		const double volEst = GeometryUtil::estimateVolume(*box.createMesh(0));
		BOOST_CHECK_CLOSE(volEst,x*y*z,1e-5);
	}
	{
		// Cylinder test
		const double radius = 0.02;
		const double height = 0.1;
		const Cylinder cyl(radius, height);
		const double volEst = GeometryUtil::estimateVolume(*cyl.createMesh(26));
		BOOST_CHECK_CLOSE(volEst,Pi*radius*radius*height,1);
	}
	{
		// Pyramid test
		const double x = 0.02;
		const double y = 0.03;
		const double height = 0.1;
		const Pyramid pyramid(x, y, height);
		const double volEst = GeometryUtil::estimateVolume(*pyramid.createMesh(0));
		BOOST_CHECK_CLOSE(volEst,height*x*y/3.,1e-5);
	}
	{
		// Cone test
		const double height = 0.1;
		const double rTop = 0;
		const double rBot = 0.06;
		const Cone cone(height, rTop, rBot);
		const double volEst = GeometryUtil::estimateVolume(*cone.createMesh(26));
		BOOST_CHECK_CLOSE(volEst,Pi*height*(rBot*rBot+rTop*rTop+rBot*rTop)/3.,1);
	}
	{
		// Tube test
		const double radius = 0.02;
		const double thickness = 0.005;
		const double height = 0.1;
		const Tube tube(radius, thickness, height);
		const double volEst = GeometryUtil::estimateVolume(*tube.createMesh(26));
		BOOST_CHECK_CLOSE(volEst,Pi*height*((radius+thickness)*(radius+thickness)-radius*radius),1);
	}
	{
		// STL table test
		// Top box dimensions: {2,2,0.1}
		// Top box center: {0,0,1.05}
		// Leg dimensions: {1,0.1,0.1}
		// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
		const double volTop = 0.1*2*2;
		const double volLeg = 1.*0.1*0.1;
		const double volTot = volTop+4.*volLeg;

		{
			const std::string file = testFilePath() + "geoms/table.stl";
#if RW_HAVE_ASSIMP
			LoaderAssimp loader;
			const Model3D::Ptr model = loader.load(file);
			const GeometryData::Ptr gdata = model->toGeometryData();
			const TriMesh::Ptr stl = gdata.cast<TriMesh>();
			BOOST_REQUIRE(stl != NULL);
#else
			const TriMesh::Ptr stl = STLFile::load(file);
#endif
			const double volEst = GeometryUtil::estimateVolume(*stl);
			BOOST_CHECK_CLOSE(volEst,volTot,2e-5);
		}

		{
			const std::string file = testFilePath() + "geoms/table_union.stl";
#if RW_HAVE_ASSIMP
			LoaderAssimp loader;
			const Model3D::Ptr model = loader.load(file);
			const GeometryData::Ptr gdata = model->toGeometryData();
			const TriMesh::Ptr stl = gdata.cast<TriMesh>();
			BOOST_REQUIRE(stl != NULL);
#else
			const TriMesh::Ptr stl = STLFile::load(file);
#endif
			const double volEst = GeometryUtil::estimateVolume(*stl);
			BOOST_CHECK_CLOSE(volEst,volTot,2e-5);
		}

		// Multiple geometry test (the table again)
		{
			std::vector<Geometry::Ptr> geoms(5);
			const GeometryData::Ptr top = ownedPtr(new Box(2,2,0.1)); // Top box dimensions: {2,2,0.1}
			const GeometryData::Ptr leg = ownedPtr(new Box(0.1,0.1,1)); // Leg dimensions: {0.1,0.1,1}
			geoms[0] = ownedPtr(new Geometry(top,Transform3D<>(Vector3D<>(0,0,1.05))));
			for (std::size_t i = 1; i <= 4; i++) {
				// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
				const double x = (i <= 2) ? -0.95 : 0.95;
				const double y = (i%2) ? -0.95 : 0.95;
				geoms[i] = ownedPtr(new Geometry(leg,Transform3D<>(Vector3D<>(x,y,0.5))));
			}
			const double volEst = GeometryUtil::estimateVolume(geoms);
			BOOST_CHECK_CLOSE(volEst,volTot,2e-5);
		}
	}
}

BOOST_AUTO_TEST_CASE( EstimateCOGTest ){
	{
		// Sphere test
		const double radius = 0.02;
		const Sphere sphere(radius);
		const Vector3D<> cogEst = GeometryUtil::estimateCOG(*sphere.createMesh(0));
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::z()),std::numeric_limits<double>::epsilon());
	}
	{
		// Box test
		const double x = 0.02;
		const double y = 0.03;
		const double z = 0.04;
		const Box box(x,y,z);
		const Vector3D<> cogEst = GeometryUtil::estimateCOG(*box.createMesh(0));
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::z()),std::numeric_limits<double>::epsilon());
	}
	{
		// Cylinder test
		const double radius = 0.02;
		const double height = 0.1;
		const Cylinder cyl(radius, height);
		const Vector3D<> cogEst = GeometryUtil::estimateCOG(*cyl.createMesh(26));
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::z()),std::numeric_limits<double>::epsilon());
	}
	{
		// Pyramid test
		const double x = 0.02;
		const double y = 0.03;
		const double height = 0.1;
		const Pyramid pyramid(x, y, height);
		const Vector3D<> cogEst = GeometryUtil::estimateCOG(*pyramid.createMesh(0));
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_CLOSE(dot(cogEst,Vector3D<>::z()),1./4.*height,1e-5);
	}
	{
		// Cone test
		const double height = 0.1;
		const double rTop = 0;
		const double rBot = 0.06;
		const Cone cone(height, rTop, rBot);
		const Vector3D<> cogEst = GeometryUtil::estimateCOG(*cone.createMesh(26));
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_CLOSE(dot(cogEst,Vector3D<>::z()),-1./4.*height,1e-5);
	}
	{
		// Tube test
		const double radius = 0.02;
		const double thickness = 0.005;
		const double height = 0.1;
		const Tube tube(radius, thickness, height);
		const Vector3D<> cogEst = GeometryUtil::estimateCOG(*tube.createMesh(26));
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
		BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::z()),std::numeric_limits<double>::epsilon());
	}
	{
		// STL table test
		// Top box dimensions: {2,2,0.1}
		// Top box center: {0,0,1.05}
		// Leg dimensions: {1,0.1,0.1}
		// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
		const double volTop = 0.1*2*2;
		const double volLeg = 1.*0.1*0.1;
		const double volTot = volTop+4.*volLeg;
		const double massTopRatio = volTop/volTot;
		const double massLegRatio = volLeg/volTot;
		const double cogZ = massTopRatio*1.05+4.*massLegRatio*0.5;

		{
			const std::string file = testFilePath() + "geoms/table.stl";
#if RW_HAVE_ASSIMP
			LoaderAssimp loader;
			const Model3D::Ptr model = loader.load(file);
			const GeometryData::Ptr gdata = model->toGeometryData();
			const TriMesh::Ptr stl = gdata.cast<TriMesh>();
			BOOST_REQUIRE(stl != NULL);
#else
			const TriMesh::Ptr stl = STLFile::load(file);
#endif
			const Vector3D<> cogEst = GeometryUtil::estimateCOG(*stl);
			BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
			BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
			BOOST_CHECK_CLOSE(dot(cogEst,Vector3D<>::z()),cogZ,1e-5);
		}

		{
			const std::string file = testFilePath() + "geoms/table_union.stl";
#if RW_HAVE_ASSIMP
			LoaderAssimp loader;
			const Model3D::Ptr model = loader.load(file);
			const GeometryData::Ptr gdata = model->toGeometryData();
			const TriMesh::Ptr stl = gdata.cast<TriMesh>();
			BOOST_REQUIRE(stl != NULL);
#else
			const TriMesh::Ptr stl = STLFile::load(file);
#endif
			const Vector3D<> cogEst = GeometryUtil::estimateCOG(*stl);
			BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
			BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
			BOOST_CHECK_CLOSE(dot(cogEst,Vector3D<>::z()),cogZ,1e-5);
		}

		// Multiple geometry test (the table again)
		{
			std::vector<Geometry::Ptr> geoms(5);
			const GeometryData::Ptr top = ownedPtr(new Box(2,2,0.1)); // Top box dimensions: {2,2,0.1}
			const GeometryData::Ptr leg = ownedPtr(new Box(0.1,0.1,1)); // Leg dimensions: {0.1,0.1,1}
			geoms[0] = ownedPtr(new Geometry(top,Transform3D<>(Vector3D<>(0,0,1.05))));
			for (std::size_t i = 1; i <= 4; i++) {
				// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
				const double x = (i <= 2) ? -0.95 : 0.95;
				const double y = (i%2) ? -0.95 : 0.95;
				geoms[i] = ownedPtr(new Geometry(leg,Transform3D<>(Vector3D<>(x,y,0.5))));
			}
			const Vector3D<> cogEst = GeometryUtil::estimateCOG(geoms);
			BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
			BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
			BOOST_CHECK_CLOSE(dot(cogEst,Vector3D<>::z()),cogZ,1e-5);
		}
	}
}

BOOST_AUTO_TEST_CASE( EstimateInertiaTest ){
	const double mass = 0.5;
	const Transform3D<> ref = Transform3D<>::identity();
	{
		// Sphere test
		const double radius = 0.02;
		const Sphere sphere(radius);
		const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(sphere.createMesh(0)))); // note: sphere ignores granulation
		const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, ref);
		const double I = 2.*mass*radius*radius/5.;
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),I,0.1);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),I,0.1);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),I,0.1);
	}
	{
		// Box test
		const double x = 0.02;
		const double y = 0.03;
		const double z = 0.04;
		const Box box(x,y,z);
		const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(box.createMesh(0)))); // note: box ignores granulation
		const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, ref);
		const double Ix = mass*(y*y+z*z)/12.;
		const double Iy = mass*(x*x+z*z)/12.;
		const double Iz = mass*(x*x+y*y)/12.;
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,1e-5);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Iy,1e-5);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,1e-5);
	}
	{
		// Cylinder test
		const double radius = 0.02;
		const double height = 0.1;
		const Cylinder cyl(radius, height);
		const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(cyl.createMesh(26))));
		const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, ref);
		const double Ix = mass*(3*radius*radius+height*height)/12.;
		const double Iz = mass*radius*radius/2.;
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,0.2);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,0.2);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,1);
	}
	{
		// Pyramid test
		const double x = 0.02;
		const double y = 0.03;
		const double height = 0.1;
		const Pyramid pyramid(x, y, height);
		const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(pyramid.createMesh(0)))); // note: pyramid ignores granulation
		const Transform3D<> cog(-Vector3D<>::z()*height/4.);
		const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, cog);
		const double Ix = mass*(4.*y*y+3.*height*height)/80.;
		const double Iy = mass*(4.*x*x+3.*height*height)/80.;
		const double Iz = mass*(x*x+y*y)/20.;
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,1e-5);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Iy,1e-5);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,1e-5);
	}
	{
		// Cone test
		const double height = 0.1;
		const double rTop = 0;
		const double rBot = 0.06;
		const Cone cone(height, rTop, rBot);
		const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(cone.createMesh(26))));
		const Transform3D<> cog(Vector3D<>::z()*height/4.);
		const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, cog);
		const double Ix = 3.*mass*(4.*rBot*rBot+height*height)/80.;
		const double Iz = 3.*mass*rBot*rBot/10.;
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,0.6);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,0.6);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,1);
	}
	{
		// Tube test
		const double radius = 0.02;
		const double thickness = 0.005;
		const double height = 0.1;
		const Tube tube(radius, thickness, height);
		const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(tube.createMesh(26))));
		const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, ref);
		const double Ix = mass*(3*radius*radius+3*(radius+thickness)*(radius+thickness)+height*height)/12.;
		const double Iz = mass*(radius*radius+(radius+thickness)*(radius+thickness))/2.;
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,0.25);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,0.25);
		BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,1);
	}
	{
		// STL table test
		// Top box dimensions: {2,2,0.1}
		// Top box center: {0,0,1.05}
		// Leg dimensions: {0.1,0.1,1}
		// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
		const double volTop = 0.1*2*2;
		const double volLeg = 1.*0.1*0.1;
		const double volTot = volTop+4.*volLeg;
		const double massTop = mass*volTop/volTot;
		const double massLeg = mass*volLeg/volTot;
		const double cogZ = (massTop*1.05+4.*massLeg*0.5)/mass;
		const double IxTop = massTop*(2*2+0.1*0.1)/12.+massTop*(1.05-cogZ)*(1.05-cogZ);
		const double IzTop = massTop*(2*2+2*2)/12.;
		const double IxLeg = massLeg*(0.1*0.1+1.*1.)/12.+massLeg*((0.5-cogZ)*(0.5-cogZ)+0.95*0.95);
		const double IzLeg = massLeg*(0.1*0.1+0.1*0.1)/12.+massLeg*2*0.95*0.95;
		const double Ix = IxTop+4.*IxLeg;
		const double Iz = IzTop+4.*IzLeg;
		const Transform3D<> cog(-Vector3D<>::z()*cogZ);

		{
			const std::string file = testFilePath() + "geoms/table.stl";
#if RW_HAVE_ASSIMP
			LoaderAssimp loader;
			const Model3D::Ptr model = loader.load(file);
			const GeometryData::Ptr stl = model->toGeometryData();
#else
			const GeometryData::Ptr stl = STLFile::load(file);
#endif
			const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(stl)));
			const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, cog);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,2e-5);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,2e-5);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,2e-5);
		}

		{
			const std::string file = testFilePath() + "geoms/table_union.stl";
#if RW_HAVE_ASSIMP
			LoaderAssimp loader;
			const Model3D::Ptr model = loader.load(file);
			const GeometryData::Ptr stl = model->toGeometryData();
#else
			const GeometryData::Ptr stl = STLFile::load(file);
#endif
			const std::vector<Geometry::Ptr> geoms(1,ownedPtr(new Geometry(stl)));
			const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, cog);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,2e-5);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,2e-5);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,2e-5);
		}

		// Multiple geometry test (the table again)
		{
			std::vector<Geometry::Ptr> geoms(5);
			const GeometryData::Ptr top = ownedPtr(new Box(2,2,0.1)); // Top box dimensions: {2,2,0.1}
			const GeometryData::Ptr leg = ownedPtr(new Box(0.1,0.1,1)); // Leg dimensions: {0.1,0.1,1}
			geoms[0] = ownedPtr(new Geometry(top,Transform3D<>(Vector3D<>(0,0,1.05))));
			for (std::size_t i = 1; i <= 4; i++) {
				// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
				const double x = (i <= 2) ? -0.95 : 0.95;
				const double y = (i%2) ? -0.95 : 0.95;
				geoms[i] = ownedPtr(new Geometry(leg,Transform3D<>(Vector3D<>(x,y,0.5))));
			}
			const InertiaMatrix<> inertiaEst = GeometryUtil::estimateInertia(mass, geoms, cog);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,2e-5);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,2e-5);
			BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,2e-5);
		}
	}
}

BOOST_AUTO_TEST_CASE( EstimateInertiaCOGTest ){
	const double mass = 0.5;
	const Transform3D<> ref = Transform3D<>::identity();

	// STL table test
	// Top box dimensions: {2,2,0.1}
	// Top box center: {0,0,1.05}
	// Leg dimensions: {0.1,0.1,1}
	// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
	const double volTop = 0.1*2*2;
	const double volLeg = 1.*0.1*0.1;
	const double volTot = volTop+4.*volLeg;
	const double massTop = mass*volTop/volTot;
	const double massLeg = mass*volLeg/volTot;
	const double cogZ = (massTop*1.05+4.*massLeg*0.5)/mass;
	const double IxTop = massTop*(2*2+0.1*0.1)/12.+massTop*(1.05-cogZ)*(1.05-cogZ);
	const double IzTop = massTop*(2*2+2*2)/12.;
	const double IxLeg = massLeg*(0.1*0.1+1.*1.)/12.+massLeg*((0.5-cogZ)*(0.5-cogZ)+0.95*0.95);
	const double IzLeg = massLeg*(0.1*0.1+0.1*0.1)/12.+massLeg*2*0.95*0.95;
	const double Ix = IxTop+4.*IxLeg;
	const double Iz = IzTop+4.*IzLeg;

	std::vector<Geometry::Ptr> geoms(5);
	const GeometryData::Ptr top = ownedPtr(new Box(2,2,0.1)); // Top box dimensions: {2,2,0.1}
	const GeometryData::Ptr leg = ownedPtr(new Box(0.1,0.1,1)); // Leg dimensions: {0.1,0.1,1}
	geoms[0] = ownedPtr(new Geometry(top,Transform3D<>(Vector3D<>(0,0,1.05))));
	for (std::size_t i = 1; i <= 4; i++) {
		// Leg positions: {+/- 0.95, +/- 0.95, 0.5}
		const double x = (i <= 2) ? -0.95 : 0.95;
		const double y = (i%2) ? -0.95 : 0.95;
		geoms[i] = ownedPtr(new Geometry(leg,Transform3D<>(Vector3D<>(x,y,0.5))));
	}
	State state; // just dummy variable - not used
	const std::pair<Vector3D<>, InertiaMatrix<> > est = GeometryUtil::estimateInertiaCOG(mass, geoms, NULL, state, ref);
	const InertiaMatrix<>& inertiaEst = est.second;
	const Vector3D<>& cogEst = est.first;
	BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::x()),std::numeric_limits<double>::epsilon());
	BOOST_CHECK_SMALL(dot(cogEst,Vector3D<>::y()),std::numeric_limits<double>::epsilon());
	BOOST_CHECK_CLOSE(dot(cogEst,Vector3D<>::z()),cogZ,1e-5);
	BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::x(),Vector3D<>::x()),Ix,2e-5);
	BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::y(),Vector3D<>::y()),Ix,2e-5);
	BOOST_CHECK_CLOSE(dot(inertiaEst*Vector3D<>::z(),Vector3D<>::z()),Iz,2e-5);
}
