/*
 * RenderUtil.cpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */

#include "RenderUtil.hpp"

#include <rw/geometry/Line.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rwlibs::opengl;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

rw::common::Ptr<RenderLines> RenderUtil::makeCameraViewRender(
		double w, double h, double fovy,
		double vnear, double vfar)
{
	// calculate  aspect and fovy
	double aspect = w/h;
	double fovyhalf = fovy * Deg2Rad * 0.5;
	double z = -vfar;
	double y = tan(fovyhalf)*z;
	double x = y*aspect;
	double ynear = tan(fovyhalf)*vnear;
	double xnear = ynear*aspect;

	// the 4 lines shooting from pinhole
	std::vector<Line> lines;
	lines.push_back(Line(Vector3D<> (  0 ,  0 ,  0 ),
					  	 Vector3D<> (  x ,  y ,  z )));
	lines.push_back(Line(Vector3D<> (  0 ,  0 ,  0 ),
						Vector3D<> (  x , -y ,  z )));

	lines.push_back(Line(Vector3D<> (  0 ,  0 ,  0 ),
						Vector3D<> ( -x ,  y ,  z )));
	lines.push_back(Line(Vector3D<> (  0 ,  0 ,  0 ),
						Vector3D<> ( -x , -y ,  z )));

	// draw far clip
	lines.push_back(Line(Vector3D<> (   x ,  y ,  z ),
						Vector3D<> (  -x ,  y ,  z )));
	lines.push_back(Line(Vector3D<> (   x , -y ,  z ),
						Vector3D<> (   x ,  y ,  z )));

	lines.push_back(Line(Vector3D<> (   x ,  -y ,  z ),
						Vector3D<> (  -x ,  -y ,  z )));
	lines.push_back(Line(Vector3D<> (  -x ,  -y ,  z ),
						Vector3D<> (  -x ,   y ,  z )));

	// draw near clip
	lines.push_back(Line(Vector3D<> (  xnear ,  ynear ,  -vnear ),
						Vector3D<> ( -xnear ,  ynear ,  -vnear )));
	lines.push_back(Line(Vector3D<> (  xnear , -ynear ,  -vnear ),
						Vector3D<> (  xnear ,  ynear ,  -vnear )));

	lines.push_back(Line(Vector3D<> (  xnear ,  -ynear ,  -vnear ),
						Vector3D<> ( -xnear ,  -ynear ,  -vnear )));
	lines.push_back(Line(Vector3D<> ( -xnear ,  -ynear ,  -vnear ),
						Vector3D<> ( -xnear ,   ynear ,  -vnear )));

	return ownedPtr( new RenderLines(lines) );
}

rw::common::Ptr<RenderLines> makeWorldGridRender(float size, float resolution)
{
	float halfsize = size/2;
	std::vector<Line> lines;

    for (size_t i=0; i*resolution <= halfsize; i++){
        lines.push_back(Line(Vector3D<>(halfsize,  resolution*i , 0 ),
        					Vector3D<>( -halfsize,  resolution*i , 0 )));
        lines.push_back(Line(Vector3D<>(  halfsize, -resolution*i , 0 ),
        					Vector3D<>( -halfsize, -resolution*i , 0 )));
        lines.push_back(Line(Vector3D<>(  resolution*i,  halfsize , 0 ),
        					Vector3D<>(  resolution*i, -halfsize , 0 )));
        lines.push_back(Line(Vector3D<>( -resolution*i,  halfsize , 0 ),
        					Vector3D<>( -resolution*i, -halfsize , 0 )));
    }

    return ownedPtr( new RenderLines(lines) );
}
