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

#include "GeometryUtil.hpp"

#include <stack>

#include <rw/models/Accessor.hpp>
//#include <rw/geometry/FaceArrayFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/math/MetricUtil.hpp>
#include "Geometry.hpp"
#include "TriMesh.hpp"
#include "PlainTriMesh.hpp"
#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

using namespace rw::geometry;

/*
 * Locates all frames that are staticly connected to the frame and
 * that has geometry information
 */
std::vector<Frame*> GeometryUtil::getAnchoredFrames(Frame &frame, const State &state){
    // first locate the independant parent everything else than fixed
    Frame *parent = &frame;

    // if the current frame is the independent parent then use that
    kinematics::FrameType type = kinematics::FrameType::FixedFrame;
    if( rw::models::Accessor::frameType().has(*parent) ){
    	if( rw::models::Accessor::frameType().get(*parent).get() != kinematics::FrameType::FixedFrame){
    		return getAnchoredChildFrames(parent,state);
    	}
    }

    // else search for the independent parent
    for(; parent->getParent(state)!=NULL; parent = parent->getParent(state)){
        bool hasType = rw::models::Accessor::frameType().has(*parent);
        type = kinematics::FrameType::FixedFrame;
        if( hasType )
             type = rw::models::Accessor::frameType().get(*parent);
        if( type.get() != kinematics::FrameType::FixedFrame )
            break;
    }
    return getAnchoredChildFrames(parent,state);
}

std::vector<Frame*> GeometryUtil::getAnchoredChildFrames(Frame *parent, const State &state){
    // next follow all children and locate fixed anchored frames
    std::vector<Frame*> res;
    std::stack<Frame*> fstack;

	fstack.push(parent);

    //if( rw::models::Accessor::collisionModelInfo().has(*parent) )
	//res.push_back(&frame);
    res.push_back(parent);

    while( !fstack.empty() ){
        parent = fstack.top();
        fstack.pop();
        Frame::iterator_pair pairiter = parent->getChildren(state);
        for(; pairiter.first!=pairiter.second; ++pairiter.first ){
            Frame *f = &(*(pairiter.first));

            kinematics::FrameType type = kinematics::FrameType::FixedFrame;
            bool hasType = rw::models::Accessor::frameType().has(*f);
            type = kinematics::FrameType::FixedFrame;
            if( hasType )
                 type = rw::models::Accessor::frameType().get(*f);

            // also check if frame has geometric properties attached
            //bool hasGeo = rw::models::Accessor::collisionModelInfo().has(*f);

            if( type.get() == kinematics::FrameType::FixedFrame ){
                fstack.push(f);
                //if( hasGeo )
                res.push_back(f);
            }
        }
    }
    return res;
}

#ifdef OLD_FACE_STUFF
std::pair<rw::math::Vector3D<>, rw::math::InertiaMatrix<> >
GeometryUtil::estimateInertia(
    double mass,
    rw::kinematics::Frame &refframe,
    const State &state)
{
    typedef std::vector<geometry::Face<float> > TriMesh;
    std::vector<std::pair<Transform3D<>, TriMesh > > meshes;
	std::vector<Frame*> frames = getAnchoredFrames(refframe, state);
	// first calculate center
	Vector3D<> center(0,0,0);
	BOOST_FOREACH(const Frame *frame, frames){
	    if( frame==NULL )
	        continue;
	    // check if frame has collision descriptor
	    if( !Accessor::collisionModelInfo().has(*frame) )
	        continue;

	    Transform3D<> pTf = Kinematics::frameTframe(&refframe, frame, state);
	    std::string geofile = Accessor::collisionModelInfo().get(*frame)[0].getId();
	    Transform3D<> fTgeo = pTf * Accessor::collisionModelInfo().get(*frame)[0].getTransform();

	    meshes.push_back(std::make_pair(fTgeo,TriMesh()) );
	    // get the geo descriptor
	    if( !geometry::FaceArrayFactory::loadFaceArrayFile(geofile,meshes.back().second) )
	        continue;

	    for (size_t i=0;i<meshes.back().second.size();i++){
	        geometry::Face<float> &f = meshes.back().second[i];
	        Vector3D<> v1(f._vertex1[0],f._vertex1[1],f._vertex1[2]);
	        Vector3D<> v2(f._vertex2[0],f._vertex2[1],f._vertex2[2]);
	        Vector3D<> v3(f._vertex3[0],f._vertex3[1],f._vertex3[2]);
	        double area = (cross( v1-v2 , v1-v3 )).norm2()/2;
	        Vector3D<> centroid = ((fTgeo * v1 + fTgeo * v2 + fTgeo * v3)/3) * area;
	        center += centroid;
	    }
	    center /= meshes.back().second.size();
	}

	InertiaMatrix<> itotal(0,0,0);

	for(size_t i=0; i<meshes.size(); i++){
	    Transform3D<> t3d = meshes[i].first;
	    t3d.P() += center;
		InertiaMatrix<> inertia = estimateInertia(mass, meshes[i].second, t3d);
		itotal = InertiaMatrix<>( itotal.m() + inertia.m() );
	}
	return std::make_pair(center, itotal);
}
#endif

rw::math::Vector3D<>
GeometryUtil::estimateCOG(const std::vector<GeometryPtr> &geoms)
{
    // first find center mass
    double totalArea(0);
    Vector3D<> center(0.f,0.f,0.f);
    BOOST_FOREACH(GeometryPtr geom, geoms){
        GeometryDataPtr gdata = geom->getGeometryData();
        // check if type of geom is really a trimesh
        if( !dynamic_cast<TriMesh*>(gdata.get()) ){
            continue;
        }
        TriMesh *trimesh = dynamic_cast<TriMesh*>(gdata.get());

        Transform3D<> t3d = geom->getTransform();
        for(size_t i=0; i<trimesh->getSize(); i++){
            Triangle<double> tri = trimesh->getTriangle(i);
            const Vector3D<>& p = t3d* (tri[0]);
            const Vector3D<>& q = t3d* (tri[1]);
            const Vector3D<>& r = t3d* (tri[2]);

            // calc triangle area
            double a = (cross( p-q , p-r )).norm2()/2;

            // calc triangle centroid
            Vector3D<> c = (p+q+r)/3;

            center += c*a;
            totalArea += a;
        }
    }
    center /= totalArea;
    return cast<double>(center);
}

double GeometryUtil::calcMaxDist(const std::vector<GeometryPtr> &geoms,
                                 const rw::math::Vector3D<> center)
{
    double maxDist = 0;

    // first find center mass
    BOOST_FOREACH(GeometryPtr geom, geoms){
        GeometryDataPtr gdata = geom->getGeometryData();
        // check if type of geom is really a trimesh
        if( !dynamic_cast<TriMesh*>(gdata.get()) ){
            continue;
        }
        TriMesh *trimesh = dynamic_cast<TriMesh*>(gdata.get());

        Transform3D<> t3d = geom->getTransform();
        for(size_t i=0; i<trimesh->getSize(); i++){
            Triangle<double> tri = trimesh->getTriangle(i);
            const Vector3D<>& p = t3d* (tri[0]);
            const Vector3D<>& q = t3d* (tri[1]);
            const Vector3D<>& r = t3d* (tri[2]);
            maxDist = std::max(MetricUtil::dist2(center,p), maxDist);
            maxDist = std::max(MetricUtil::dist2(center,q), maxDist);
            maxDist = std::max(MetricUtil::dist2(center,r), maxDist);
        }
    }
    return maxDist;
}


std::pair<Vector3D<>, InertiaMatrix<> >
GeometryUtil::estimateInertiaCOG(
    double mass,
    const std::vector<GeometryPtr>& geoms,
    const Transform3D<>& ref)
{
    Vector3D<float> center = cast<float>( ref * estimateCOG( geoms ) );
    Transform3D<> nref = ref;
    nref.P() -= cast<double>(center);
    InertiaMatrix<> inertia = estimateInertia(mass, geoms, nref);
    return std::make_pair(cast<double>(center),inertia);
}

rw::math::InertiaMatrix<>
GeometryUtil::estimateInertia(
    double mass,
    const std::vector<GeometryPtr>& geoms,
    const Transform3D<>& ref)
{
    double Ixx = 0, Iyy=0, Izz = 0; // the diagonal elements
    double Ixy = 0, Ixz=0, Iyz = 0; // the off diagonal elements
    int triCnt = 0;
    BOOST_FOREACH(GeometryPtr geom, geoms){
        GeometryDataPtr gdata = geom->getGeometryData();
        // check if type of geom is really a trimesh
        if( !dynamic_cast<TriMesh*>(gdata.get()) ){
            continue;
        }
        TriMesh *trimesh = dynamic_cast<TriMesh*>(gdata.get());

        Transform3D<> t3d = ref*geom->getTransform();

        triCnt += trimesh->getSize();
        for(size_t i=0; i<trimesh->getSize(); i++){
            Triangle<double> tri = trimesh->getTriangle(i);
            const Vector3D<>& p = t3d* (tri[0]);
            const Vector3D<>& q = t3d* (tri[1]);
            const Vector3D<>& r = t3d* (tri[2]);

            // calc triangle area
            //double a = (cross( p-q , p-r )).norm2()/2;

            // calc triangle centroid
            Vector3D<> c = (p+q+r)/3;
            //Vector3D<float> pos = c*a;
            Vector3D<> pos = c;
            double x = pos(0), y = pos(1), z = pos(2);
            Ixx += y*y + z*z;
            Iyy += x*x + z*z;
            Izz += x*x + y*y;
            Ixy += fabs(x*y);
            Ixz += fabs(x*z);
            Iyz += fabs(y*z);
        }
    }
    //std::cout << "Total Area: " << totalArea << " mass: " << mass << std::endl;
    //std::cout << "Center: " << center <<  std::endl;
    //double ptMass = mass/totalArea;
    double ptMass = mass/triCnt;
    Ixx *= ptMass;
    Iyy *= ptMass;
    Izz *= ptMass;

    double Iyx = Ixy = -ptMass*Ixy;
    double Izx = Ixz = -ptMass*Ixz;
    double Izy = Iyz = -ptMass*Iyz;

    InertiaMatrix<> inertia( Ixx, Ixy, Ixz,
                            Iyx, Iyy, Iyz,
                            Izx, Izy, Izz);
    return inertia;
}

#ifdef OLD_FACE_STUFF
InertiaMatrix<>
    GeometryUtil::estimateInertia(
        double mass,
        const std::vector<geometry::Face<float> >& faces,
        const Transform3D<>& t3d)
{
	size_t faceCnt = faces.size();
	double Ixx = 0, Iyy=0, Izz = 0; // the diagonal elements
	double Ixy = 0, Ixz=0, Iyz = 0; // the off diagonal elements
	for(size_t i=0; i<faceCnt; i++){
		const geometry::Face<float> &face = faces[i];
/*		float x = (face._vertex1[0] + face._vertex2[0] + face._vertex3[0])/3.0;
		float y = (face._vertex1[1] + face._vertex2[1] + face._vertex3[1])/3.0;
		float z = (face._vertex1[2] + face._vertex2[2] + face._vertex3[2])/3.0;
	*/
		float x = face._vertex1[0];
		float y = face._vertex1[1];
		float z = face._vertex1[2];
		Vector3D<> pos = t3d*Vector3D<>(x,y,z);
		x = pos(0); y = pos(1); z = pos(2);
		Ixx += y*y + z*z;
		Iyy += x*x + z*z;
		Izz += x*x + y*y;
		Ixy += x*y;
		Ixz += x*z;
		Iyz += y*z;
	}
	double ptMass = mass/faceCnt;
	Ixx *= ptMass;
	Iyy *= ptMass;
	Izz *= ptMass;
	double Iyx = Ixy = -ptMass*Ixy;
	double Izx = Ixz = -ptMass*Ixz;
	double Izy = Iyz = -ptMass*Iyz;
	return InertiaMatrix<>( Ixx, Ixy, Ixz,
							Iyx, Iyy, Iyz,
							Izx, Izy, Izz);
}

#endif
