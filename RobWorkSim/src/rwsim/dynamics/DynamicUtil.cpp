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

#include "DynamicUtil.hpp"

#include <stack>

//#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rwsim/dynamics/RigidDevice.hpp>
//#include <rw/geometry/Geometry.hpp>
//#include <rw/geometry/TriMesh.hpp>
//#include <rw/geometry/PlainTriMesh.hpp>

#include <boost/foreach.hpp>

#include "DynamicWorkCell.hpp"
#include "RigidBody.hpp"

using namespace rw;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rwsim::dynamics;
/*
 * Locates all frames that are staticly connected to the frame and
 * that has geometry information
 */
std::vector<Frame*> DynamicUtil::getAnchoredFrames(Frame &frame, const State &state){
    // first locate the independant parent everything else than fixed
    Frame *parent = &frame;

    // if the current frame is the independent parent then use that
    if( dynamic_cast<FixedFrame*>(parent)==NULL ){
        return getAnchoredChildFrames(parent,state);
    }

    // else search for the independent parent
    for(; parent->getParent(state)!=NULL; parent = parent->getParent(state)){
        if( dynamic_cast<FixedFrame*>(parent)==NULL )
            break;
    }
    return getAnchoredChildFrames(parent,state);
}

std::vector<Frame*> DynamicUtil::getAnchoredChildFrames(Frame *parent, const State &state){
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

            if( dynamic_cast<FixedFrame*>(f) ){
                fstack.push(f);
                res.push_back(f);
            }
        }
    }
    return res;
}

std::vector<Frame*> DynamicUtil::getAnchoredChildFrames(Frame *initparent,
                                                        const State &state,
                                                        const std::vector<Frame*>& exclude){
    // next follow all children and locate fixed anchored frames
    std::vector<Frame*> res;
    std::stack<Frame*> fstack;

    fstack.push(initparent);

    //res.push_back(&frame);
    res.push_back(initparent);

    while( !fstack.empty() ){
        Frame *parent = fstack.top();
        fstack.pop();
        Frame::iterator_pair pairiter = parent->getChildren(state);
        for(; pairiter.first!=pairiter.second; ++pairiter.first ){
            Frame *f = &(*(pairiter.first));

            // also check if frame has geometric properties attached
            //bool hasGeo = rw::models::Accessor::collisionModelInfo().has(*f);

            if( dynamic_cast<FixedFrame*>(f) ){
                size_t i;
                for(i=0;i<exclude.size();i++)
                    if(exclude[i]!=initparent && exclude[i]==f)
                        break;
                if(i!=exclude.size())
                    continue;

                fstack.push(f);
                //if( hasGeo )
                res.push_back(f);
            }
        }
    }
    return res;
}

#ifdef oldold
std::pair<rw::math::Vector3D<>, rw::math::InertiaMatrix<> >
DynamicUtil::estimateInertia(
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
/*
rw::math::Vector3D<>
DynamicUtil::estimateCOG(
     double mass,
     const std::vector<Geometry*> &geoms)
{
    // first find center mass
    double totalArea(0);
    Vector3D<> center(0.f,0.f,0.f);
    BOOST_FOREACH(Geometry *geom, geoms){
        GeometryDataPtr gdata = geom->getGeometryData();
        // check if type of geom is really a trimesh
        if( !dynamic_cast<TriMesh*>(gdata.get()) ){
            continue;
        }
        TriMesh *trimesh = dynamic_cast<TriMesh*>(gdata.get());

        Transform3D<> t3d = geom->getTransform();
        for(int i=0; i<trimesh->size(); i++){
            TriangleN0<double> tri = trimesh->getTriangle(i);
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
*/
/*
std::pair<Vector3D<>, InertiaMatrix<> >
DynamicUtil::estimateInertiaCOG(
    double mass,
    const std::vector<Geometry*>& geoms,
    const Transform3D<>& ref)
{
    Vector3D<float> center = cast<float>( ref * estimateCOG(mass, geoms) );
    Transform3D<> nref = ref;
    nref.P() -= cast<double>(center);
    InertiaMatrix<> inertia = estimateInertia(mass, geoms, nref);
    return std::make_pair(cast<double>(center),inertia);
}

rw::math::InertiaMatrix<>
DynamicUtil::estimateInertia(
    double mass,
    const std::vector<Geometry*>& geoms,
    const Transform3D<>& ref)
{
    double Ixx = 0, Iyy=0, Izz = 0; // the diagonal elements
    double Ixy = 0, Ixz=0, Iyz = 0; // the off diagonal elements
    int triCnt = 0;
    BOOST_FOREACH(Geometry *geom, geoms){
        GeometryDataPtr gdata = geom->getGeometryData();
        // check if type of geom is really a trimesh
        if( !dynamic_cast<TriMesh*>(gdata.get()) ){
            continue;
        }
        TriMesh *trimesh = dynamic_cast<TriMesh*>(gdata.get());

        Transform3D<> t3d = ref*geom->getTransform();

        triCnt += trimesh->size();
        for(int i=0; i<trimesh->size(); i++){
            TriangleN0<double> tri = trimesh->getTriangle(i);
            const Vector3D<>& p = t3d* (tri[0]);
            const Vector3D<>& q = t3d* (tri[1]);
            const Vector3D<>& r = t3d* (tri[2]);

            // calc triangle area
            double a = (cross( p-q , p-r )).norm2()/2;

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
*/
#ifdef OLDOLD
InertiaMatrix<>
    DynamicUtil::estimateInertia(
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
std::vector<RigidBody::Ptr> DynamicUtil::getRigidBodies(DynamicWorkCell& dwc){
	using namespace dynamics;
	std::vector<RigidBody::Ptr> bodies = dwc.findBodies<RigidBody>();
    return bodies;
}

bool DynamicUtil::isResting(DynamicDevice::Ptr dev, const rw::kinematics::State& state, double max_linjointvel, double max_angjointvel){
    if(RigidDevice *rdev = dynamic_cast<RigidDevice*>(dev.get())){
        std::vector<Joint*> rjoints = rdev->getJointDevice()->getJoints();
        Q vel = rdev->getJointVelocities(state);
        //std::cout << vel << std::endl;
        RW_ASSERT_MSG(vel.size()<=rjoints.size(), vel.size() << "<=" << rjoints.size());
        int depOffset = 0;
        for(size_t i=0; i<rjoints.size();i++){
            if( dynamic_cast<rw::models::RevoluteJoint*>(rjoints[i]) ){
                if(max_angjointvel<vel[i-depOffset])
                    return false;
            } else if(dynamic_cast<rw::models::PrismaticJoint*>(rjoints[i])){
                if(max_linjointvel<vel[i-depOffset])
                    return false;
            } else {
                depOffset++;
            }
        }

    }
    return true;
}


bool DynamicUtil::isResting(DynamicWorkCell::Ptr dwc, const rw::kinematics::State& state, double max_lin, double max_ang, double max_jointvel){
    // first check all rigid bodies
    std::vector<RigidBody::Ptr> bodies = dwc->findBodies<RigidBody>();
    BOOST_FOREACH(RigidBody::Ptr rbody, bodies){
        Vector3D<> avel = rbody->getAngVel(state);
        if(MetricUtil::norm2(avel)>max_ang)
            return false;
        Vector3D<> lvel = rbody->getLinVel(state);
        if(MetricUtil::norm2(lvel)>max_lin)
            return false;
    }

    std::vector<DynamicDevice::Ptr> devices = dwc->getDynamicDevices();
    BOOST_FOREACH(DynamicDevice::Ptr dev, devices){
        if(!DynamicUtil::isResting(dev, state, max_lin, max_jointvel) )
            return false;
    }
    return true;
}

/*
rw::math::Q DynamicUtil::computeTorques(
		const State& defstate,
		const rw::math::Q& q,
		const rw::math::Q& dq,
		const rw::math::Q& ddq,
		dynamics::RigidDevice::Ptr dev, const rw::math::Vector3D<>& gravity=rw::math::Vector3D<>(0,0,-9.82))
{
	State state = defstate;
	std::vector<Body::Ptr> links = dev->getLinks();
	std::vector<Joint*> joints = dev->getJointDevice()->getJoints();
	// outward iterations
	//std::vector<Transform3D<> >

	for(int i=0;i<links.size();i++){

		R // rotation from joint i to i+1
		w = R*wi +


	}


}

*/
