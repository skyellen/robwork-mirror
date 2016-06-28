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
#include "Geometry.hpp"
#include "TriMesh.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/math/MetricUtil.hpp>

#include <boost/foreach.hpp>

#include <stack>

using namespace rw;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;

namespace {
// Code based on Brian Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties" (1995)
// journal of graphics tools, volume 1, number 1, 1996.

struct ProjectionIntegrals {
	ProjectionIntegrals():
		P1(0), Pa(0), Pb(0), Paa(0), Pab(0), Pbb(0), Paaa(0), Paab(0), Pabb(0), Pbbb(0)
	{
	}
	double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
};

struct FaceIntegrals {
	FaceIntegrals():
		Fa(0), Fb(0), Fc(0), Faa(0), Fbb(0), Fcc(0), Faaa(0), Fbbb(0), Fccc(0), Faab(0), Fbbc(0), Fcca(0)
	{
	}
	double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
};

/* compute various integrations over projection of face */
ProjectionIntegrals compProjectionIntegrals(const Triangle<double>& f, const Transform3D<>& t3d, bool noInertia = false)
{
	ProjectionIntegrals res;

	double a0, a1, da;
	double b0, b1, db;
	double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
	double a1_2, a1_3, b1_2, b1_3;
	double C1, Ca, Caa, Cb, Cbb;
	double Cab, Kab;

	// Only used with inertia
	double Caaa, Cbbb;
	double Caab, Kaab, Cabb, Kabb;

	const Vector3D<> n = t3d.R()*f.calcFaceNormal();
	const Vector3D<> nabs = Math::abs(n);
	std::size_t A, B, C;
	if (nabs[0] > nabs[1] && nabs[0] > nabs[2]) C = 0;
	else C = (nabs[1] > nabs[2]) ? 1 : 2;
	A = (C + 1) % 3;
	B = (A + 1) % 3;

	for (int i = 0; i < 3; i++) {
		a0 = (t3d*f.getVertex(i))[A];
		b0 = (t3d*f.getVertex(i))[B];
		a1 = (t3d*f.getVertex((i+1) % 3))[A];
		b1 = (t3d*f.getVertex((i+1) % 3))[B];
		da = a1 - a0;
		db = b1 - b0;
		a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
		b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
		a1_2 = a1 * a1; a1_3 = a1_2 * a1;
		b1_2 = b1 * b1; b1_3 = b1_2 * b1;

		C1 = a1 + a0;
		Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3;
		Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3;
		Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;

		res.P1 += db*C1;
		res.Pa += db*Ca;
		res.Paa += db*Caa;
		res.Pb += da*Cb;
		res.Pbb += da*Cbb;
		res.Pab += db*(b1*Cab + b0*Kab);

		if (!noInertia) {
			Caaa = a1*Caa + a0_4;
			Cbbb = b1*Cbb + b0_4;
			Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
			Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
			Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

			res.Paaa += db*Caaa;
			res.Pbbb += da*Cbbb;
			res.Paab += db*(b1*Caab + b0*Kaab);
			res.Pabb += da*(a1*Cabb + a0*Kabb);
		}
	}

	res.P1 /= 2.0;
	res.Pa /= 6.0;
	res.Paa /= 12.0;
	res.Pb /= -6.0;
	res.Pbb /= -12.0;
	res.Pab /= 24.0;

	if (!noInertia) {
		res.Paaa /= 20.0;
		res.Pbbb /= -20.0;
		res.Paab /= 60.0;
		res.Pabb /= -60.0;
	}

	return res;
}

FaceIntegrals compFaceIntegrals(const Triangle<double>& f, const Transform3D<>& t3d, bool noInertia = false)
{
	FaceIntegrals res;

	const ProjectionIntegrals p = compProjectionIntegrals(f, t3d, noInertia);

	const Vector3D<> n = t3d.R()*f.calcFaceNormal();
	const Vector3D<> nabs = Math::abs(n);
	std::size_t A, B, C;
	if (nabs[0] > nabs[1] && nabs[0] > nabs[2]) C = 0;
	else C = (nabs[1] > nabs[2]) ? 1 : 2;
	A = (C + 1) % 3;
	B = (A + 1) % 3;
	const double w = -dot(n,t3d*f.getVertex(0));

	const double k1 = 1 / n[C];
	const double k2 = k1 * k1;
	const double k3 = k2 * k1;
	const double k4 = k3 * k1;

	res.Fa = k1 * p.Pa;
	res.Fb = k1 * p.Pb;
	res.Fc = -k2 * (n[A]*p.Pa + n[B]*p.Pb + w*p.P1);

	res.Faa = k1 * p.Paa;
	res.Fbb = k1 * p.Pbb;
	res.Fcc = k3 * (std::pow(n[A],2)*p.Paa + 2*n[A]*n[B]*p.Pab + std::pow(n[B],2)*p.Pbb
			+ w*(2*(n[A]*p.Pa + n[B]*p.Pb) + w*p.P1));

	if (!noInertia) {
		res.Faaa = k1 * p.Paaa;
		res.Fbbb = k1 * p.Pbbb;
		res.Fccc = -k4 * (std::pow(n[A],3)*p.Paaa + 3*std::pow(n[A],2)*n[B]*p.Paab
				+ 3*n[A]*std::pow(n[B],2)*p.Pabb + std::pow(n[B],3)*p.Pbbb
				+ 3*w*(std::pow(n[A],2)*p.Paa + 2*n[A]*n[B]*p.Pab + std::pow(n[B],2)*p.Pbb)
				+ w*w*(3*(n[A]*p.Pa + n[B]*p.Pb) + w*p.P1));

		res.Faab = k1 * p.Paab;
		res.Fbbc = -k2 * (n[A]*p.Pabb + n[B]*p.Pbbb + w*p.Pbb);
		res.Fcca = k3 * (std::pow(n[A],2)*p.Paaa + 2*n[A]*n[B]*p.Paab + std::pow(n[B],2)*p.Pabb
				+ w*(2*(n[A]*p.Paa + n[B]*p.Pab) + w*p.Pa));
	}

	return res;
}

void compVolumeIntegrals(const rw::common::Ptr<const TriMesh> p, const Transform3D<>& t3d, double& T0, Vector3D<>& T1, Vector3D<>& T2, Vector3D<>& TP, bool noInertia = false)
{
	std::size_t used = 0;
	for (std::size_t i = 0; i < p->size(); i++) {
		const Triangle<double>& face = p->getTriangle(i);
		const Vector3D<> n = t3d.R()*face.calcFaceNormal();
		const Vector3D<> nabs = Math::abs(n);
		std::size_t A, B, C;
		if (nabs[0] > nabs[1] && nabs[0] > nabs[2]) C = 0;
		else C = (nabs[1] > nabs[2]) ? 1 : 2;
		A = (C + 1) % 3;
		B = (A + 1) % 3;
		if (!(nabs[C] > 0)) {
			if (face.calcArea() == 0)
				continue;
			else
				RW_THROW("Could not determine normal for face with non-zero area.");
		} else {
			used++;
		}

		const FaceIntegrals f = compFaceIntegrals(face, t3d, noInertia);

		T0 += n[0] * ((A == 0) ? f.Fa : ((B == 0) ? f.Fb : f.Fc));

		T1[A] += n[A] * f.Faa;
		T1[B] += n[B] * f.Fbb;
		T1[C] += n[C] * f.Fcc;

		if (!noInertia) {
			T2[A] += n[A] * f.Faaa;
			T2[B] += n[B] * f.Fbbb;
			T2[C] += n[C] * f.Fccc;
			TP[A] += n[A] * f.Faab;
			TP[B] += n[B] * f.Fbbc;
			TP[C] += n[C] * f.Fcca;
		}
	}

	T1 /= 2;
	if (!noInertia) {
		T2 /= 3;
		TP /= 2;
	}
}

struct MassParameters {
	MassParameters(): cog(Vector3D<>::zero()), inertia(0,0,0), volume(0) {}
	Vector3D<> cog;
	InertiaMatrix<> inertia;
	double volume;
};

InertiaMatrix<> moveInertiaPoint(double mass, const InertiaMatrix<>& inertia, const Vector3D<>& cog) {
	InertiaMatrix<> J = inertia;
	J(0,0) -= mass * (cog[1]*cog[1] + cog[2]*cog[2]);
	J(1,1) -= mass * (cog[2]*cog[2] + cog[0]*cog[0]);
	J(2,2) -= mass * (cog[0]*cog[0] + cog[1]*cog[1]);
	J(0,1) = J(1,0) += mass * cog[0] * cog[1];
	J(1,2) = J(2,1) += mass * cog[1] * cog[2];
	J(2,0) = J(0,2) += mass * cog[2] * cog[0];
	return J;
}

MassParameters mirtichMassParameters(double mass, const Transform3D<>& t3d, const rw::common::Ptr<const TriMesh> trimesh, bool noInertia = false)
{
	MassParameters par;

	Vector3D<> T1 = Vector3D<>::zero();
	Vector3D<> T2 = Vector3D<>::zero();
	Vector3D<> TP = Vector3D<>::zero();

	// Do the computations
	compVolumeIntegrals(trimesh, t3d, par.volume, T1, T2, TP, noInertia);

	//RW_ASSERT(par.volume > 0);

	par.cog = T1/par.volume;

	if (!noInertia) {
		const double density = mass/par.volume;
		InertiaMatrix<>& J = par.inertia;
		J(0,0) = density * (T2[1] + T2[2]);
		J(1,1) = density * (T2[2] + T2[0]);
		J(2,2) = density * (T2[0] + T2[1]);
		J(0,1) = J(1,0) = - density * TP[0];
		J(1,2) = J(2,1) = - density * TP[1];
		J(2,0) = J(0,2) = - density * TP[2];
	}

    return par;
}
}

double GeometryUtil::estimateVolume(const std::vector<Geometry::Ptr> &geoms) {
    double totalVolume = 0;
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        const GeometryData::Ptr gdata = geom->getGeometryData();
        const TriMesh::Ptr trimesh = gdata->getTriMesh(false);
        const MassParameters par = mirtichMassParameters(1,Transform3D<>::identity(),trimesh,true);
        totalVolume += par.volume;
    }
    return totalVolume;
}

double GeometryUtil::estimateVolume(const TriMesh &trimesh) {
	const MassParameters par = mirtichMassParameters(1,Transform3D<>::identity(),&trimesh,true);
	return par.volume;
}

/*
 * Locates all frames that are staticly connected to the frame and
 * that has geometry information
 */
std::vector<Frame*> GeometryUtil::getAnchoredFrames(Frame &frame, const State &state){
    // first locate the independant parent everything else than fixed
    Frame *parent = &frame;

    // if the current frame is the independent parent then use that
    if( dynamic_cast<FixedFrame*>(parent)==NULL ){
        return getAnchoredChildFrames(parent,state);
    }

    // else search for the independent parent
    for(; parent->getParent(state)!=NULL; parent = parent->getParent(state)){
        if( dynamic_cast<FixedFrame*>(parent)==NULL ){
            return getAnchoredChildFrames(parent,state);
        }
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

            if( dynamic_cast<FixedFrame*>(parent) ){
                fstack.push(f);
                //if( hasGeo )
                res.push_back(f);
            }
        }
    }
    return res;
}

Vector3D<> GeometryUtil::estimateCOG(const std::vector<Geometry::Ptr> &geoms)
{
    if(geoms.size()==0)
        RW_THROW("At least one geometry is required!");
    double totalVolume = 0;
    Vector3D<> center(0.,0.,0.);
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        const GeometryData::Ptr gdata = geom->getGeometryData();
        const TriMesh::Ptr trimesh = gdata->getTriMesh(false);
        const Transform3D<> t3d = geom->getTransform();
        const MassParameters par = mirtichMassParameters(1,t3d,trimesh,true);
        totalVolume += par.volume;
        center += par.cog*par.volume;
    }
    if(totalVolume==0.0)
        RW_THROW("There are no valid geometries!");
    return center/totalVolume;
}


Vector3D<> GeometryUtil::estimateCOG(const std::vector<Geometry::Ptr> &geoms, const Frame* ref, const State& state)
{
    if(geoms.size()==0)
        RW_THROW("At least one geometry is required!");
    double totalVolume = 0;
    Vector3D<> center(0.,0.,0.);
    BOOST_FOREACH(Geometry::Ptr geom, geoms){
        const GeometryData::Ptr gdata = geom->getGeometryData();
        const TriMesh::Ptr trimesh = gdata->getTriMesh(false);
        Transform3D<> t3d;
        if(geom->getFrame()!=NULL)
        	t3d = Kinematics::frameTframe(ref,geom->getFrame(), state);
        t3d = t3d*geom->getTransform();
        const MassParameters par = mirtichMassParameters(1,t3d,trimesh,true);
        totalVolume += par.volume;
        center += par.cog*par.volume;
    }
    if(totalVolume==0.0)
        RW_THROW("There are no valid geometries!");
    return center/totalVolume;
}

Vector3D<> GeometryUtil::estimateCOG(const TriMesh& trimesh, const Transform3D<>& t3d){
	const MassParameters par = mirtichMassParameters(1,t3d,&trimesh,true);
	return par.cog;
}

double GeometryUtil::calcMaxDist(const std::vector<Geometry::Ptr> &geoms,
                                 const rw::math::Vector3D<> center,
                                 rw::kinematics::Frame* ref, const rw::kinematics::State& state)
{
    double maxDist = 0;

    // first find center mass
	BOOST_FOREACH(Geometry::Ptr geom, geoms){
		GeometryData::Ptr gdata = geom->getGeometryData();
        // check if type of geom is really a trimesh
		TriMesh::Ptr trimesh = gdata->getTriMesh(false);

        Transform3D<> t3d;
        if(geom->getFrame()!=NULL)
            t3d = Kinematics::frameTframe(ref,geom->getFrame(), state);
        t3d = t3d*geom->getTransform();

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

std::pair<Vector3D<>, InertiaMatrix<> > GeometryUtil::estimateInertiaCOG(double mass,
	const std::vector<Geometry::Ptr>& geoms,
	const Frame* refframe, const State& state,
    const Transform3D<>& ref)
{
	if(geoms.size()==0)
		RW_THROW("At least one geometry is required!");
    const Vector3D<float> center = cast<float>( ref * estimateCOG( geoms, refframe, state ) );
    Transform3D<> nref = ref;
    nref.P() -= cast<double>(center);
    const InertiaMatrix<> inertia = estimateInertia(mass, geoms, refframe, state, nref);
    return std::make_pair(cast<double>(center),inertia);
}

InertiaMatrix<> GeometryUtil::estimateInertia(
    double mass,
	const std::vector<Geometry::Ptr>& geoms,
	const Frame* refframe, const State& state,
    const Transform3D<>& ref)
{
	if(geoms.size()==0)
		RW_THROW("At least one geometry is required!");

    double totalVolume = 0;
	std::vector<MassParameters> par(geoms.size());
	std::size_t i = 0;
    BOOST_FOREACH(const Geometry::Ptr geom, geoms){
    	const GeometryData::Ptr gdata = geom->getGeometryData();
    	const TriMesh::Ptr trimesh = gdata->getTriMesh(false);

        Transform3D<> t3d = ref;
        if(geom->getFrame()!=NULL)
            t3d = Kinematics::frameTframe(refframe,geom->getFrame(), state);
        t3d = t3d*geom->getTransform();

        par[i] = mirtichMassParameters(1, t3d, trimesh);
        totalVolume += par[i].volume;
        i++;
    }
    if(totalVolume==0.0)
        RW_THROW("There are no valid geometries!");
	InertiaMatrix<> inertia;
	for (std::size_t i = 0; i < geoms.size(); i++) {
    	const double m = mass*par[i].volume/totalVolume;
    	const InertiaMatrix<>& J = par[i].inertia;
    	for (std::size_t i = 0; i < 3; i++) {
    		for (std::size_t j = 0; j < 3; j++) {
    			inertia(i,j) += m*J(i,j);
    		}
    	}
    }
    return inertia;
}

InertiaMatrix<> GeometryUtil::estimateInertia(
    double mass,
    const std::vector<Geometry::Ptr>& geoms,
    const Transform3D<>& reftrans)
{
    if(geoms.size()==0)
        RW_THROW("At least one geometry is required!");

    double totalVolume = 0;
	std::vector<MassParameters> par(geoms.size());
	std::size_t i = 0;
    BOOST_FOREACH(const Geometry::Ptr geom, geoms){
    	const GeometryData::Ptr gdata = geom->getGeometryData();
    	const TriMesh::Ptr trimesh = gdata->getTriMesh(false);
    	const Transform3D<> t3d = reftrans*geom->getTransform();
    	par[i] = mirtichMassParameters(1, t3d, trimesh);
        totalVolume += par[i].volume;
        i++;
    }
    if(totalVolume==0.0)
        RW_THROW("There are no valid geometries!");
	InertiaMatrix<> inertia;
	for (std::size_t i = 0; i < geoms.size(); i++) {
    	const double m = mass*par[i].volume/totalVolume;
    	const InertiaMatrix<>& J = par[i].inertia;
    	for (std::size_t i = 0; i < 3; i++) {
    		for (std::size_t j = 0; j < 3; j++) {
    			inertia(i,j) += m*J(i,j);
    		}
    	}
    }
    return inertia;
}
