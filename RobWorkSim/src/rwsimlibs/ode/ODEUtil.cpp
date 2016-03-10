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

#include "ODEUtil.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsim::simulator;
using namespace rw::geometry;

void ODEUtil::toODERotation(const rw::math::Rotation3D<>& rwR, dMatrix3 R){
    R[0] = rwR(0,0);
    R[1] = rwR(0,1);
    R[2] = rwR(0,2);
    R[4] = rwR(1,0);
    R[5] = rwR(1,1);
    R[6] = rwR(1,2);
    R[8] = rwR(2,0);
    R[9] = rwR(2,1);
    R[10] = rwR(2,2);
}

void ODEUtil::setODEBodyT3D(dBodyID bodyId, const rw::math::Transform3D<>& t3d){
    //std::cout << bodyId << " " << t3d.P()[0] << " " << t3d.P()[1] << " " << t3d.P()[2] << std::endl;
    dBodySetPosition(bodyId, t3d.P()[0], t3d.P()[1], t3d.P()[2] );
    rw::math::Quaternion<> rwQuat( t3d.R() );
    dMatrix3 R;
    toODERotation(t3d.R(),R);
    dBodySetRotation(bodyId, R);
}

void ODEUtil::setODEGeomT3D(dGeomID geoId, const rw::math::Transform3D<>& t3d){
    dGeomSetPosition(geoId, t3d.P()[0], t3d.P()[1], t3d.P()[2] );
    dMatrix3 R;
    toODERotation(t3d.R(),R);
    dGeomSetRotation(geoId, R);
}

rw::math::Transform3D<> ODEUtil::getODEGeomT3D(dGeomID geomId){
	if (dGeomGetClass(geomId) == dPlaneClass) {
		dVector4 result;
		dGeomPlaneGetParams(geomId, result);
		const Vector3D<> n(result[0],result[1],result[2]);
		const double d = result[3];
		return Transform3D<>(d*n,EAA<>(Vector3D<>::z(),n).toRotation3D());
	} else {
		const dReal *v = dGeomGetPosition(geomId);
		dReal q[4];
		dGeomGetQuaternion(geomId, q);

		Vector3D<> pos(v[0],v[1],v[2]);
		Quaternion<> quat(q[1],q[2],q[3],q[0]);
		return Transform3D<>(pos,quat);
	}
}
rw::math::Transform3D<> ODEUtil::getODEBodyT3D(dBodyID bodyId){
    const dReal *v = dBodyGetPosition(bodyId);
    const dReal *q = dBodyGetQuaternion(bodyId);

    Vector3D<> pos(v[0],v[1],v[2]);
    Quaternion<> quat(q[1],q[2],q[3],q[0]);
    return Transform3D<>(pos,quat);
}

void ODEUtil::setODEBodyMass(dBodyID body, double mass, const Vector3D<>& c, const InertiaMatrix<>& I){
    dMass m;
    dReal i11 = I(0,0);
    dReal i22 = I(1,1);
    dReal i33 = I(2,2);
    dReal i12 = I(0,1);
    dReal i13 = I(0,2);
    dReal i23 = I(1,2);
    dMassSetParameters(&m, mass,
                       c(0), c(1), c(2),
                       i11, i22, i33,
                       i12, i13, i23);
    dMassCheck(&m);
    dBodySetMass(body, &m);
}


double ODEUtil::calcElasticERP(double kp, double kd, double dt){
    return (dt*kp)/(dt*kp+kd);
}

double ODEUtil::calcElasticCFM(double kp, double kd, double dt){
    return 1.0/(dt*kp+kd);
}

namespace {
    rw::common::Cache<GeometryData*, ODEUtil::TriMeshData > _cache;
}

ODEUtil::TriMeshData::Ptr ODEUtil::buildTriMesh(GeometryData::Ptr gdata, bool invert){
    // check if the geometry is allready in cache
    if( _cache.isInCache(gdata.get()) ){
        ODEUtil::TriMeshData::Ptr tridata = _cache.get(gdata.get());
        return tridata;
    }

    //bool ownedData = false;
    IndexedTriMesh<float>::Ptr imesh;

    // if not in cache then we need to create a TriMeshData geom,
    // but only if the geomdata is a trianglemesh
    if( !dynamic_cast<TriMesh*>(gdata.get()) ){
        TriMesh::Ptr mesh = gdata->getTriMesh();
        imesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >(*mesh,0.00001);
        //ownedData = true;
    } else if( !dynamic_cast< IndexedTriMesh<float>* >(gdata.get()) ){
        // convert the trimesh to an indexed trimesh
        imesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >(*((TriMesh*)gdata.get()),0.00001);
        //ownedData = true;
    } else {
        imesh = static_cast< IndexedTriMesh<float>* >(gdata.get());
    }
    int nrOfVerts = (int)imesh->getVertices().size();
    int nrOfTris = (int)imesh->getSize();
    // std::cout  << "- NR of faces: " << nrOfTris << std::endl;
    // std::cout  << "- NR of verts: " << nrOfVerts << std::endl;

    ODEUtil::TriMeshData::Ptr data =
        ownedPtr(new ODEUtil::TriMeshData(nrOfTris*3, nrOfVerts*3));
    dTriMeshDataID triMeshDataId = dGeomTriMeshDataCreate();

    //const float myScale = 1.02;

    data->triMeshID = triMeshDataId;
    int vertIdx = 0;
    BOOST_FOREACH(const Vector3D<float>& v, imesh->getVertices()){
        data->vertices[vertIdx+0] = v(0);
        data->vertices[vertIdx+1] = v(1);
        data->vertices[vertIdx+2] = v(2);
        vertIdx+=3;
    }
    int indiIdx = 0;
    //BOOST_FOREACH(, imesh->getTriangles()){
    for(size_t i=0;i<imesh->getSize();i++){
        const IndexedTriangle<uint32_t> tri = imesh->getIndexedTriangle(i);
        if(invert){
            data->indices[indiIdx+0] = tri.getVertexIdx(2);
            data->indices[indiIdx+1] = tri.getVertexIdx(1);
            data->indices[indiIdx+2] = tri.getVertexIdx(0);
        } else {
            data->indices[indiIdx+0] = tri.getVertexIdx(0);
            data->indices[indiIdx+1] = tri.getVertexIdx(1);
            data->indices[indiIdx+2] = tri.getVertexIdx(2);
        }
        //if(data->indices[indiIdx+0]>=(size_t)nrOfVerts)
        //  std::cout << indiIdx+0 << " " << data->indices[indiIdx+0] << "<" << nrOfVerts << std::endl;
        RW_ASSERT( data->indices[indiIdx+0]< (size_t)nrOfVerts );
        //if(data->indices[indiIdx+1]>=(size_t)nrOfVerts)
        //  std::cout << data->indices[indiIdx+1] << "<" << nrOfVerts << std::endl;
        RW_ASSERT( data->indices[indiIdx+1]<(size_t)nrOfVerts );
        //if(data->indices[indiIdx+2]>=(size_t)nrOfVerts)
        //  std::cout << data->indices[indiIdx+2] << "<" << nrOfVerts << std::endl;
        RW_ASSERT( data->indices[indiIdx+2]<(size_t)nrOfVerts );

        indiIdx+=3;
    }

    dGeomTriMeshDataBuildSingle(triMeshDataId,
            &data->vertices[0], 3*sizeof(float), nrOfVerts,
            (dTriIndex*)&data->indices[0], nrOfTris*3, 3*sizeof(dTriIndex));

    // write all data to the disc
    /*
    std::ofstream fstr;
    std::stringstream sstr;
    sstr << "test_data_" << nrOfVerts << ".h";
    fstr.open(sstr.str().c_str());
    if(!fstr.is_open())
        RW_THROW("fstr not open!");
    fstr << "const int VertexCount = " << nrOfVerts << "\n"
         << "const int IndexCount = " << nrOfTris << " * 3\n"
         << "\n\n"
         << "float Vertices[VertexCount * 3] = {";
    for(int i=0;i<nrOfVerts-1;i++){
        fstr << data->vertices[i*3+0] << ","
             << data->vertices[i*3+1] << ","
             << data->vertices[i*3+2] << ",\n";
    }
    fstr << data->vertices[(nrOfVerts-1)*3+0] << ","
         << data->vertices[(nrOfVerts-1)*3+1] << ","
         << data->vertices[(nrOfVerts-1)*3+2] << "\n };";

    fstr << "\n\ndTriIndex Indices[IndexCount/3][3] = { \n";
    for(int i=0;i<nrOfTris-1;i++){
        fstr << "{" << data->indices[i*3+0] << ","
             << data->indices[i*3+1] << ","
             << data->indices[i*3+2] << "},\n";
    }
    fstr << "{" << data->indices[(nrOfTris-1)*3+0] << ","
         << data->indices[(nrOfTris-1)*3+1] << ","
         << data->indices[(nrOfTris-1)*3+2] << "}\n };";

    fstr.close();
*/
    //triMeshDatas.push_back(boost::shared_ptr<ODESimulator::TriMeshData>(data) );

//      if( ownedData )
//          delete imesh;

    return data;
}

std::vector<ODEUtil::TriGeomData*> ODEUtil::buildTriGeom(std::vector<Geometry::Ptr> geoms, dSpaceID spaceid, Frame* ref, const State& state, bool invert){
    std::vector<ODEUtil::TriGeomData*> triGeomDatas;
    //std::cout << "buildTriGeom: " << geoms.size() << std::endl;
    for(size_t i=0; i<geoms.size(); i++){
        GeometryData::Ptr rwgdata = geoms[i]->getGeometryData();
        Transform3D<> transform;
        if(geoms[i]->getFrame()!=NULL)
            transform = Kinematics::frameTframe(ref,geoms[i]->getFrame(),state );
        transform =transform* geoms[i]->getTransform();

        ODEUtil::TriMeshData::Ptr triMeshData = buildTriMesh(rwgdata,invert);
        if(triMeshData==NULL){
            //std::cout << "TriMeshNull" << std::endl;
            continue;
        }

        dGeomID geoId;
        bool isTriMesh=false;
        bool placeable=true;
        if( Sphere* sphere_rw = dynamic_cast<Sphere*>(rwgdata.get()) ){
            geoId = dCreateSphere(spaceid, (dReal)sphere_rw->getRadius());
        } else if( Cylinder* cyl_rw = dynamic_cast<Cylinder*>(rwgdata.get()) ){
            geoId = dCreateCylinder(spaceid, (dReal)cyl_rw->getRadius(), (dReal)cyl_rw->getHeight());
        } else if( Plane* plane_rw = dynamic_cast<Plane*>(rwgdata.get()) ){
            Vector3D<> n = plane_rw->normal();
            geoId = dCreatePlane(spaceid, (dReal)n[0], (dReal)n[1], (dReal)n[2], (dReal)plane_rw->d());
            placeable = false;
        } else {
            geoId = dCreateTriMesh(spaceid, triMeshData->triMeshID, NULL, NULL, NULL);
            isTriMesh = true;
        }

        dGeomSetData(geoId, triMeshData->triMeshID);
        ODEUtil::TriGeomData *gdata = new ODEUtil::TriGeomData(triMeshData);
        gdata->refframe = geoms[i]->getFrame();
        gdata->isGeomTriMesh = isTriMesh;
        ODEUtil::toODETransform(transform, gdata->p, gdata->rot);
        gdata->t3d = transform;
        triGeomDatas.push_back(gdata);
        gdata->geomId = geoId;
        gdata->isPlaceable = placeable;
    }
    // create geo
    return triGeomDatas;
}





