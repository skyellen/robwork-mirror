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


#include "ProximityStrategyYaobi.hpp"

#include <cfloat>
#include <vector>

#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>

#include <boost/foreach.hpp>

#include <rw/proximity/ProximityStrategyData.hpp>

#include <yaobi/yaobi_mesh_interface.h>
#include <yaobi/yaobi_tree_builder.h>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// Utilities

namespace
{
	Ptr<yaobi::CollModel> makeModelFromSoup(TriMesh::Ptr mesh, const double scale)
    {
        unsigned char tri_stride(3);
        unsigned num_tris((unsigned)mesh->getSize());
        unsigned num_verts(num_tris*3);
        yaobi::AppRealT *vertices = new yaobi::AppRealT[num_verts*3];
        int *tris = new int[num_tris*3];

        for(unsigned triIdx = 0, vertIdx = 0;
            triIdx < num_tris;
            triIdx++, vertIdx += 3)
        {
            const Triangle<double> face = mesh->getTriangle(triIdx);

        	Vector3D<yaobi::Real> v0 = cast<yaobi::Real>(face[0]*scale);
        	Vector3D<yaobi::Real> v1 = cast<yaobi::Real>(face[1]*scale);
        	Vector3D<yaobi::Real> v2 = cast<yaobi::Real>(face[2]*scale);

            for (size_t j=0; j < 3; j++) {
                vertices[vertIdx*3+0+j] = v0[j];
                vertices[vertIdx*3+3+j] = v1[j];
                vertices[vertIdx*3+6+j] = v2[j];
            }
            tris[vertIdx+0] = vertIdx+0;
            tris[vertIdx+1] = vertIdx+1;
            tris[vertIdx+2] = vertIdx+2;
        }

        yaobi::TriMeshInterface* tri = new yaobi::TriMeshInterface(
            num_verts, vertices,
            num_tris, tris,
            tri_stride,
            yaobi::OWN_DATA);

        Ptr<yaobi::CollModel> model = ownedPtr(new yaobi::CollModel(tri, yaobi::OWN_DATA));
											   yaobi::build_obb_tree( *model, yaobi::OWN_DATA );

        //model->ShrinkToFit();
        return model;
    }

    // Convert Transform3D to Yaobi representation.
    void toTransform(
        const Transform3D<>& tr,
        yaobi::Real T[3][4])
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = static_cast<yaobi::Real>(tr(i, j));

    }

    void collide(
        const yaobi::CollModel& ma, const Transform3D<>& wTa,
        const yaobi::CollModel& mb, const Transform3D<>& wTb,
        yaobi::CollideResult& result, yaobi::QueryType type)
    {
        yaobi::Real ta[3][4];
        yaobi::Real tb[3][4];

        toTransform(wTa, ta);
        toTransform(wTb, tb);

        Collide(result, ta, ma, tb, mb, type);
    }
}

//----------------------------------------------------------------------
// ProximityStrategyYaobi

ProximityStrategyYaobi::ProximityStrategyYaobi()
{}


rw::proximity::ProximityModel::Ptr ProximityStrategyYaobi::createModel()
{
    YaobiProximityModel *model = new YaobiProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyYaobi::destroyModel(rw::proximity::ProximityModel* model){
    RW_ASSERT(model!=NULL);

    //model->models.clear();
    //YaobiProximityModel *pmodel = (YaobiProximityModel*) model.get();
}

bool ProximityStrategyYaobi::addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom){
    RW_ASSERT(model!=NULL);
    YaobiProximityModel *pmodel = (YaobiProximityModel*) model;
    YaobiModelPtr yaobimodel;
	GeometryData::Ptr gdata = geom.getGeometryData();
    // first check if model is in cache
    if( _modelCache.has(geom.getId()) ){
        yaobimodel = _modelCache.get(geom.getId());
    } else {
		TriMesh::Ptr mesh = gdata->getTriMesh(false);
        if(mesh->getSize()==0)
            return false;

        yaobimodel = makeModelFromSoup( mesh, geom.getScale());
    }
    pmodel->models.push_back( RWYaobiModel(geom.getTransform(), yaobimodel) );

    _allmodels.push_back(pmodel->models.back());
    _geoIdToModelIdx[geom.getId()].push_back((int)_allmodels.size()-1);
    return true;
}

bool ProximityStrategyYaobi::addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy){
    // we allways copy the data here
    return addGeometry(model,*geom);
}

bool ProximityStrategyYaobi::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
	return false;
}

bool ProximityStrategyYaobi::doInCollision(ProximityModel::Ptr aModel,
    const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
    const Transform3D<>& wTb,
    ProximityStrategyData& data)
{
	bool firstContact = data.getCollisionQueryType() == CollisionStrategy::FirstContact;
	bool isColliding = false;
    YaobiProximityModel *a = (YaobiProximityModel*)aModel.get();
    YaobiProximityModel *b = (YaobiProximityModel*)bModel.get();

    yaobi::QueryType qtype = yaobi::FIRST_CONTACT_ONLY;
    if(!firstContact)
    	qtype = yaobi::ALL_CONTACTS;
    yaobi::CollideResult result;
    BOOST_FOREACH(const RWYaobiModel& ma, a->models) {
        BOOST_FOREACH(const RWYaobiModel& mb, b->models) {
            //! Search for all contacting triangles
            collide(
                *ma.second, wTa * ma.first,
                *mb.second, wTb * mb.first,
                result, qtype);

            // TODO: copy all colliding triangles into data
            if (firstContact && result.IsColliding())
            	return true;
			if (result.IsColliding())
				isColliding = true;
        }
    }

    return isColliding;
}

void ProximityStrategyYaobi::clear()
{
	// TODO: also clear cache
    //_frameModelMap.clear();
    _modelCache.clear();
    _geoIdToModelIdx.clear();
    _allmodels.clear();

    clearFrames();
}

void ProximityStrategyYaobi::getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
									  rw::proximity::ProximityStrategyData& data)
{
	RW_THROW("NOT IM PLEMENTED IN YAOBI COLLISION STRATEGY!");
}



std::vector<std::string> ProximityStrategyYaobi::getGeometryIDs(rw::proximity::ProximityModel* model){
	RW_THROW("ProximityStrategyYaobi::getGeometryIDs(rw::proximity::ProximityModel* model): Not Implemented");
}

CollisionStrategy::Ptr ProximityStrategyYaobi::make()
{
    return ownedPtr(new ProximityStrategyYaobi);
}
