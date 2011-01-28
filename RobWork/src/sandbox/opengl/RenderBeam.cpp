/*
 * RenderBeam.cpp
 *
 *  Created on: 16/12/2010
 *      Author: jimali
 */

#include "RenderBeam.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rwlibs::opengl;
using namespace rw::trajectory;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::kinematics;

RenderBeam::RenderBeam(rw::models::BeamJoint* joint):
        _joints( std::vector<rw::models::BeamJoint*>(1,joint) )
{

}

RenderBeam::RenderBeam(std::vector<rw::models::BeamJoint*> joints):_joints(joints){

}

RenderBeam::~RenderBeam(){

}

void RenderBeam::update(const rw::kinematics::State& state){
    // transform each vertice in the triangle mesh according to the current state

    for(size_t j=0; j<_joints.size(); j++){
        Interpolator<Transform3D<> >::Ptr inter = _joints[j]->getInterpolator(state);
        Transform3D<> bTf = Transform3D<>::identity();
        if(j>0)
            bTf = Kinematics::frameTframe(_joints[0]->getParent(state),_joints[j]->getParent(state), state);
        Transform3D<> bTjfixed = bTf;//*_joints[j]->getFixedTransform() ;
        for(int i=0;i<6;i++){
            _transforms[j*6+i].transform = bTjfixed * inter->x(i/5.0);
            _transforms[j*6+i].s = i/5.0;
        }
    }

    for(size_t i=0;i<_verticetransform.size();i++){
        // the vertice at i should be transformed by transforms tidx and tidx+1

        int tidx = _verticetransform[i];
        if( tidx == -1 ){
            _mesh->getVertex(i) = _origvertices[i];
        } else {
            Transform3D<> t3d = _transforms[tidx].transform * inverse(_origtransforms[tidx].transform);
            Vector3D<> v = t3d *_origvertices[i]*0.5;
            if(tidx>0){
                v += _transforms[tidx-1].transform * inverse(_origtransforms[tidx-1].transform)*_origvertices[i]*0.5;
            } else {
                v += v;
            }
            _mesh->getVertex(i) = v;
        }
    }
}

void RenderBeam::init(rw::geometry::IndexedTriMesh<>::Ptr mesh, rw::kinematics::State& state){
    std::cout << "RenderBeam::init" << std::endl;
    _mesh = mesh;
    // the vertices of the mesh should be split into groups, which
    // each are controlled by one transformation
    _transforms.resize( _joints.size()*(5+1) );

    // the geometry is described relative to joint0 parent frame

    for(size_t j=0; j<_joints.size(); j++){
        Transform3D<> bTf = Transform3D<>::identity();
        if(j>0)
            bTf = Kinematics::frameTframe(_joints[0]->getParent(state),_joints[j]->getParent(state), state);

        Transform3D<> bTjfixed =  bTf; //*_joints[j]->getFixedTransform() ;
        std::cout << "bTf: " << j << "  " << bTf << std::endl;
        Interpolator<Transform3D<> >::Ptr inter = _joints[j]->getInterpolator(state);
        std::cout << "bTf: " << j << "  " << (bTf*inter->x(0)) << std::endl;
        for(int i=0;i<6;i++){
            _transforms[j*6+i].transform = bTjfixed * inter->x(i/5.0); // parent to joint
            _transforms[j*6+i].s = i/5.0;
            //std::cout << _transforms[i].transform << std::endl;
        }
    }

    _origvertices = _mesh->getVertices();
    //std::cout << "_origvertices: " << _origvertices.size() << std::endl;
    _verticetransform.resize(_origvertices.size());

    // now for each vertice in the mesh, find the 2 segments that it is closest to
    for(size_t i=0;i<_origvertices.size();i++){
        Vector3D<> &v = _origvertices[i];
        double minDist = MetricUtil::dist2(v,_transforms[0].transform.P());
        _verticetransform[i] = 0;
        for(size_t j=1;j<_transforms.size();j++){
            double dist = MetricUtil::dist2(v,_transforms[j].transform.P());
            if( dist < minDist ){
                minDist = dist;
                _verticetransform[i] = j;
            }
        }
        //std::cout << i << ": " << _verticetransform[i] << std::endl;
    }

    _origtransforms = _transforms;
}


void RenderBeam::draw(DrawType type, double alpha) const {
    if(_mesh==NULL)
        return;
    //std::cout << "RenderBeam::draw: " << _mesh->size() << std::endl;
    // Enable texture coordiantes, normals, and vertices arrays
    //if (obj.hasTexture())
    //  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    //if (lit)
    //glEnableClientState(GL_NORMAL_ARRAY);
    /*
    glPushMatrix();
    glEnableClientState(GL_VERTEX_ARRAY);

    // Point them to the objects arrays
    //if (obj.hasTexture())
    //  glTexCoordPointer(2, GL_FLOAT, 0, &(obj._texCoords.at(0)[0]));
    //if (lit)
    //glNormalPointer(GL_DOUBLE, sizeof(Vector3D<float>), &(obj._normals.at(0)[0]));
    glVertexPointer(3, GL_DOUBLE, sizeof(Vector3D<>), &(_mesh->getVertices()[0][0]));

    // Loop through the faces as sorted by material and draw them
    void *indices = _mesh->getIndices();

    // Draw the faces using an index to the vertex array
    //std::cout << "faces._subFaces.size()" << faces._subFaces.size() << std::endl;
    int nrIndices = _mesh->size()*3;
    glDrawElements(
        GL_TRIANGLES, nrIndices, GL_UNSIGNED_SHORT, indices);

    glPopMatrix();
    */
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPushMatrix();
    glBegin(GL_TRIANGLES);
    // Draw all faces.

    for(size_t i=0;i<_mesh->getSize();i++){
        Triangle<double> tri = _mesh->getTriangle(i);
        Vector3D<float> n = cast<float>(tri.calcFaceNormal());
        Vector3D<float> v0 = cast<float>(tri[0]);
        Vector3D<float> v1 = cast<float>(tri[1]);
        Vector3D<float> v2 = cast<float>(tri[2]);
        glNormal3fv(&n[0]);
        glVertex3fv(&v0[0]);
        glVertex3fv(&v1[0]);
        glVertex3fv(&v2[0]);
    }

    glEnd();
    glPopMatrix();

    //std::cout << "RenderBeam::drawdone: " << _mesh->size() << std::endl;

}
