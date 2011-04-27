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

#include "ODEDebugRender.hpp"

#include "ODESimulator.hpp"

#include <rwlibs/os/rwgl.hpp>
#include <ode/ode.h>
#include <boost/foreach.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rw/math.hpp>
#include <rw/kinematics.hpp>

using namespace rwsim::drawable;
using namespace rwlibs::opengl;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rw::math;
using namespace rw::kinematics;

namespace {

    void odeToGLTransform(
        const dReal* pos,
        const dReal* rot,
        GLfloat* gltrans)
    {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++)
                gltrans[j + 4 * k] =
                    (float)rot[4 *j +  k];

            gltrans[12 + j] =
                (float)pos[j];
        }

        gltrans[3] = gltrans[7] = gltrans[11] = 0;
        gltrans[15] = 1;
    }
}

void ODEDebugRender::draw(const rw::graphics::DrawableNode::RenderInfo& info, DrawType draw, double alpha) const
{

    if (DRAW_COLLISION_GEOMETRY & _drawMask) {
        const std::vector<ODESimulator::TriGeomData*>& trimeshs = _sim->getTriMeshs();
        //std::vector<dContact> contacts = _sim->getContacts();
        BOOST_FOREACH(ODESimulator::TriGeomData* trigeom, trimeshs){
            ODESimulator::TriMeshData::Ptr trimesh = trigeom->tridata;
            // multiply stack transform with geom transform
            const dReal* pos = dGeomGetPosition(trigeom->geomId);
            const dReal* rot = dGeomGetRotation(trigeom->geomId);

            float gltrans[16];
            odeToGLTransform(pos, rot, gltrans);

            glPushMatrix();
            glMultMatrixf(gltrans);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glBegin(GL_TRIANGLES);

            for (size_t i = 0; i < trimesh->indices.size() / 3; i++) {
                const float *p;
                p = &trimesh->vertices[trimesh->indices[i * 3 + 0] * 3];
                glVertex3f((float)p[0],(float)p[1],(float)p[2]);

                p = &trimesh->vertices[trimesh->indices[i * 3 + 1] * 3];
                glVertex3f((float)p[0],(float)p[1],(float)p[2]);

                p = &trimesh->vertices[trimesh->indices[i * 3 + 2] * 3];
                glVertex3f((float)p[0],(float)p[1],(float)p[2]);
            }

            // draw all contacts
            glEnd();
            glPopMatrix();
        }
    }
    //getContactManifoldMap

    if(DRAW_CONTACT_NORMAL & _drawMask){
        glPushMatrix();
        //glPointSize(20.0);
        //glBegin(GL_POINTS);
        std::vector<ContactPoint> contacts = _sim->getContacts();
        for (size_t i = 0; i < contacts.size(); i++) {
            ContactPoint &con = contacts[i];
            // draw the contact normal
            //DrawableUtil::drawGLVertex(con.p);
            //if(DRAW_CONTACT_NORMAL & _drawMask){
            if(DRAW_FRICTION_CONE & _drawMask){
                // TODO: we need to determine the mu of the friction cone
            }

            glLineWidth(0.1);
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            DrawableUtil::drawGLVertex(con.p);
            glColor3f(0.0, 1.0, 0.0);
            DrawableUtil::drawGLVertex(con.p + con.n);
            glEnd();

            //}
        }
        //glEnd( );
        glPopMatrix();
    }


    //if(DRAW_CONTACT_NORMAL & _drawMask){
    /*   glPushMatrix();
     //const std::vector<ContactPoint>& contacts = _sim->getContacts();
     for(int i=0; i<contacts.size(); i++){
     ContactPoint con = contacts[i];
     // draw the contact normal
     glLineWidth(0.1);
     glBegin(GL_LINES);
     glColor3f(1.0, 0.0, 0.0);
     DrawableUtil::drawGLVertex(con.p);
     DrawableUtil::drawGLVertex(con.p+con.n);
     glEnd();
     }
     glPopMatrix();
     //}
     * */

    if( (DRAW_BODY_FORCES & _drawMask) && info._state != NULL ){
        Vector3D<> gravity = _sim->getGravity();
        std::vector<ODEBody*> bodies = _sim->getODEBodies();
        BOOST_FOREACH(ODEBody *body, bodies){
            Body *rwbody = body->getRwBody();
            if(rwbody==NULL)
                continue;
            Vector3D<> grav = gravity*rwbody->getInfo().mass; // gravity is handled inside ODE
            Vector3D<> force = (body->getLastForce()+grav)/20;
            //std::cout << "FORCE:" << force << std::endl;
            Transform3D<> pos = Kinematics::worldTframe(rwbody->getBodyFrame(), *info._state);

            glLineWidth(2.5);
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            DrawableUtil::drawGLVertex(pos.P());
            glColor3f(0.0, 0.0, 0.0);
            DrawableUtil::drawGLVertex(pos.P() + force);
            glEnd();

        }
    }
}
