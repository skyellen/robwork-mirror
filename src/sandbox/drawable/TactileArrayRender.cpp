#include "TactileArrayRender.hpp"

#include <rwlibs/drawable/DrawableUtil.hpp>

using namespace rw::math;
using namespace rw::sensor;
using namespace rwlibs::drawable;
using namespace boost::numeric;

void TactileArrayRender::draw(Render::DrawType type, double alpha) const{
    //if( _force.norm2()<0.001 )
    //    return;
    if(_sensor==NULL)
        return;

    ublas::matrix<float> values = _sensor->getTexelData();
    const TactileArray::VertexMatrix& verts = _sensor->getVertexGrid();
    Transform3D<> fTverts = _sensor->getTransform();
    float maxForce = _sensor->getPressureLimit().second;
    //draw all texels
    glPushMatrix();
    float gltrans[16];
    DrawableUtil::transform3DToGLTransform(fTverts, gltrans);
    glMultMatrixf(gltrans);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for(int x=0; x<(int)values.size1(); x++){
        for(int y=0; y<(int)values.size2(); y++){
            float col = (float)(values(x,y)/maxForce);
            glBegin(GL_QUADS);
            glColor3f(col, 0.0, 1-col);
            Vector3D<> v = verts[x][y];
            glVertex3f(v(0), v(1), v(2));    // Bottom Left
            v = verts[x][y+1];
            glVertex3f(v(0), v(1), v(2));    // Bottom Left
            v = verts[x+1][y+1];
            glVertex3f(v(0), v(1), v(2));    // Bottom Left
            v = verts[x+1][y];
            glVertex3f(v(0), v(1), v(2));    // Bottom Left
            glEnd();
        }
    }

    // now draw the normals

    const TactileArray::VertexMatrix& normals = _sensor->getNormals();
    const TactileArray::VertexMatrix& centers = _sensor->getCenters();
    glBegin(GL_LINES);
    for(int x=0; x<(int)values.size1(); x++){
        for(int y=0; y<(int)values.size2(); y++){
            float col = (float)(values(x,y)/maxForce);
            glColor3f(col, 0.0, 1-col);
            DrawableUtil::drawGLVertex(centers[x][y]);
            DrawableUtil::drawGLVertex(centers[x][y]+normals[x][y]*0.01);
        }
    }
    glEnd();
    glPopMatrix();
}

