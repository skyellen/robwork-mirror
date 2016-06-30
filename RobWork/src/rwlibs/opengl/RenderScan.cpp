/*
 * RenderScan.cpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */
#include "RenderScan.hpp"
#include <rw/math/Math.hpp>
#include <rw/sensor/Scanner25DModel.hpp>
#include <rwlibs/os/rwgl.hpp>

using namespace rwlibs::opengl;
using namespace rw::math;
using namespace rw::graphics;

RenderScan::RenderScan(const rw::geometry::PointCloud& img):
        _minDepth(0),
        _maxDepth(10)
{
    setScan(img);
}


RenderScan::RenderScan():
        _minDepth(0),
        _maxDepth(10)
{}

RenderScan::RenderScan(const rw::sensor::Scanner25DModel::Ptr scanner)
{
    //_scanner = scanner;
    _minDepth = (float)scanner->getRange().first;
    _maxDepth = (float)scanner->getRange().second;

}


RenderScan::~RenderScan(){}

void RenderScan::setScan(const rw::geometry::PointCloud& img){
	_img = img;
}

void RenderScan::setScan(float dist){
	_img.resize(1,1);
	_img.getData()[0] = Vector3D<float>(0,0,dist);
}

void RenderScan::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
{

    /*const rw::sensor::Image25D *img = &_img;
    if(_scanner!=NULL){
        //std::cout << "using scanner " ;
        img = &_scanner->getScan();
        //std::cout << img->getHeight() << " " << img->getWidth() << " \n";
    }*/

	// ignores drawstate
	glPushMatrix();
	float dist = _maxDepth-_minDepth;
    for (size_t y = 0; y < static_cast<size_t>(_img.getHeight()); y++) {
    	// we only draw stuff that is within range

		glBegin(GL_LINE_STRIP);
        for (size_t x = 0; x < static_cast<size_t>(_img.getWidth()); x++) {
        	//Unused: const Vector3D<float> &v1 = _img.getData()[x+y*_img.getWidth()];
        	//if(fabs(v1[2])>_maxDepth || fabs(v1[2])<_minDepth)
        	//	continue;

          //  for(size_t j=x; j<_img.getWidth(); j++){
        	const Vector3D<float> &v = _img.getData()[x+y*_img.getWidth()];
//        	x = j;
        	//if(fabs(v[2])>_maxDepth || fabs(v[2])<_minDepth)
        	//	break;
        	float col = (float)Math::clamp( (fabs(v[2])-_minDepth)/dist, 0.0f, 1.0f);
			glColor3f(col, 0.0, 1.0f-col);
			glVertex3d(v(0), v(1), Math::clamp( (double)v(2) , -2, 0 ));    // Bottom Left
	//        }

        }
		glEnd();

    }
    glPopMatrix();
}
