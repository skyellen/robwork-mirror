/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "RenderIVG.hpp"

#include "IVGReader.hpp"
#include <rw/math/Vector3D.hpp>
#include <rw/common/macros.hpp>

#include <boost/bind.hpp>

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::geometry;



RenderIVG::RenderIVG(const std::string &filename, float r, float g, float b)
    : _r(r), _g(g), _b(b)
{
    readIVG(filename);

    _displayListId = glGenLists(1);
    glNewList(_displayListId, GL_COMPILE);
    glPushMatrix();
    glBegin(GL_TRIANGLES);

    // Draw all faces.
    std::for_each(_vfaces.begin(), _vfaces.end(), boost::bind(&RenderIVG::drawFace, this, _1));

    glEnd();
    glPopMatrix();
    glEndList();
}

void RenderIVG::setFaces(const std::vector<Face<float> >& faces) {
//    _vfaces = faces;
}

void RenderIVG::setColor(float r, float g, float b)
{
    _r = r;
    _g = g;
    _b = b;
}


void RenderIVG::drawFace(const ColorFace& cface)
{
    glNormal3fv(cface._normal1);
    glVertex3fv(cface._vertex1);
    glNormal3fv(cface._normal2);
    glVertex3fv(cface._vertex2);
    glNormal3fv(cface._normal3);
    glVertex3fv(cface._vertex3);
    //glColor4f(cface._r, cface._g, cface._b, cface._alpha);
}


void RenderIVG::readIVG(const std::string &filename)
{
	FILE *fp;
	if(! (fp = fopen(filename.c_str(), "rb")) )
	{
		RW_THROW("Couldn't open '" << filename << "'");
	}
	long lnStartPos = ftell(fp);
	fseek(fp, 0, SEEK_END);
	long lnEndPos = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	const long lnArraySize = lnEndPos - lnStartPos;
	char *pc_Buf = new char[lnArraySize];

	if (fread(pc_Buf, lnArraySize, 1, fp) == 0) {
	    fclose(fp);
	    RW_THROW("Failed to read bytes from file "<<filename);
	}
	fclose(fp);

	IVGReader reader;
	std::list<CIvgEntity> scene;

	/*int n_Ent = */reader.ParseIVGGeometry(pc_Buf, scene);
	delete pc_Buf;

	std::list<CIvgEntity>::iterator it;
	for(it=scene.begin(); it!=scene.end(); it++)
	{
		CIVGNode *ivgNode = it->pFirst;
		while(ivgNode)
		{
			if(ivgNode->nType == IVG_TYPE_TESS)
			{
				CIVGTess *ivgTess = static_cast<CIVGTess*>(ivgNode);
				for(int triIndx=0; triIndx<ivgTess->nTriangles; triIndx++)
				{
					/* 23 - blue shin material */
					//ColorFace face(0.2f, 0.2f, 0.3f, _alpha);	// ambient
					//ColorFace face(.0, .0, .47, _alpha);	// diffuse
					ColorFace face(0.4f, 0.4f, 0.5f, 1.0f);		// specular
					//ColorFace face(_r, _g, _b, _alpha);

					face._vertex1[0] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[0]].x();
					face._vertex1[1] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[0]].y();
					face._vertex1[2] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[0]].z();

					face._vertex2[0] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[1]].x();
					face._vertex2[1] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[1]].y();
					face._vertex2[2] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[1]].z();

					face._vertex3[0] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[2]].x();
					face._vertex3[1] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[2]].y();
					face._vertex3[2] = ivgTess->pVertices[ivgTess->pTriangles[triIndx].vInx[2]].z();

					face._normal1[0] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[0]].x();
					face._normal1[1] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[0]].y();
					face._normal1[2] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[0]].z();

					face._normal2[0] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[1]].x();
					face._normal2[1] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[1]].y();
					face._normal2[2] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[1]].z();

					face._normal3[0] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[2]].x();
					face._normal3[1] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[2]].y();
					face._normal3[2] = ivgTess->pNormals[ivgTess->pTriangles[triIndx].nInx[2]].z();

					_vfaces.push_back(face);
				}
			}
			ivgNode = ivgNode->pNext;
		}
	}
}
