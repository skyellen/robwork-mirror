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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

#include <boost/foreach.hpp>

#include "Triangulate.hpp"
#include "Plane.hpp"

using namespace rw::math;
using namespace rw::geometry;

static const float EPSILON = 0.0000000001f;

double Triangulate::calcArea(const std::vector<rw::math::Vector2D<> > &contour)
{
    int n = (int)contour.size();

    float A=0.0f;

    for(int p=n-1,q=0; q<n; p=q++)
    {
        A+= (float)(contour[p](0)*contour[q](1) - contour[q](0)*contour[p](1));
    }
    return A*0.5f;
}

/*
  InsideTriangle decides if a point P is Inside of the triangle
  defined by A, B, C.
*/
bool Triangulate::insideTriangle2D(
    float Ax, float Ay,
    float Bx, float By,
    float Cx, float Cy,
    float Px, float Py)

{
    float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
    float cCROSSap, bCROSScp, aCROSSbp;

    ax = Cx - Bx;  ay = Cy - By;
    bx = Ax - Cx;  by = Ay - Cy;
    cx = Bx - Ax;  cy = By - Ay;
    apx= Px - Ax;  apy= Py - Ay;
    bpx= Px - Bx;  bpy= Py - By;
    cpx= Px - Cx;  cpy= Py - Cy;

    aCROSSbp = ax*bpy - ay*bpx;
    cCROSSap = cx*apy - cy*apx;
    bCROSScp = bx*cpy - by*cpx;

    return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
};

bool Triangulate::snip(const std::vector<rw::math::Vector2D<> >& contour,int u,int v,int w,int n,int *V)
{
    int p;
    float Ax, Ay, Bx, By, Cx, Cy, Px, Py;

    Ax = (float)contour[V[u]](0);
    Ay = (float)contour[V[u]](1);

    Bx = (float)contour[V[v]](0);
    By = (float)contour[V[v]](1);

    Cx = (float)contour[V[w]](0);
    Cy = (float)contour[V[w]](1);

    if ( EPSILON > (((Bx-Ax)*(Cy-Ay)) - ((By-Ay)*(Cx-Ax))) ) return false;

    for (p=0;p<n;p++)
    {
        if( (p == u) || (p == v) || (p == w) ) continue;
        Px = (float)contour[V[p]](0);
        Py = (float)contour[V[p]](1);
        if (insideTriangle2D(Ax,Ay,Bx,By,Cx,Cy,Px,Py)) return false;
    }

    return true;
}


bool Triangulate::processPoints(const std::vector< Vector3D<> >& contour, std::vector<int>& result, double colinearCriteria) {
	//Compute plane of the contour 
	if (contour.size() < 3) {
		return false;
	}

	Vector3D<> v1 = normalize(contour[1]-contour[0]);
	Vector3D<> v2;
	size_t i = 2;
	do {
		v2 = normalize(contour[i]-contour[0]);
		i++;
	} while (fabs(dot(v1, v2) - 1) < colinearCriteria && i<contour.size());
	if (i == contour.size()) {
		RW_THROW("The points on the contour appears to be colinear");
	}
	
	Vector3D<> normal = Math::abs(cross(v1, v2));
	int maxIdx = 0;
	if (normal[0] > normal[1] && normal[0] > normal[2]) {
		maxIdx = 0;	
	}
	else if (normal[1] > normal[0] && normal[1] > normal[2]) {	
		maxIdx = 1;
	}
	else {
		maxIdx = 2;
	}
	
	std::vector<Vector2D<> > contour2D;
	BOOST_FOREACH(const Vector3D<>& p3d, contour) {
		contour2D.push_back(Vector2D<>(p3d[(maxIdx+1) % 3], p3d[(maxIdx+2) % 3]));
	}

	return processPoints(contour2D, result);

}

bool Triangulate::processPoints(const std::vector< rw::math::Vector2D<> >& contour, std::vector<int>& result)
//bool Triangulate::Process(const Vector2DVector &contour,Vector2DVector &result)
{
    /* allocate and initialize list of Vertices in polygon */

    int n = (int)contour.size();

    if ( n < 3 ){
    	//rw::common::Log::debugLog() << "Triangulate::processPoints has recieved polygon with less than 3 vertices" << std::endl;
    	return false;
    }

    int *V = new int[n];

    /* we want a counter-clockwise polygon in V */

    if ( 0.0f < calcArea(contour) ) {
        for (int v=0; v<n; v++) {
			V[v] = v;
		}
	}
    else {
        for(int v=0; v<n; v++) {
			V[v] = (n-1)-v;
		}
	}

    int nv = n;

    /*  remove nv-2 Vertices, creating 1 triangle every time */
    int count = 2*nv;   /* error detection */

    for(int m=0, v=nv-1; nv>2; )
    {
        /* if we loop, it is probably a non-simple polygon */
        if (0 >= (count--))
        {
            //** Triangulate: ERROR - probable bad polygon!
        	delete[] V;
        	rw::common::Log::debugLog() << "Triangulate::processPoints appear to have received a non-simple polygon" << std::endl;
            return false;
        }

        /* three consecutive vertices in current polygon, <u,v,w> */
        int u = v; 
		if (nv <= u) {
			u = 0;     /* previous */
		}
        v = u+1; 
		if (nv <= v) {
			v = 0;     /* new v    */
		}
        int w = v+1; 
		if (nv <= w)  {
			w = 0;     /* next     */
		}

        if ( snip(contour,u,v,w,nv,V) )
        {
            int a,b,c,s,t;

            /* true names of the vertices */
            a = V[u]; b = V[v]; c = V[w];

            /* output Triangle */
            result.push_back( a );
            result.push_back( b );
            result.push_back( c );

            m++;

            /* remove v from remaining polygon */
            for(s=v,t=v+1;t<nv;s++,t++) {
				V[s] = V[t]; 
			}
			nv--;

            /* resest error detection counter */
            count = 2*nv;
        }
    }

    delete[] V;

    return true;
}
