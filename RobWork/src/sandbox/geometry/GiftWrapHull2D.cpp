/*
 * GiftWrapHull3D.cpp
 *
 *  Created on: 30-03-2009
 *      Author: jimali
 */
#include "GiftWrapHull3D.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>

using namespace rw::geometry;
using namespace rw::math;


typedef std::stack<std::pair<GiftWrapHull3D::EdgeIdx,int> > EdgeStackType;

namespace {
	void addEdge(const GiftWrapHull3D::EdgeIdx& edge,
			     EdgeStackType& edgeStack,
				 std::set<GiftWrapHull3D::EdgeIdx>& edgeSet,
				 int triIdx){
		GiftWrapHull3D::EdgeIdx setEdge = edge;
		if(edge.first>edge.second)
			std::swap(setEdge.first,setEdge.second);

		if(edgeSet.find(setEdge) == edgeSet.end()){
			edgeSet.insert(setEdge);
			edgeStack.push(make_pair(edge,triIdx) );
		}
	}

	Vector3D<> calcNormal(const Vector3D<>& v1, const Vector3D<>& v2, const Vector3D<>& v3){
		//std::cout << "cross( " <<  v2-v1 << "," << v3-v1 << ")" << std::endl;
		return normalize( cross( v2-v1, v3-v1) );
	}

	void addMinVal(double val, Vector3D<>& vals, int pIdx, int min[3]){
		if(val>=vals(2))
			return;
		if(val<vals(0)){
			vals(2) = vals(1);
			vals(1) = vals(0);
			vals(0) = val;
			min[2] = min[1];
			min[1] = min[0];
			min[0] = pIdx;
		} else if( val<vals(1) ){
			vals(2) = vals(1);
			vals(1) = val;
			min[2] = min[1];
			min[1] = pIdx;
		} else {
			vals(2) = val;
			min[2] = pIdx;
		}
	}

	void addMaxVal(double val, Vector3D<>& vals, int pIdx, int max[3]){
		if(val<=vals(2))
			return;
		if(val>vals(0)){
			vals(2) = vals(1);
			vals(1) = vals(0);
			vals(0) = val;
			max[2] = max[1];
			max[1] = max[0];
			max[0] = pIdx;
		} else if( val>vals(1) ){
			vals(2) = vals(1);
			vals(1) = val;
			max[2] = max[1];
			max[1] = pIdx;
		} else {
			vals(2) = val;
			max[2] = pIdx;
		}
	}

}

void GiftWrapHull3D::rebuild(const std::vector<rw::math::Vector3D<> >& vertices){
	_vertices = vertices;
	_tris.clear();
	_edgeSet.clear();


	// first we need to find a starting edge.
	// we find the two min of an axis
	int min[3]={0,0}, max[3]={0,0,0};
	Vector3D<> minV=_vertices[0],maxV=_vertices[0];
	std::cout << "[" << 0 << "]" << _vertices[0] << std::endl;
	for(size_t i=1;i<_vertices.size();i++){
		std::cout << "[" << i << "]" << _vertices[i] << std::endl;
		Vector3D<> p = _vertices[i];
		addMinVal(p(0), minV, i, min);
		addMaxVal(p(0), maxV, i, max);
	}
	// we now have two triangles min and max in the x-axis
	// order the indexes such that we form a triangle with an outer normal
	Vector3D<> nMinX(-1,0,0);
	int v3start = search(EdgeIdx(min[0],min[1]),nMinX);

	// check that this actually worked...
	Vector3D<> n = calcNormal(_vertices[min[0]],_vertices[min[1]],_vertices[v3start]);
	std::cout << min[0] << "," << min[1] << "," << v3start << n << std::endl;
	std::cout << _vertices[min[0]] << std::endl
			  << _vertices[min[1]] << std::endl
			  << _vertices[v3start] << std::endl;
	if( n(0)>0 ){
		std::swap(min[1],v3start);
	}
	n = calcNormal(_vertices[min[0]],_vertices[min[1]],_vertices[v3start]);
	std::cout << min[0] << "," << min[1] << "," << v3start << n << std::endl;
	RW_ASSERT( n(0)<=0 );

	// now we have a starting triangle on the convex hull iterate from that
	_tris.push_back(TriangleIdx(min[0],min[1],v3start,n));
	_edgeStack.push( make_pair(EdgeIdx(min[0],min[1]),0) );
	_edgeStack.push( make_pair(EdgeIdx(min[1],v3start),0) );
	_edgeStack.push( make_pair(EdgeIdx(v3start,min[0]),0) );

	while( !_edgeStack.empty() ){
		std::pair<EdgeIdx,int> edge = _edgeStack.top();
		_edgeStack.pop();

		int v1 = edge.first.second,
			v2 = edge.first.first;
		int v3 = search(edge.first,_tris[edge.second]._n);

		n = calcNormal(_vertices[v1], _vertices[v2], _vertices[v3]);
		_tris.push_back( TriangleIdx(v1, v2, v3, n) );

		addEdge(EdgeIdx(v1,v2),_edgeStack, _edgeSet, _tris.size()-1);
		addEdge(EdgeIdx(v2,v3),_edgeStack, _edgeSet, _tris.size()-1);
		addEdge(EdgeIdx(v3,v1),_edgeStack, _edgeSet, _tris.size()-1);
	}
}

namespace {
	/**
	 * @brief calculate the distance to the halfspace that the triangle is in
	 * @param x
	 * @return
	 */
	double halfSpaceDist(const rw::math::Vector3D<>& x,
						 const rw::math::Vector3D<>& n,
						 const rw::math::Vector3D<>& v){
		//std::cout<< "dot(_normal,x): " << dot(_normal,x) << " " << _d << std::endl;
		return dot(n,x) - dot(n, v);
	}
}

bool GiftWrapHull3D::isInside(const rw::math::Vector3D<>& vertex){
	const static double EPSILON = 0.0000001;
	if( _tris.size()==0 )
		return 0;
	BOOST_FOREACH( TriangleIdx &face, _tris ){
		double dist =
			halfSpaceDist( vertex, face._n, _vertices[face._vIdx[0]] );
		if( dist > EPSILON )
			return false;
	}
	return true;
}

double GiftWrapHull3D::getMinDist(const rw::math::Vector3D<>& vertex){
	const static double EPSILON = 0.0000001;
	if( _tris.size()==0 )
		return 0;
	double minDist = halfSpaceDist( vertex, _tris[0]._n, _vertices[_tris[0]._vIdx[0]] );
	BOOST_FOREACH( TriangleIdx &face, _tris ){
		double dist =
			halfSpaceDist( vertex, face._n, _vertices[face._vIdx[0]] );
		if( dist > EPSILON )
			return 0;
		if(dist>minDist){
			minDist = dist;
		}
	}
	return fabs(minDist);
}

rw::geometry::PlainTriMesh<TriangleN1<> >* GiftWrapHull3D::toTriMesh(){
	PlainTriMesh<TriangleN1<> > *mesh = new PlainTriMesh<TriangleN1<> >(_tris.size());
	for(size_t i=0;i<_tris.size();i++){
		int v1 = _tris[i]._vIdx[0];
		int v2 = _tris[i]._vIdx[1];
		int v3 = _tris[i]._vIdx[2];
		(*mesh)[i] = TriangleN1<>(_vertices[v1],_vertices[v2],_vertices[v3],_tris[i]._n);
	}
	return mesh;
}

int GiftWrapHull3D::search(const EdgeIdx& edge, const rw::math::Vector3D<>& normal){
	// run p through all points and find the triangle(edge,p) that has the
	// smallest angle with the vector normal
	double minVal=10,dist=1000;
	int candIdx=0;

	for(size_t i=0;i<_vertices.size();i++){
		if(i==(size_t)edge.first || i==(size_t)edge.second)
			continue;
		const Vector3D<>& cand = _vertices[i];
		// calculate the normal of the triangle (first,second,cand)
		Vector3D<> candNorm = calcNormal(_vertices[edge.first],_vertices[edge.second],cand);
		double dotAng = dot(normal, candNorm);
		if(dotAng > minVal+0.00001)
			continue;
		/*if( fabs(dotAng-minVal)<0.0001 ){
			// TODO: the value is the same, check if this point is closer than the other
			double d = dot(ac,ab)/MetricUtil::norm2(ab);
		}*/

		minVal = dotAng;
		candIdx = i;
		// TODO: if more than one edge is found we use the closest
	}
	return candIdx;
}
