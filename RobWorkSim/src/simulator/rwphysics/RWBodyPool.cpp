/*
 * RWBodyPool.cpp
 *
 *  Created on: Jun 8, 2009
 *      Author: jimali
 */

#include "RWBodyPool.hpp"


RWBodyPool::RWBodyPool(int nrBodies):_bodies(nrBodies){
	// add all bodes to the free list
	for(int i=nrBodies-1;i>=0;i--){
		_bodies[i] = new RWBody(i);
		_freeBodyIDs.push(i);
	}
}

RWBody *RWBodyPool::createBody(RWBody::BodyType type){
	RWBody *body;
	if(_freeBodyIDs.empty()){
		// we need to add a new body to the body vector
		body = new RWBody( _bodies.size() );
		_bodies.push_back(body);
		return body;
	}
	// else
	int idx = _freeBodyIDs.top();
	_freeBodyIDs.pop();
	body = _bodies[idx];
	body->setType(type);
	return body;
}

void RWBodyPool::deleteBody(RWBody* body){
	// todo: check if body is allready deleted
	int bidx = body->getId();
	RW_ASSERT(0<=bidx && bidx<_bodies.size());
	_freeBodyIDs.push(bidx);
}

const RWBodyList& RWBodyPool::getBodies() const{
	return _bodies;
}
