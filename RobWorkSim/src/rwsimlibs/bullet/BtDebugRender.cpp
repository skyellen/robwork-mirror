/*
 * BtDebugRender.cpp
 *
 *  Created on: 26-08-2008
 *      Author: jimali
 */

#include "BtDebugRender.hpp"

#include <btBulletDynamicsCommon.h>

#include "BtSimulator.hpp"

using namespace rwsim::simulator;
using namespace rwsim::drawable;

BtDebugRender::BtDebugRender(BtSimulator * sim):
    _sim(sim)
{
    //_debugDrawer = new GLDebugDrawer();
    //_debugDrawer->setDebugMode(1+2+4+8);
    //_sim->getBtWorld()->setDebugDrawer(_debugDrawer);
}

BtDebugRender::~BtDebugRender(){}

void BtDebugRender::draw(DrawType draw, double alpha) const {
    //_sim->getBtWorld()->debugDrawWorld();
}

void BtDebugRender::setDrawMask(unsigned int mask){
    _drawMask = mask;
}
