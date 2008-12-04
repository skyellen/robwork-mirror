/*
 * XMLTrajectoryFormat.cpp
 *
 *  Created on: Nov 27, 2008
 *      Author: lpe
 */

#include "XMLTrajectoryFormat.hpp"

using namespace rw::loaders;
using namespace xercesc;

const XercesInitializer XMLTrajectoryFormat::initializer;

const XMLCh* XMLTrajectoryFormat::QTrajectoryId = XMLString::transcode("QTrajectory");
const XMLCh* XMLTrajectoryFormat::V3DTrajectoryId = XMLString::transcode("V3DTrajectory");

const XMLCh* XMLTrajectoryFormat::R3DTrajectoryId = XMLString::transcode("R3DTrajectory");
const XMLCh* XMLTrajectoryFormat::T3DTrajectoryId = XMLString::transcode("T3DTrajectory");

const XMLCh* XMLTrajectoryFormat::QLinearInterpolatorId = XMLString::transcode("QLinearInterpolator");
const XMLCh* XMLTrajectoryFormat::QCubicSplineInterpolatorId = XMLString::transcode("QCubicSplineInterpolator");

const XMLCh* XMLTrajectoryFormat::V3DLinearInterpolatorId = XMLString::transcode("V3DLinearInterpolator");
const XMLCh* XMLTrajectoryFormat::V3DCubicSplineInterpolatorId = XMLString::transcode("V3DCubicSplineInterpolator");
const XMLCh* XMLTrajectoryFormat::V3DCircularInterpolatorId = XMLString::transcode("V3DCircularInterpolator");

const XMLCh* XMLTrajectoryFormat::R3DLinearInterpolatorId = XMLString::transcode("R3DLinearInterpolator");
const XMLCh* XMLTrajectoryFormat::R3DCubicSplineInterpolatorId = XMLString::transcode("R3DCubicSplineInterpolator");

const XMLCh* XMLTrajectoryFormat::T3DLinearInterpolatorId = XMLString::transcode("T3DLinearInterpolator");
const XMLCh* XMLTrajectoryFormat::T3DCubicSplineInterpolatorId = XMLString::transcode("T3DCubicSplineInterpolator");


const XMLCh* XMLTrajectoryFormat::ParabolicBlendId = XMLString::transcode("ParabolicBlend");
const XMLCh* XMLTrajectoryFormat::LloydHaywardBlendId = XMLString::transcode("LloydHaywardBlend");


const XMLCh* XMLTrajectoryFormat::DurationAttributeId = XMLString::transcode("duration");
const XMLCh* XMLTrajectoryFormat::StartTimeAttributeId = XMLString::transcode("starttime");
const XMLCh* XMLTrajectoryFormat::TauAttributeId = XMLString::transcode("tau");
const XMLCh* XMLTrajectoryFormat::KappaAttributeId = XMLString::transcode("kappa");

