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

#include "XMLTrajectoryFormat.hpp"

#include "XercesUtils.hpp"

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

