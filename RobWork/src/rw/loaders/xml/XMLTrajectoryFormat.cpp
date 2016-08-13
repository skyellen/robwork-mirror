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

XMLTrajectoryFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		static XercesInitializer initializer;
		idQTrajectory();
		idV3DTrajectory();
		idR3DTrajectory();
		idT3DTrajectory();
		idQLinearInterpolator();
		idQCubicSplineInterpolator();
		idV3DLinearInterpolator();
		idV3DCubicSplineInterpolator();
		idV3DCircularInterpolator();
		idR3DLinearInterpolator();
		idR3DCubicSplineInterpolator();
		idT3DLinearInterpolator();
		idT3DCubicSplineInterpolator();
		idParabolicBlend();
		idLloydHaywardBlend();
		idDurationAttribute();
		idStartTimeAttribute();
		idTauAttribute();
		idKappaAttribute();
		done = true;
	}
}

const XMLTrajectoryFormat::Initializer XMLTrajectoryFormat::initializer;

const XMLCh* XMLTrajectoryFormat::idQTrajectory() {
	static const XMLStr id("QTrajectory");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idV3DTrajectory() {
	static const XMLStr id("V3DTrajectory");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idR3DTrajectory() {
	static const XMLStr id("R3DTrajectory");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idT3DTrajectory() {
	static const XMLStr id("T3DTrajectory");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idQLinearInterpolator() {
	static const XMLStr id("QLinearInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idQCubicSplineInterpolator() {
	static const XMLStr id("QCubicSplineInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idV3DLinearInterpolator() {
	static const XMLStr id("V3DLinearInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idV3DCubicSplineInterpolator() {
	static const XMLStr id("V3DCubicSplineInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idV3DCircularInterpolator() {
	static const XMLStr id("V3DCircularInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idR3DLinearInterpolator() {
	static const XMLStr id("R3DLinearInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idR3DCubicSplineInterpolator() {
	static const XMLStr id("R3DCubicSplineInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idT3DLinearInterpolator() {
	static const XMLStr id("T3DLinearInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idT3DCubicSplineInterpolator() {
	static const XMLStr id("T3DCubicSplineInterpolator");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idParabolicBlend() {
	static const XMLStr id("ParabolicBlend");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idLloydHaywardBlend() {
	static const XMLStr id("LloydHaywardBlend");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idDurationAttribute() {
	static const XMLStr id("duration");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idStartTimeAttribute() {
	static const XMLStr id("starttime");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idTauAttribute() {
	static const XMLStr id("tau");
	return id.uni();
}

const XMLCh* XMLTrajectoryFormat::idKappaAttribute() {
	static const XMLStr id("kappa");
	return id.uni();
}
