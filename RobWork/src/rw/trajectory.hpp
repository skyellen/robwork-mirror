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
/**
 * @file rw/trajectory.hpp
 *
 * this file includes all header files from the trajectory namespace
 */


#ifndef RW_TRAJECTORY_HPP_
#define RW_TRAJECTORY_HPP_

#include "./trajectory/Blend.hpp"
#include "./trajectory/BlendedTrajectory.hpp"
#include "./trajectory/CircularInterpolator.hpp"
#include "./trajectory/CubicSplineFactory.hpp"
#include "./trajectory/CubicSplineInterpolator.hpp"
#include "./trajectory/Interpolator.hpp"
#include "./trajectory/InterpolatorUtil.hpp"
#include "./trajectory/LinearInterpolator.hpp"
#include "./trajectory/LloydHaywardBlend.hpp"
#include "./trajectory/ParabolicBlend.hpp"
#include "./trajectory/Path.hpp"
#include "./trajectory/Timed.hpp"
#include "./trajectory/TimedUtil.hpp"
#include "./trajectory/Trajectory.hpp"
#include "./trajectory/TrajectoryFactory.hpp"
#include "./trajectory/TrajectoryIterator.hpp"
#include "./trajectory/TrajectorySequence.hpp"
#include "./trajectory/RampInterpolator.hpp"


#endif /* TRAJECTORY_HPP_ */
