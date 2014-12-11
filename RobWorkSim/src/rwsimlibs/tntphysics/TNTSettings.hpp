/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTSETTINGS_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTSETTINGS_HPP_

/**
 * @file TNTSettings.hpp
 *
 * Different settings for the engines TNTPhysics, TNTWorld and TNTIsland.
 *
 * This allows easy enable and disable of debugging and additional sanity checks.
 * Under normal circumstances such features should be disabled as they make the
 * engines inefficient.
 */

// Constraint settings
#define TNT_SPRING_EIGENVALUE_SQRT_THRESHOLD 1e-6

// Solver settings
#define TNT_SVD_PRECISSION 1e-8

// Rollback
#define TNT_ENABLE_ROLLBACK 1

// Constraint Correction
#define TNT_ENABLE_CONSTRAINT_CORRECTION 1

// Iterations
#define TNT_MAX_ITERATIONS 10

// Contact resolution
#define TNT_CONTACTRESOLVER_MAX_ITERATIONS 250
#define TNT_CONSTRAINT_MAX_FORCE 1000 // Newton

// Bodies
#define TNT_WORKSPACE 50 // 50x50x50 meters
#define TNT_MAX_LINVEL 100 // meters per second
#define TNT_MAX_ANGVEL 100 // radians per second

// Sanity Checks
#define TNT_CHECK_HIGH 3
#define TNT_CHECK_MID 2
#define TNT_CHECK_LOW 1
#define TNT_CHECK_LEVEL TNT_CHECK_HIGH

// Debugging Settings
#define TNT_DEBUG_ENABLE false
//#define TNT_DEBUG_FILE "tntdebug.txt"
#define TNT_DEBUG_TIMING_LIMIT 2 // Lower limit for timing values to output in ms

#if TNT_DEBUG_ENABLE
#define TNT_DEBUG_ENABLE_GENERAL true
#define TNT_DEBUG_ENABLE_TIMING true
#define TNT_DEBUG_ENABLE_CONTACTS true
#define TNT_DEBUG_ENABLE_ROLLBACK true
#define TNT_DEBUG_ENABLE_BOUNCING true
#define TNT_DEBUG_ENABLE_SOLVER true
#define TNT_DEBUG_ENABLE_INTEGRATOR true
#define TNT_DEBUG_ENABLE_CORRECTION true
#else
#define TNT_DEBUG_ENABLE_GENERAL false
#define TNT_DEBUG_ENABLE_TIMING false
#define TNT_DEBUG_ENABLE_CONTACTS false
#define TNT_DEBUG_ENABLE_ROLLBACK false
#define TNT_DEBUG_ENABLE_BOUNCING false
#define TNT_DEBUG_ENABLE_SOLVER false
#define TNT_DEBUG_ENABLE_INTEGRATOR false
#define TNT_DEBUG_ENABLE_CORRECTION false
#endif

// Debugging Macros
#if TNT_DEBUG_ENABLE

#ifdef TNT_DEBUG_FILE
#define TNT_DEBUG(header,ostreamExpression)                                    \
{                                                                              \
	int TNT__line = __LINE__;                                                  \
	std::stringstream TNT__stream;                                             \
	TNT__stream << header << ostreamExpression;                                \
	rw::common::Message TNT__message(__FILE__, TNT__line, TNT__stream.str());  \
	std::ofstream TNT__filestream;                                             \
	TNT__filestream.open(TNT_DEBUG_FILE,std::ios_base::app);                   \
	TNT__filestream << TNT__message;                                           \
	TNT__filestream.close();                                                   \
}
#define TNT_DEBUG_DELIMITER()                                                  \
{                                                                              \
	std::ofstream TNT__filestream;                                             \
	TNT__filestream.open(TNT_DEBUG_FILE,std::ios_base::app);                   \
	TNT__filestream << "---------------------------------------" << std::endl; \
	TNT__filestream.close();                                                   \
}
#else
#define TNT_DEBUG(header,ostreamExpression)                                    \
{                                                                              \
	int TNT__line = __LINE__;                                                  \
	std::stringstream TNT__stream;                                             \
	TNT__stream << header << ostreamExpression;                                \
	rw::common::Message TNT__message(__FILE__, TNT__line, TNT__stream.str());  \
	rw::common::Log::debugLog().write( TNT__message);                          \
}
#define TNT_DEBUG_DELIMITER()                                                  \
{                                                                              \
	std::stringstream TNT__stream;                                             \
	TNT__stream << "---------------------------------------" << std::endl;     \
	rw::common::Log::debugLog().write(TNT__stream.str());                      \
}
#endif

#ifdef TNT_DEBUG_ENABLE_GENERAL
#define TNT_DEBUG_GENERAL(ostreamExpression) TNT_DEBUG("GENERAL - ",ostreamExpression)
#else
#define TNT_DEBUG_GENERAL(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_TIMING
#define TNT_DEBUG_TIMING(ostreamExpression) TNT_DEBUG("TIMING - ",ostreamExpression)
#else
#define TNT_DEBUG_TIMING(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_CONTACTS
#define TNT_DEBUG_CONTACTS(ostreamExpression) TNT_DEBUG("CONTACTS - ",ostreamExpression)
#else
#define TNT_DEBUG_CONTACTS(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_ROLLBACK
#define TNT_DEBUG_ROLLBACK(ostreamExpression) TNT_DEBUG("ROLLBACK - ",ostreamExpression)
#else
#define TNT_DEBUG_ROLLBACK(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_BOUNCING
#define TNT_DEBUG_BOUNCING(ostreamExpression) TNT_DEBUG("BOUNCING - ",ostreamExpression)
#else
#define TNT_DEBUG_BOUNCING(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_SOLVER
#define TNT_DEBUG_SOLVER(ostreamExpression) TNT_DEBUG("SOLVER - ",ostreamExpression)
#else
#define TNT_DEBUG_SOLVER(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_INTEGRATOR
#define TNT_DEBUG_INTEGRATOR(ostreamExpression) TNT_DEBUG("INTEGRATOR - ",ostreamExpression)
#else
#define TNT_DEBUG_INTEGRATOR(ostreamExpression)
#endif

#ifdef TNT_DEBUG_ENABLE_CORRECTION
#define TNT_DEBUG_CORRECTION(ostreamExpression) TNT_DEBUG("CORRECTION - ",ostreamExpression)
#else
#define TNT_DEBUG_CORRECTION(ostreamExpression)
#endif

#else // !TNT_DEBUG_ENABLE
#define TNT_DEBUG(type,ostreamExpression)
#define TNT_DEBUG_DELIMITER()

#define TNT_DEBUG_GENERAL(ostreamExpression)
#define TNT_DEBUG_TIMING(ostreamExpression)
#define TNT_DEBUG_CONTACTS(ostreamExpression)
#define TNT_DEBUG_ROLLBACK(ostreamExpression)
#define TNT_DEBUG_BOUNCING(ostreamExpression)
#define TNT_DEBUG_SOLVER(ostreamExpression)
#define TNT_DEBUG_INTEGRATOR(ostreamExpression)
#define TNT_DEBUG_CORRECTION(ostreamExpression)

#endif

// Time Measurement Macros
#ifdef TNT_DEBUG_ENABLE_TIMING
#define TNT_TIMING( str, func )										\
    {																\
		const long start = rw::common::TimerUtil::currentTimeMs();	\
		func;														\
		const long end = rw::common::TimerUtil::currentTimeMs();	\
		if (end-start > TNT_DEBUG_TIMING_LIMIT) {					\
    		TNT_DEBUG_TIMING(str <<": " << (end-start) <<" ms")		\
    	}															\
    }
#else
#define TNT_TIMING( str, func ) {func;}
#endif

#endif /* RWSIMLIBS_TNTPHYSICS_TNTSETTINGS_HPP_ */
