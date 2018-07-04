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


#ifndef RW_PATHPLANNING_PATHANALYZER_HPP_
#define RW_PATHPLANNING_PATHANALYZER_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Metric.hpp>
#include <rw/trajectory/Path.hpp>

/**
 * @file PathAnalyzer.hpp
 */

namespace rw { namespace models { class Device; } }
namespace rw { namespace proximity { class DistanceCalculator; } }

namespace rw {
namespace pathplanning {


/** @addtogroup pathplanning */
/*@{*/

/**
 * @brief The PathAnalyzer provides a set a basic tools for analyzing a path.
 *
 * Features in the PathAnalyzer include analysis of joint space, Cartesian space,
 * estimation of execution time and measures for clearance. See more details in
 * the result structs PathAnalyzer::JointSpaceAnalysis, PathAnalyzer::CartesianAnalysis,
 * PathAnalyzer::TimeAnalysis and PathAnalyzer::ClearanceAnalysis.
 */
class PathAnalyzer
{
public:

	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<PathAnalyzer> Ptr;
	//! @brief smart pointer type to this const class
    typedef rw::common::Ptr<const PathAnalyzer> CPtr;

    /**
     * @brief Result struct for joint space analysis
     */
    struct JointSpaceAnalysis {
       /** Number of nodes */
       double nodecount;
       /** Total length in joint space */
       double length;

       /** Constructs JointSpaceAnalysis struct initialized to zero. */
       JointSpaceAnalysis() {
           nodecount = 0;
           length = 0;
       }
    };

    /**
     * @brief Result struct for Cartesian analysis
     */
    struct CartesianAnalysis {
        /** Cartesian length of the Path */
        double length;

        /** Total distance travelled in the x,y and z directions */
        rw::math::Vector3D<> distances;

        /** Lower bound on the Cartesian position */
        rw::math::Vector3D<> lower;
        /** Upper bound on the Cartesian position */
        rw::math::Vector3D<> upper;

        /** Construct CartesianAnalysis struct with length initialized to 0*/
        CartesianAnalysis() {
            length = 0;
        }
    };

    /**
     * @brief Result struct for Time analysis
     */
    struct TimeAnalysis {
        /** Time to execute path when considering only velocity limits */
        double time1;
        /**
         * Time to execute path when considering both velocity and acceleration limits.
         * NOT IMPLEMENTED YET
         */
        double time2;

        /** Construct TimeAnalysis struct with times initialized to 0 */
        TimeAnalysis() {
            time1 = 0;
            time2 = 0;
        }
    };

    /**
     * @brief Result struct for CleracenAnalysis
     */
    struct ClearanceAnalysis {
        /** Average clearance */
        double average;
        /** Minimum clearance */
        double min;

        /** Construct ClearanceAnalysis struct with distances initialized to 0 */
        ClearanceAnalysis() {
            average = 0;
            min = 0;
        }
    };

public:
    /**
     * @brief Construct PathAnalyzer for a specific device
     *
     * @param device [in] Device to be associated with the path
     * @param state [in] State of the workcell
     */
	PathAnalyzer(const rw::common::Ptr<const rw::models::Device>& device, const rw::kinematics::State& state);

	/**
	 * @brief Destructor
	 */
	virtual ~PathAnalyzer();

	/**
	 * @brief Performs joint space analysis of path.
	 *
	 * @param path [in] Path to analyze
	 * @param metric [in] Metric to use for calculating the distance in joint space.
	 * @return Result of the joint space analysis
	 */
	JointSpaceAnalysis analyzeJointSpace(const rw::trajectory::QPath& path,
		rw::math::QMetric::Ptr metric = NULL) const;

	/**
	 * @brief Performs analysis in Cartesian space.
	 *
	 * The method calculates the Cartesian distance travelled by the speficied \b frame. It provides
	 * the total distance travelled, the coordinate wise distances, and upper and lower bounds
	 * on the location of the frame during the path.
	 *
	 * @param path [in] Path to analyze
	 * @param frame [in] Frame for which to analyze the path.
	 * @return Result of the analysis.
	 */
	CartesianAnalysis analyzeCartesian(const rw::trajectory::QPath& path, const rw::kinematics::Frame* frame);

	/**
	 * @brief Peforms analysis of the time
	 *
	 * Calculates the time needed to execute the path when considering the dynamics.
	 *
	 * @param path [in] Path to analyze.
	 */
	TimeAnalysis analyzeTime(const rw::trajectory::QPath& path) const;

	/**
	 * @brief Performs an analysis of the clearance
	 *
	 * Calculates the average and minimum clearance along a path. Only the nodes of the path
	 * are sampled. The path should therefore be divided with the desired resolution before
	 * invoking this method.
	 *
	 * @param path [in] Path to analyze
	 * @param distanceCalculator [in] DistanceCalculator to be used in the analysis
	 * @return Result of the analysis.
	 */
	ClearanceAnalysis analyzeClearance(const rw::trajectory::QPath& path, const rw::common::Ptr<const rw::proximity::DistanceCalculator>& distanceCalculator);

    //TODO: Move to path statistics
	/**
	@brief The length of the path from \b begin up to and excluding \b end.
 		This method will in a new version be moved to a collection of tools for path
 		statistic.

		If the range [\b begin, \b end) is of length 0 or 1 then a length of
		0 is returned.

		Note that the element pointed to by \b end is not included in the
		path and need not exist.

		The distance between adjacent element is measured by \b metric.
	*/
	template <class It>
	static
	double pathLength(
	It begin,
	It end,
	const rw::math::Metric<typename It::value_type>& metric)
	{
	// If the sequence is empty:
	if (begin == end) return 0;

	It p = begin;
	It q = p; ++q;

	double result = 0;
	for (; q != end; ++p, ++q)
	result += metric.distance(*p, *q);
	return result;
	}


private:
	rw::common::Ptr<const rw::models::Device> _device;
    rw::kinematics::State _state;
};


/** @} */

} //end namespace pathplanning
} //end namespace rw

#endif /*RW_PATHPLANNING_PATHANALYZER_HPP*/
