#ifndef RW_PATHPLANNING_PATHANALYZER_HPP_
#define RW_PATHPLANNING_PATHANALYZER_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Metric.hpp>
#include <rw/models/Device.hpp>
#include <rw/pathplanning/Path.hpp>
#include <rw/proximity/DistanceCalculator.hpp>

/**
 * @file PathAnalyzer.hpp
 */

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
    /**
     * @brief Result struct for joint space analysis
     */
    struct JointSpaceAnalysis {
       /** Number of nodes */
       int nodecount;
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
	PathAnalyzer(rw::models::Device* device, const rw::kinematics::State& state);

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
	JointSpaceAnalysis analyzeJointSpace(rw::pathplanning::Path& path, rw::math::Metric<double>* metric = NULL);

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
	CartesianAnalysis analyzeCartesian(rw::pathplanning::Path& path, rw::kinematics::Frame* frame);
	
	/**
	 * @brief Peforms analysis of the time
	 * 
	 * Calculates the time needed to execute the path when considering the dynamics.
	 * 
	 * @param path [in] Path to analyze.
	 */
	TimeAnalysis analyzeTime(rw::pathplanning::Path& path);
	
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
	ClearanceAnalysis analyzeClearance(rw::pathplanning::Path& path, rw::proximity::DistanceCalculator* distanceCalculator);
		
private:
    rw::models::Device* _device;
    rw::kinematics::State _state;
};


/** @} */

} //end namespace pathplanning
} //end namespace rw

#endif /*RW_PATHPLANNING_PATHANALYZER_HPP*/
