#ifndef RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP
#define RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP

#include <rw/math/Metric.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/Path.hpp>
#include <rw/pathplanning/PathPlanner.hpp>
#include <boost/shared_ptr.hpp>



namespace rwlibs {
namespace pathoptimization {

/**
 * @brief The PathLengthOptimizer implements the 3 different path length optimizers 
 * presented in [1].
 * 
 * [1]: R. Geraerts and M.H. Overmars, Creating High-Quality Paths for Motion Planning,
 * The International Journal of Robotics Research, Vol. 26, No. 8, 845-863 (2007)
 * 
 * The simplest algorithm \b pathPruning runs through the path an tests if nodes with 
 * index i and i+2 can be directly connected. If so it removed node i+1.
 * 
 * The \b shortCut algorithm works similary except that it takes two random indices
 * i and j and tries to connect those. This algorithm is non-deterministic but more 
 * powerful than pathPruning.
 * 
 * The \b partialShortCut algorithm select two random node indices i and j and a random
 * position in the configuration vector. A shortcut is then only tried between the values 
 * corresponding to the random position. This algorithm is generally more powerful than 
 * shortCut but may in some cases be more computational expensive. 
 * 
 */
class PathLengthOptimizer
{
public:
	/**
	 * @brief Construct PathLengthOptimizer
	 * 
	 * Based on the parameters a rw::pathplanning::StraightLinePathPlanner is constructed
	 * to be used a local planner between nodes.
	 * 
	 * @param device [in] Device corresponding to the path
	 * @param state [in] State showing how the workcell is connected
	 * @param collisionDetector [in] CollisionDetector to use
	 * @param resolution [in] The resolution used ot test the path
	 */
	PathLengthOptimizer(rw::models::Device* device,
						const rw::kinematics::State& state,
						boost::shared_ptr<rw::proximity::CollisionDetector> collisionDetector,
						double resolution,
						boost::shared_ptr<rw::math::Metric<double> > metric);

	/**
	 * @brief Construct PathLengthOptimizer
	 * 
	 * Constructs a PathLengthOptimizer with a PathPlanner to use as local planner
	 * between configurations. The planner needs to be deterministic as it is only used to see
	 * if two nodes can be connected. The path it may return is not stored. 
	 * 
	 * It is recommond to use a simple planner such as the rw::pathplanning::StraightLinePathPlanner . 
	 * 
	 * @param localplanner [in] The local planner. PathLengthOptimizer takes owner ship of the
	 * localplanner.
	 */
	PathLengthOptimizer(rw::pathplanning::PathPlanner* localplanner,
	                    boost::shared_ptr<rw::math::Metric<double> > metric);
	
	/**
	 * @brief Destructor
	 */
	virtual ~PathLengthOptimizer();
	
	/**
	 * @brief Optimizes using path pruning.
	 * 
	 * \b pathPruning runs through the path an tests if nodes with 
	 * index i and i+2 can be directly connected. If so it removed node i+1.
	 * 
	 * @param path [in] Path to optimize
	 * @return The optimized path 
	 */
	rw::pathplanning::Path pathPruning(const rw::pathplanning::Path& path);

	/**
	 * @brief Optimizes using the shortcut technique
	 * 
	 * The \b shortCut algorithm works by selecting two random indices i and j and 
	 * try to connect those. 
	 * 
	 * The algorithm will loop until either the specified \b cnt is of met or the specified
	 * time is reached.
	 * 
	 * @param path [in] Path to optimize
	 * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
	 * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
	 * @return The optimized path 
 	 */
	rw::pathplanning::Path shortCut(const rw::pathplanning::Path& path, size_t cnt, double time, double subDivisionSize);
	
	/**
	 * @brief Optimizes using the partial shortcut technique
	 * 
	 * The \b partialShortCut algorithm select two random node indices i and j and a random
	 * position in the configuration vector. A shortcut is then only tried between the values 
	 * corresponding to the random position.  
	 * 
	 * The algorithm will loop until either the specified \b cnt is of met or the specified
	 * time is reached.
	 * 
	 * @param path [in] Path to optimize
	 * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
	 * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
	 * @return The optimized path 
 	 */	 
	rw::pathplanning::Path partialShortCut(const rw::pathplanning::Path& path, size_t cnt, double time, double subDivisionSize);

	
private:
	rw::models::WorkCell* _workcell;
	rw::models::Device* _device;
	rw::kinematics::State _state;
	boost::shared_ptr<rw::math::Metric<double> > _metric;
	boost::shared_ptr<rw::proximity::CollisionDetector> _collisionDetector;
	double _stepsize;
	
	rw::pathplanning::PathPlanner* _localplanner;
	
	void initialize();
	
	void resamplePath(rw::pathplanning::Path& path, double subDivisionSize);
	
	rw::pathplanning::Path::iterator resample(rw::pathplanning::Path::iterator it1, 
	                                          rw::pathplanning::Path::iterator it2, 
	                                          double subDivisionSize,
	                                          rw::pathplanning::Path& result);
	

	
};

} //end namespace pathoptimization
} //end namespace rwlibs

#endif /*PATHLENGTHOPTIMIZER_HPP_*/
