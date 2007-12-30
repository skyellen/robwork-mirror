#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCEOPTIMIZATION_HPP
#define RWLIBS_PATHOPTIMIZATION_CLEARANCEOPTIMIZATION_HPP

#include <rw/pathplanning/Path.hpp>
#include <rw/math/Metric.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/DistanceCalculator.hpp>

#include "ClearanceCalculator.hpp"

namespace rwlibs {
namespace pathoptimization {
    

    
/**
 * @brief The ClearanceOptimizer implements the C-Retraction algorithms from [1]
 * 
 * [1]: R. Geraerts and M.H. Overmars, Creating High-Quality Paths for Motion Planning,
 * The International Journal of Robotics Research, Vol. 26, No. 8, 845-863 (2007)
 * 
 * The algorithms work by first subdividing the path, to give a dense and even distribution
 * of nodes along the path. Nodes are then tried moved in a random direction to improve the 
 * clearance. After having iterated through the entire path some nodes will be moved, thus
 * a validation step is used to insert extra nodes where the density is not high enough. This 
 * is then followed by a method for removing undesired branches.
 */
class ClearanceOptimizer {
public:
	
    /**
     * @brief Constructs clearance optimizer
     * 
     * The clearance optimizer currently assumes the configuration space of the device is rectangular.          ´ 
     * 
     * @param workcell [in] WorkCell to use
     * @param device [in] Device to plan for
     * @param state [in] State containing position of all other devices and how frames are assembled.
     * @param metric [in] Metric to use for computing distance betweem configurations
     * @param clearanceCalculator [in] Calculator for calculating the clearance 
     * @param stepsize [in] Maximum size between configurations in the dense path
     * @param maxcount [in] Number of time to attempt optimizing the path using the random direction. 
     */
    ClearanceOptimizer(rw::models::WorkCell* workcell,
						  rw::models::Device* device,
						  const rw::kinematics::State& state,
						  boost::shared_ptr<rw::math::Metric<double> > metric, 
						  boost::shared_ptr<ClearanceCalculator> clearanceCalculator,
						  double stepsize,
						  size_t maxcount);
	
    /**
     * @brief Destructor
     */
	~ClearanceOptimizer();
	
	/**
	 * @brief Runs optimization algorithm
	 * 
	 * Calling this method runs the path optimization algorithm. This call blocks until the optimized
	 * path is ready. This may take quite a while, depending on the \b maxcount specified and the amount 
	 * of geometry in the scene.
	 * 
	 * @param path [in] Path to optimize
	 * @return The optimized path with node no further than \b stepsize apart 
	 */
	rw::pathplanning::Path optimize(const rw::pathplanning::Path& path);
private:

    //AugmentedQ is a configuration and its clearance
    typedef std::pair<rw::math::Q, double> AugmentedQ;
    
    //Path of AugmentedQ's
    typedef std::list<AugmentedQ > AugmentedPath;
    
    //Returns whether a q is valid, that is within bounds
	bool isValid(const rw::math::Q& q);
	
	//Calculated the clearance for a configuration q
	double clearance(const rw::math::Q& q);
	
	//Performs a subdivision of the path and augments all configs with the clearance
	void subDivideAndAugmentPath(const rw::pathplanning::Path& inputPath, AugmentedPath& path);
	
	//Validates the path, by going through it and insert node where they are further than stepsize apart
	AugmentedPath validatePath(const AugmentedPath& newPath, const AugmentedPath& orgPath);

	//Removed branches 
	void removeBranches(AugmentedPath& path);
	
	//Calculates the avarage clearance of the path	
	double calcAvgClearance(const AugmentedPath& path);
	
	//Performs an interpolator of two configurations.
	rw::math::Q interpolate(const rw::math::Q& q1, const rw::math::Q& q2, double ratio);
	
	//Returns a random direction	
	rw::math::Q randomDirection();
	
	

	rw::models::WorkCell* _workcell;
	rw::models::Device* _device;	
	rw::kinematics::State _state;
	boost::shared_ptr<rw::math::Metric<double> > _metric;
	boost::shared_ptr<ClearanceCalculator> _clearanceCalculator;
	double _stepsize;
	size_t _maxcount;
	size_t _dof;
	
	rw::math::Q _qupper;
	rw::math::Q _qlower;
	
};

} //end namespace pathoptimization
} //end namespace rwlibs


#endif //#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCEOPTIMIZATION_HPP
