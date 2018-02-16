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


#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCEOPTIMIZATION_HPP
#define RWLIBS_PATHOPTIMIZATION_CLEARANCEOPTIMIZATION_HPP

#include <rw/common/PropertyMap.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/math/Metric.hpp>
//#include <rw/models/WorkCell.hpp>

#include <list>

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class Device; } }

namespace rwlibs {
namespace pathoptimization {
	class ClearanceCalculator;

    /** @addtogroup pathoptimization */
    /*@{*/


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
     * The clearance optimizer currently assumes the configuration space of the device is rectangular.
     *
     * @param device [in] Device to plan for
     * @param state [in] State containing position of all other devices and how frames are assembled.
     * @param metric [in] Metric to use for computing distance betweem configurations
     * @param clearanceCalculator [in] Calculator for calculating the clearance
     */
	ClearanceOptimizer(//rw::models::WorkCell::Ptr workcell,
		rw::common::Ptr<rw::models::Device> device,
		const rw::kinematics::State& state,
		rw::math::QMetric::Ptr metric,
		rw::common::Ptr<ClearanceCalculator> clearanceCalculator);

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
	 * @param stepsize [in] Maximum size between configurations in the dense path
     * @param maxcount [in] Number of time to attempt optimizing the path using the random direction. If \b maxcount=0 only the maxtime will be used.
	 * @param maxtime [in] The maximal time allowed to optimize. If \b maxtime<=0 only the \b maxcount will be used
	 * @return The optimized path with node no further than \b stepsize apart
	 */
	rw::trajectory::QPath optimize(const rw::trajectory::QPath& path,
	                                double stepsize,
	                                size_t maxcount,
	                                double maxtime);

	/**
	 * @brief Runs optimization algorithm
	 *
	 * Runs the optimization algorithm using the parameters specified in the property map
	 *
	 * @param path [in] Path to optimize
	 * @return The optimized path
	 */
	rw::trajectory::QPath optimize(const rw::trajectory::QPath& path);

   //!Property key for the maximal number of loops. Set LOOPCOUNT=0 to deactivate it
    static const std::string PROP_LOOPCOUNT;
    //!Property key for max time. Set MAXTIME=0 to deactivate it
    static const std::string PROP_MAXTIME;
    //!Property key for step size
    static const std::string PROP_STEPSIZE;

    /**
     * @brief Returns the PropertyMap associated with the optimizer
     * @return The PropertyMap
     */
	rw::common::PropertyMap& getPropertyMap();

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
	void subDivideAndAugmentPath(const rw::trajectory::QPath& inputPath, AugmentedPath& path);

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

	rw::common::PropertyMap _propertymap;

	//rw::models::WorkCell::Ptr _workcell;
	rw::common::Ptr<rw::models::Device> _device;
	rw::kinematics::State _state;
	rw::math::QMetric::Ptr _metric;
	rw::common::Ptr<ClearanceCalculator> _clearanceCalculator;
	double _stepsize;
	size_t _dof;

	rw::math::Q _qupper;
	rw::math::Q _qlower;

};

/** @} */

} //end namespace pathoptimization
} //end namespace rwlibs


#endif //#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCEOPTIMIZATION_HPP
