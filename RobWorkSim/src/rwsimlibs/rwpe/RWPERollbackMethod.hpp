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

#ifndef RWSIMLIBS_RWPE_RWPEROLLBACKMETHOD_HPP_
#define RWSIMLIBS_RWPE_RWPEROLLBACKMETHOD_HPP_

/**
 * @file RWPERollbackMethod.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPERollbackMethod
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/kinematics/FramePairMap.hpp>

// Forward declarations
namespace rwsim { namespace contacts { class Contact; } }

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The rollback method calculates new guesses for the correct step size based
 * on previous samples of the distance or penetration between objects.
 *
 * The simulator provides a set of samples. A sample is a timestamp and a list of distances
 * for all frame pairs that are in rollback. It is up to the rollback method to decide how many
 * samples are needed. For more complex algorithms it is possible to inherit the RollbackData
 * class to add user defined data that is stored between calls to getTimestep().
 */
class RWPERollbackMethod {
public:
    //! Smart pointer type of RWPERollbackMethod
    typedef rw::common::Ptr<const RWPERollbackMethod> Ptr;

	//! @brief A sample defines distances for all contact pairs for a specific time.
	struct Sample {
	    //! @brief Default constructor.
		Sample();

	    /**
	     * @brief Construct Sample from a list of contacts at the given time.
	     * @param time [in] the time.
	     * @param contacts [in] the list of contacts to construct Sample for.
	     */
		Sample(double time, const std::vector<rwsim::contacts::Contact>& contacts);

		//! @brief The time where the sample is taken (relative to begginning of step).
		double time;

		//! @brief A set of all frame pairs that has contacts used in rollback.
		std::set<std::pair<const rw::kinematics::Frame*, const rw::kinematics::Frame*> > framePairs;

		//! @brief A map giving the nearest distance or biggest penetration between all frame pairs.
		rw::kinematics::FramePairMap<double> distance;
	};

	//! @brief Comparison of samples that allows ordering by time.
	struct SampleCompare {
		/**
		 * @brief Comparison operator.
		 * @param s1 [in] first sample.
		 * @param s2 [in] second sample.
		 * @return true if s1 has time smaller than s2, false otherwise.
		 */
		bool operator()(const Sample& s1, const Sample& s2) const;
	};

	//! @brief A collection of samples - the input to the rollback method provided by the physics engine.
	typedef std::set<Sample, SampleCompare> SampleSet;

	//! @brief Data type for methods that need to store internal data during rollback.
	class RollbackData {
	public:
	    //! @brief Constructor.
		RollbackData() {};

		//! @brief Destructor.
		virtual ~RollbackData() {};
	};

    //! @brief Constructor.
	RWPERollbackMethod();

	//! @brief Destructor.
	virtual ~RWPERollbackMethod();

	/**
	 * @brief Create RollbackData.
	 * @return a pointer to a new data structure - owned by the caller.
	 */
	virtual RollbackData* createData() const;

	/**
	 * @brief Get a new timestep based on the current samples.
	 * @param samples [in/out] the samples known.
	 * @param data [in] a pointer to the data, see createData().
	 * @return a new timestep where the physics engine should find the new Sample.
	 */
	virtual double getTimestep(SampleSet& samples, RollbackData* data) const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::rwpe::RWPERollbackMethod::Factory,rwsimlibs::rwpe::RWPERollbackMethod,rwsimlibs.rwpe.RWPERollbackMethod}
	 */

	/**
	 * @brief A factory for a RWPERollbackMethod. This factory also defines an ExtensionPoint.
	 *
	 * By default the factory provides the following RWPERollbackMethod type:
	 *  - Ridder - RWPERollbackMethodRidder
	 */
	class Factory: public rw::common::ExtensionPoint<RWPERollbackMethod> {
	public:
		/**
		 * @brief Get the available rollback methods.
		 * @return a vector of identifiers for rollback methods.
		 */
		static std::vector<std::string> getMethods();

		/**
		 * @brief Check if rollback method is available.
		 * @param method [in] the name of the method.
		 * @return true if available, false otherwise.
		 */
		static bool hasMethod(const std::string& method);

		/**
		 * @brief Create a new rollback method.
		 * @param method [in] the name of the method.
		 * @return a pointer to a new RWPERollbackMethod.
		 */
		static RWPERollbackMethod::Ptr makeMethod(const std::string& method);

	private:
		Factory();
	};
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEROLLBACKMETHOD_HPP_ */
