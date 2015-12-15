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

#ifndef RWSIMLIBS_RWPE_RWPERESTITUTIONMODEL_HPP_
#define RWSIMLIBS_RWPE_RWPERESTITUTIONMODEL_HPP_

/**
 * @file RWPERestitutionModel.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPERestitutionModel
 */

#include <rw/common/ExtensionPoint.hpp>

// Forward declarations
namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEContact;
class RWPEIslandState;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief A restitution model calculates restitution coefficients based on the relative incomming velocity.
 */
class RWPERestitutionModel {
public:
    //! @brief The possible restitution values.
    struct Values {
    	//! @brief Default constructor - only restitution in normal direction enabled with coefficients set to zero.
    	Values(): normal(0), enableTangent(false), tangent(0), enableAngular(false), angular(0) {};
    	//! @brief Linear restitution coefficient in the normal direction.
    	double normal;
    	//! @brief Solve for tangential restitution.
    	bool enableTangent;
    	//! @brief Linear restitution coefficient in the motion direction.
    	double tangent;
    	//! @brief Solve for angular restitution.
    	bool enableAngular;
    	//! @brief Angular restitution coefficient around the normal direction.
    	double angular;
    };

    //! @brief Constructor.
	RWPERestitutionModel() {};

    //! @brief Destructor.
	virtual ~RWPERestitutionModel() {};

	/**
	 * @brief Get a model with the given properties.
	 * @param map [in] a map of properties with the required parameters for the model.
	 * @return a restitution model with the given properties (can return NULL if properties are not as expected).
	 */
	virtual const RWPERestitutionModel* withProperties(const rw::common::PropertyMap &map) const = 0;

	/**
	 * @brief Get the restitution coefficients.
	 * @param contact [in] the contact to find coefficient for.
	 * @param islandState [in] the current state of the system.
	 * @param rwstate [in] the current state of the system.
	 * @return the Values.
	 */
	virtual Values getRestitution(const RWPEContact& contact, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate) const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::rwpe::RWPERestitutionModel::Factory,rwsimlibs::rwpe::RWPERestitutionModel,rwsimlibs.rwpe.RWPERestitutionModel}
	 */

	/**
	 * @brief A factory for a RWPERestitutionModel. This factory also defines an ExtensionPoint.
	 *
	 * By default the factory provides the following RWPERestitutionModel type:
	 *  - Newton - RWPERestitutionModelNewton
	 */
	class Factory: public rw::common::ExtensionPoint<RWPERestitutionModel> {
	public:
		/**
		 * @brief Get the available models.
		 * @return a vector of identifiers for models.
		 */
		static std::vector<std::string> getModels();

		/**
		 * @brief Check if model is available.
		 * @param model [in] the name of the model.
		 * @return true if available, false otherwise.
		 */
		static bool hasModel(const std::string& model);

		/**
		 * @brief Create a new model.
		 * @param model [in] the name of the model.
		 * @param properties [in] a map of properties with the required parameters for the given model.
		 * @return a pointer to a new RWPERestitutionModel owned by the caller (can return NULL if properties are not as expected).
		 */
		static const RWPERestitutionModel* makeModel(const std::string& model, const rw::common::PropertyMap &properties);

	private:
		Factory();
	};
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPERESTITUTIONMODEL_HPP_ */
