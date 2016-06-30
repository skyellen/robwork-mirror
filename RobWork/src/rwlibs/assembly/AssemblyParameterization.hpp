/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYPARAMETERIZATION_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYPARAMETERIZATION_HPP_

/**
 * @file AssemblyParameterization.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyParameterization
 */

#include <rw/common/Ptr.hpp>
#include <rw/common/PropertyMap.hpp>

namespace rwlibs {
namespace assembly {
//! @addtogroup assembly

//! @{
/**
 * @brief Interface for a parameterization that can be used by a AssemblyControlStrategy.
 *
 * Each AssemblyControlStrategy will typically derive its own AssemblyParameterization that includes
 * the parameters necessary to specify the assembly operation when using that specific strategy.
 *
 * By deriving from this class, the parameterization can be stored and restored via a PropertyMap structure.
 * This is mainly used as a way to extend the assembly assembly specification with additional data, while maintaining
 * the ability to serialize, load and save the AssemblyTask. Derived classes should override the #clone, #make,
 * #reset and #toPropertyMap functions.
 *
 * Notice that this is also a concrete class, allowing the user to create a parameterization that is always empty,
 * or create a parameterization based on a custom PropertyMap. The latter is especially useful if scripting
 * interfaces are used, where subclassing is not possible.
 */
class AssemblyParameterization {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyParameterization> Ptr;

    //! @brief Construct an empty parameterization.
	AssemblyParameterization();

	/**
	 * @brief Construct a parameterization from a PropertyMap.
	 * @param pmap [in] a PropertyMap.
	 */
	AssemblyParameterization(rw::common::Ptr<rw::common::PropertyMap> pmap);

    //! @brief Destructor
	virtual ~AssemblyParameterization();

	/**
	 * @brief Store the parameterization in a PropertyMap.
	 * @return a pointer to a PropertyMap, or NULL if parameterization is empty.
	 */
	virtual rw::common::Ptr<rw::common::PropertyMap> toPropertyMap() const;

	/**
	 * @brief Clone the parameterization.
	 * @return a pointer to the new parameterization.
	 */
	virtual AssemblyParameterization::Ptr clone() const;

	/**
	 * @brief Construct a parameterization of same type from a PropertyMap.
	 * @param pmap [in] a PropertyMap.
	 */
	virtual AssemblyParameterization::Ptr make(rw::common::Ptr<rw::common::PropertyMap> pmap) const;

	/**
	 * @brief Reset the parameters by taking new parameters from a PropertyMap.
	 * @param pmap [in] a PropertyMap.
	 */
	virtual void reset(rw::common::Ptr<rw::common::PropertyMap> pmap);

protected:
	//! @brief The PropertyMap.
	rw::common::Ptr<rw::common::PropertyMap> _pmap;
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYPARAMETERIZATION_HPP_ */
