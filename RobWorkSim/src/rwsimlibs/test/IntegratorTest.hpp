/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TEST_INTEGRATORTEST_HPP_
#define RWSIMLIBS_TEST_INTEGRATORTEST_HPP_

/**
 * @file IntegratorTest.hpp
 *
 * \copydoc rwsimlibs::test::IntegratorTest
 */

#include "EngineTest.hpp"

namespace rwsimlibs {
namespace test {
//! @addtogroup rwsimlibs_test

//! @{
/**
 * @brief Common parent class for all integrator tests.
 *
 * The class defines supported engines, common parameters, and a standard dynamic workcell.
 */
class IntegratorTest: public EngineTest {
public:
	//! @brief Constructor.
	IntegratorTest();

	//! @brief Destructor.
	virtual ~IntegratorTest();

	//! @copydoc EngineTest::isEngineSupported
	virtual bool isEngineSupported(const std::string& engineID) const;

	//! @copydoc EngineTest::getDWC
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> getDWC(const rw::common::PropertyMap& map);

	//! @copydoc EngineTest::getDefaultParameters
	virtual rw::common::Ptr<rw::common::PropertyMap> getDefaultParameters() const;

	/**
	 * @brief Create new dynamic workcell.
	 * @param integratorType [in] (optional) the integrator to use.
	 * @return the dynamic workcell.
	 */
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> makeIntegratorDWC(const std::string& integratorType = "");

private:
	std::map<std::string,rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> > _integratorTypeToDWC;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */

#endif /* RWSIMLIBS_TEST_INTEGRATORTEST_HPP_ */
