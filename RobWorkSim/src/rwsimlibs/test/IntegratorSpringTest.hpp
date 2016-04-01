/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TEST_INTEGRATORSPRINGTEST_HPP_
#define RWSIMLIBS_TEST_INTEGRATORSPRINGTEST_HPP_

/**
 * @file IntegratorSpringTest.hpp
 *
 * \copydoc rwsimlibs::test::IntegratorSpringTest
 */

#include "IntegratorTest.hpp"

namespace rwsimlibs {
namespace test {
//! @addtogroup rwsimlibs_test

//! @{
/**
 * @brief Test for the motion when an undamped linear spring is used.
 *
 * The simulation is illustrated below:
 *
 * \image html tests/integrationSpring.gif "Animation of undamped spring motion."
 *
 * An analytical solution is given for this type of motion, which makes it possible to test how well the engines solve for undamped springs.
 */
class IntegratorSpringTest: public IntegratorTest {
public:
	//! @brief Constructor.
	IntegratorSpringTest();

	//! @brief Destructor.
	virtual ~IntegratorSpringTest();

	//! @copydoc EngineTest::run
	virtual void run(TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL);

	//! @copydoc EngineTest::getRunTime
	virtual double getRunTime() const;

	//! @copydoc IntegratorTest::makeIntegratorDWC
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> makeIntegratorDWC(const std::string& integratorType = "");

	/**
	 * @brief Get analytical reference position.
	 * @param t [in] the time.
	 * @return the reference position.
	 */
	static double referencePosition(double t);

	/**
	 * @brief Get analytical reference velocity.
	 * @param t [in] the time.
	 * @return the reference velocity.
	 */
	static double referenceVelocity(double t);

private:
	static void updateResults(const EngineLoopInfo& info);
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_INTEGRATORSPRINGTEST_HPP_ */
