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

#ifndef RWSIMLIBS_TEST_INTEGRATORROTATIONTEST_HPP_
#define RWSIMLIBS_TEST_INTEGRATORROTATIONTEST_HPP_

/**
 * @file IntegratorRotationTest.hpp
 *
 * \copydoc rwsimlibs::test::IntegratorRotationTest
 */

#include "IntegratorTest.hpp"

namespace rwsimlibs {
namespace test {
//! @addtogroup rwsimlibs_test

//! @{
/**
 * @brief Test for rotational motion.
 *
 * According to Euler's equations of motion, the rotation around a non-inertial axis of a
 * body will cause a non-linear term in the equations of motion. The simulation is illustrated below:
 *
 * \image html tests/integrationRotation.gif "Motion of rotating body with no gravity."
 *
 * An analytical solution is NOT given for this type of motion. In this test the focus is on preservation of energy.
 */
class IntegratorRotationTest: public IntegratorTest {
public:
	//! @brief Smart pointer to IntegratorRotationTest.
	typedef rw::common::Ptr<IntegratorRotationTest> Ptr;

	//! @brief Constructor.
	IntegratorRotationTest();

	//! @brief Destructor.
	virtual ~IntegratorRotationTest();

	//! @copydoc EngineTest::run
	virtual void run(TestHandle::Ptr handle, const std::string& engineID, const rw::common::PropertyMap& parameters, rw::common::Ptr<rwsim::log::SimulatorLogScope> verbose = NULL);

	//! @copydoc EngineTest::getRunTime
	virtual double getRunTime() const;

	//! @copydoc IntegratorTest::makeIntegratorDWC
	virtual rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> makeIntegratorDWC(const std::string& integratorType = "");

	/**
	 * @brief Get the expected kinetic energy.
	 * @param dwc [in] the dynamic workcell.
	 * @return the energy.
	 */
	static double getExpectedEnergy(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

private:
	static void initialize(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc, rw::kinematics::State& state);
	static void updateResults(const EngineLoopInfo& info);
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_INTEGRATORROTATIONTEST_HPP_ */
