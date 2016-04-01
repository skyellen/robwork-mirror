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

#include <gtest/gtest.h>

#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsimlibs/test/EngineTest.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwsim::simulator;
using namespace rwsimlibs::test;

TEST(PhysicsEngineTest, EnginesInFactory) {
	const std::vector<std::string> engines = PhysicsEngine::Factory::getEngineIDs();
	bool foundODE = false;
	bool foundBullet = false;
	bool foundRWPE = false;
	BOOST_FOREACH(const std::string& name, engines) {
		if (name == "ODE")
			foundODE = true;
		else if (name == "Bullet")
			foundBullet = true;
		else if (name == "RWPEIsland")
			foundRWPE = true;
	}
	EXPECT_TRUE(foundODE);
	EXPECT_TRUE(foundBullet);
	EXPECT_TRUE(foundRWPE);
}

struct TestParam {
	TestParam(const std::string& test, const std::string& engine):
		test(test),
		engine(engine)
	{
	}
	std::string test;
	std::string engine;
};

class PhysicsEngineTest : public ::testing::TestWithParam<TestParam> {
protected:
	PhysicsEngineTest(): handle(ownedPtr(new EngineTest::TestHandle())) {
	}

	virtual void SetUp() {
		etest = EngineTest::Factory::getTest(GetParam().test);
		if (!etest.isNull())
			pars = etest->getDefaultParameters();
	}

	const EngineTest::TestHandle::Ptr handle;
	EngineTest::Ptr etest;
	PropertyMap::Ptr pars;
};

::std::ostream& operator<<(::std::ostream& os, const TestParam& combo) {
    os << "(" << combo.test << ", " << combo.engine << ")";
    return os;
}

std::list<TestParam> tests;
INSTANTIATE_TEST_CASE_P(TestEnginePair, PhysicsEngineTest, ::testing::ValuesIn(tests));

TEST_P(PhysicsEngineTest, RunDefaultTest) {
	ASSERT_TRUE(!etest.isNull());
	etest->run(handle, GetParam().engine, *pars);
	EXPECT_EQ(handle->getError(),"");
	BOOST_FOREACH(const EngineTest::Result& result, handle->getResults()) {
		BOOST_FOREACH(const EngineTest::Failure& failure, result.failures) {
			ADD_FAILURE() << result.name << " (" << result.description << ") at time " << failure.time << ": " << failure.description;
		}
	}
}

int main(int argc, char **argv) {
	// Construct list of engine-test pairs to test
	const std::vector<std::string> engines = PhysicsEngine::Factory::getEngineIDs();
	BOOST_FOREACH(const std::string& testName, EngineTest::Factory::getTests()) {
		const EngineTest::Ptr etest = EngineTest::Factory::getTest(testName);
		BOOST_FOREACH(const std::string& engineName, engines) {
			if (etest->isEngineSupported(engineName)) {
				tests.push_back(TestParam(testName,engineName));
			}
		}
	}
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
