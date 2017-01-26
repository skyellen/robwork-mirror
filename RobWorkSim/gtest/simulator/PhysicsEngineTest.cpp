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
	BOOST_FOREACH(const std::string& name, engines) {
		if (name == "ODE")
			foundODE = true;
		else if (name == "Bullet")
			foundBullet = true;
	}
	EXPECT_TRUE(foundODE);
	EXPECT_TRUE(foundBullet);
}

struct TestParam {
	TestParam(EngineTest* test, const std::string& testName, const std::string& engine, int id, PropertyMap::Ptr parameters):
		testName(testName),
		test(test),
		engine(engine),
		id(id),
		parameters(parameters)
	{
	}
	std::string testName;
	// Notice that the test variable must be a raw pointer!
	// - it will otherwise be destructed after the main loop ends by the google test framework
	// - this might cause segmentation fault if the plugin that provided the test has already been unloaded
	EngineTest* test;
	std::string engine;
	int id;
	PropertyMap::Ptr parameters;
};

class PhysicsEngineTest : public ::testing::TestWithParam<TestParam> {
protected:
	PhysicsEngineTest(): handle(ownedPtr(new EngineTest::TestHandle())) {
	}

	const EngineTest::TestHandle::Ptr handle;
};

::std::ostream& operator<<(::std::ostream& os, const TestParam& combo) {
	if (combo.id < 0)
		os << "(" << combo.testName << ", " << combo.engine << ") default";
	else
		os << "(" << combo.testName << ", " << combo.engine << ") predefined #" << combo.id;
    return os;
}

std::list<TestParam> defaultTests;
std::list<TestParam> predefinedTests;
INSTANTIATE_TEST_CASE_P(DefaultParameter, PhysicsEngineTest, ::testing::ValuesIn(defaultTests));
INSTANTIATE_TEST_CASE_P(PredefinedParameters, PhysicsEngineTest, ::testing::ValuesIn(predefinedTests));

TEST_P(PhysicsEngineTest, TestEngineParameterTest) {
	ASSERT_TRUE(GetParam().test != NULL);
	GetParam().test->run(handle, GetParam().engine, *GetParam().parameters);
	EXPECT_EQ(handle->getError(),"");
	BOOST_FOREACH(const EngineTest::Result& result, handle->getResults()) {
		BOOST_FOREACH(const EngineTest::Failure& failure, result.failures) {
			ADD_FAILURE() << result.name << " (" << result.description << ") at time " << failure.time << ": " << failure.description;
		}
	}
}

int main(int argc, char **argv) {
	std::vector<EngineTest::Ptr> ownedTests; // we need to control the lifetime in the main loop!

	// Construct list of engine-test pairs to test
	const std::vector<std::string> engines = PhysicsEngine::Factory::getEngineIDs();
	BOOST_FOREACH(const std::string& testName, EngineTest::Factory::getTests()) {
		ownedTests.push_back(EngineTest::Factory::getTest(testName));
		EngineTest* const etest = ownedTests.back().get();
		BOOST_FOREACH(const std::string& engineName, engines) {
			if (etest->isEngineSupported(engineName)) {
				const PropertyMap::Ptr def = etest->getDefaultParameters();
				defaultTests.push_back(TestParam(etest,testName,engineName,-1,def));
				const std::vector<PropertyMap::Ptr> predefined = etest->getPredefinedParameters();
				for (std::size_t i = 0; i < predefined.size(); i++) {
					predefinedTests.push_back(TestParam(etest,testName,engineName,static_cast<int>(i),predefined[i]));
				}
			}
		}
	}
	testing::InitGoogleTest(&argc, argv);
	const int res = RUN_ALL_TESTS();
	ownedTests.clear(); // important! - we must destruct the EngineTest instances explicitly before the main loop ends
	return res;
}
