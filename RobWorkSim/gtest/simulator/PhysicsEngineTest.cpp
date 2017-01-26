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

#include <rw/common/BINArchive.hpp>
#include <rw/common/INIArchive.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/log/LogContactSet.hpp>
#include <rwsim/log/LogEquationSystem.hpp>
using rwsim::contacts::Contact;
using rwsim::log::SimulatorLogScope;
using rwsim::log::LogContactSet;
using rwsim::log::LogEquationSystem;
TEST_P(PhysicsEngineTest, TestEngineParameterTest) {
	ASSERT_TRUE(GetParam().test != NULL);
	const SimulatorLogScope::Ptr log = ownedPtr(new SimulatorLogScope());
	GetParam().test->run(handle, GetParam().engine, *GetParam().parameters, log);
	EXPECT_EQ(handle->getError(),"");
	BOOST_FOREACH(const EngineTest::Result& result, handle->getResults()) {
		BOOST_FOREACH(const EngineTest::Failure& failure, result.failures) {
			ADD_FAILURE() << result.name << " (" << result.description << ") at time " << failure.time << ": " << failure.description;
		}
	}
	if (!log.isNull()) {
		std::stringstream file;
		file << "simlog_" << GetParam().testName << "_" << GetParam().engine << ".ini";
		std::ofstream fstream(file.str().c_str());
		fstream << std::setprecision(17) << std::endl;
		INIArchive archive(fstream);
		//archive.open(file.str());
		log->write(archive,"");
		archive.close();
		fstream.close();
	}/*
	if (!log.isNull()) {
		std::stringstream file;
		file << "matrices_" << GetParam().testName << "_" << GetParam().engine << ".m";
		std::ofstream fstream(file.str().c_str());
		fstream << std::setprecision(15) << std::endl;

		for (std::size_t stepI = 0; stepI < log->getChild(0)->children(); stepI++) {
			const SimulatorLogScope::Ptr step = log->getChild(0).cast<SimulatorLogScope>()->getChild(stepI).cast<SimulatorLogScope>();
			SimulatorLogScope::Ptr constraintSolver = NULL;
			for (std::size_t j = 0; j < step->children(); j++) {
				if (step->getChild(j)->getDescription() == "Contact & Constraint Resolver") {
					constraintSolver = step->getChild(j).cast<SimulatorLogScope>();
					break;
				}
			}
			if (constraintSolver.isNull())
				continue;
			LogEquationSystem::Ptr eqSystem = NULL;
			std::size_t j;
			for (j = 0; j < constraintSolver->children(); j++) {
				if (constraintSolver->getChild(j)->getDescription() == "Equation System") {
					eqSystem = constraintSolver->getChild(j).cast<LogEquationSystem>();
					break;
				}
			}
			if (eqSystem.isNull())
				RW_THROW("Could not find equation system for step number " << stepI << ".");

			LogContactSet::Ptr contactSet = NULL;
			for (; j < constraintSolver->children(); j++) {
				if (constraintSolver->getChild(j)->getDescription() == "Contact Positions") {
					contactSet = constraintSolver->getChild(j).cast<LogContactSet>();
					break;
				}
			}
			if (contactSet.isNull())
				RW_THROW("Could not find contact set for step number " << stepI << ".");

			// Output the equation system in MATLAB format
			const Eigen::MatrixXd& A = eqSystem->A();
			const Eigen::VectorXd& x = eqSystem->x();
			const Eigen::VectorXd& b = eqSystem->b();
			fstream << "A{" << stepI+1 << "}=[";
			for (Eigen::MatrixXd::Index i = 0; i < A.rows(); i++) {
				for (Eigen::MatrixXd::Index j = 0; j < A.cols(); j++) {
					if (j == A.cols()-1)
						fstream << A(i,j);
					else
						fstream << A(i,j) << " ";
				}
				if (i != A.rows()-1)
					fstream << ";" << std::endl;
			}
			fstream << "];" << std::endl;
			fstream << "b{" << stepI+1 << "}=[";
			for (Eigen::MatrixXd::Index i = 0; i < b.rows(); i++) {
				if (i == b.rows()-1)
					fstream << b[i];
				else
					fstream << b[i] << ";";
			}
			fstream << "];" << std::endl;
			if (x.size() > 0) {
				fstream << "x{" << stepI+1 << "}=[";
				for (Eigen::MatrixXd::Index i = 0; i < x.rows(); i++) {
					if (i == x.rows()-1)
						fstream << x[i];
					else
						fstream << x[i] << ";";
				}
				fstream << "];" << std::endl;
			}

			// Output the contacts
			const std::vector<Contact>& contacts = contactSet->getContacts();
			fstream << "contacts{" << stepI+1 << "}=[";
			for (std::size_t i = 0; i < contacts.size(); i++) {
				const Contact& c = contacts[i];
				if (c.getFrameA()->getName() == "Tube") {
					fstream << c.getPointA()[0] << " " << c.getPointA()[1] << " " << c.getPointA()[2];
					fstream << " ";
					fstream << c.getNormal()[0] << " " << c.getNormal()[1] << " " << c.getNormal()[2];
					fstream << " 1";
				} else if (c.getFrameB()->getName() == "Tube") {
					fstream << c.getPointB()[0] << " " << c.getPointB()[1] << " " << c.getPointB()[2];
					fstream << " ";
					fstream << -c.getNormal()[0] << " " << -c.getNormal()[1] << " " << -c.getNormal()[2];
					fstream << " 2";
				} else {
					RW_THROW("Weird contact found: " << c);
				}
				if (i != contacts.size()-1)
					fstream << ";" << std::endl;
			}
			fstream << "];" << std::endl;
		}

		fstream.close();
	}*/
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
