/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TestEnvironment.hpp"

#include <rw/RobWork.hpp>
#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>

#include <boost/filesystem.hpp>

using rw::RobWork;
using rw::common::DOMElem;
using rw::common::DOMParser;

TestEnvironment::TestEnvironment():
	_argc(0), _argv(NULL)
{
}

TestEnvironment::~TestEnvironment() {
}

void TestEnvironment::SetUp() {
    RobWork::init(_argc,(const char**)_argv);
    testfilesDir();
}

void TestEnvironment::init(int argc, char** argv) {
	_argc = argc;
	_argv = argv;
}

std::string TestEnvironment::executableDir() {
	static std::string executableDir;
	if (executableDir.empty()) {
	    boost::filesystem::path path(get()->_argv[0]);
		boost::filesystem::path full_path = boost::filesystem::canonical(path);
	    executableDir = full_path.parent_path().string()+"/";
	}
	return executableDir;
}

std::string TestEnvironment::testfilesDir() {
	static std::string testfilesDir;
	if (testfilesDir.empty()) {
	    const DOMParser::Ptr parser = DOMParser::make();
	    parser->load(executableDir() + "/TestSuiteConfig.xml");
	    const DOMElem::Ptr testElem = parser->getRootElement()->getChild("RobWorkStudioTest",false);
	    const DOMElem::Ptr dirElem = testElem->getChild("TestfilesDIR",false);
	    testfilesDir = dirElem->getValue()+"/";
	}
	return testfilesDir;
}

TestEnvironment* TestEnvironment::get() {
	static TestEnvironment* env = new TestEnvironment();
	return env;
}

TestEnvironment* TestEnvironment::make(int argc, char** argv) {
	TestEnvironment* const env = get();
	env->init(argc,argv);
	return env;
}
