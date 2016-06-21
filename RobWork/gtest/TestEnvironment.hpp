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

#ifndef ROBWORK_GTEST_TESTENVIRONMENT_HPP_
#define ROBWORK_GTEST_TESTENVIRONMENT_HPP_

/**
 * @file TestEnvironment.hpp
 *
 * \copydoc ::TestEnvironment
 */

#include <gtest/gtest.h>

/**
 * @brief Utility class for setting up the test environment.
 */
class TestEnvironment: public ::testing::Environment {
public:
	//! @brief Sets up this Google Test environment.
	void SetUp();

	/**
	 * @brief Initialize the environment with command-line arguments.
	 * @param argc [in] the number of arguments.
	 * @param argv [in] the arguments.
	 */
	void init(int argc, char** argv);

	/**
	 * @brief Get the test-files directory.
	 * @return the path.
	 */
	static std::string testfilesDir();

	/**
	 * @brief Get the environment object.
	 * @return pointer to this environment.
	 */
	static TestEnvironment* get();

	/**
	 * @brief Create the environment object, initialized with arguments.
	 * @param argc [in] the number of arguments.
	 * @param argv [in] the arguments.
	 * @return pointer to this environment.
	 */
	static TestEnvironment* make(int argc, char** argv);

private:
	TestEnvironment();
	virtual ~TestEnvironment();

	int _argc;
	char** _argv;
};

#endif /* ROBWORK_GTEST_TESTENVIRONMENT_HPP_ */
