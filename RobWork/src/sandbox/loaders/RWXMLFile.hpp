/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#ifndef RWXMLFILE_HPP_
#define RWXMLFILE_HPP_

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

/**
 * @brief define methods for loading and saving workcells from and to the xml
 * file format.
 *
 */

class RWXMLFile {
public:
	/**
	 * @brief save a workcell with some intial states "initState" to a file with name
	 * "filename".
	 * @param wc [in] pointer to a workcell
	 * @param initState [in] state that the workcell should initially be in
	 * @param filename [in] name of file to save workcell to
	 */
	static void saveWorkCell(rw::models::WorkCell &wc,
							 const rw::kinematics::State& initState,
							 const std::string& filename);


};

#endif /*RWXMLFILE_HPP_*/
