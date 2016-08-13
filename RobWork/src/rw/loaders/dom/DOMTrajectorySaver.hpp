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

#ifndef RW_LOADERS_DOMTRAJECTORYSAVER_HPP_
#define RW_LOADERS_DOMTRAJECTORYSAVER_HPP_

#include <rw/trajectory/Trajectory.hpp>

#include <string>
#include <iosfwd>

namespace rw {
namespace loaders {



/** @addtogroup loaders */
/*@{*/


/**
 * @brief Class for saving trajectories to file.
 *
 * Trajectories are saved in the RobWork Trajectory XML-format specified in
 * rwxml_trajectory.xsd and matching with rw::loaders::XMLTrajectoryLoader
 */
class DOMTrajectorySaver
{
public:
    /**
     * @brief Saves the QTrajectory \b trajectory to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to save
     * @param filename [in] Target filename
     */
    static bool save(const rw::trajectory::QTrajectory& trajectory, const std::string& filename);

    /**
     * @brief Saves the Vector3DTrajectory \b trajectory to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to save
     * @param filename [in] Target filename
     */
    static bool save(const rw::trajectory::Vector3DTrajectory& trajectory, const std::string& filename);

    /**
     * @brief Saves the Rotation3DTrajectory \b trajectory to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to save
     * @param filename [in] Target filename
     */
    static bool save(const rw::trajectory::Rotation3DTrajectory& trajectory, const std::string& filename);

    /**
     * @brief Saves the Transform3DTrajectory \b trajectory to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to save
     * @param filename [in] Target filename
     */
    static bool save(const rw::trajectory::Transform3DTrajectory& trajectory, const std::string& filename);

    /**
     * @brief Writes the QTrajectory \b trajectory to stream
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to write
     * @param outstream [in] Output stream
     */
    static bool write(const rw::trajectory::QTrajectory& trajectory, std::ostream& outstream);

    /**
     * @brief Writes the Vector3DQTrajectory \b trajectory to stream
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to write
     * @param outstream [in] Output stream
     */
    static bool write(const rw::trajectory::Vector3DTrajectory& trajectory, std::ostream& outstream);

    /**
     * @brief Writes the QRotation3Drajectory \b trajectory to stream
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to write
     * @param outstream [in] Output stream
     */
    static bool write(const rw::trajectory::Rotation3DTrajectory& trajectory, std::ostream& outstream);

    /**
     * @brief Writes the Transform3DTrajectory \b trajectory to stream
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param trajectory [in] Trajectory to write
     * @param outstream [in] Output stream
     */
    static bool write(const rw::trajectory::Transform3DTrajectory& trajectory, std::ostream& outstream);

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the DOMTrajectorySaver is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if DOMTrajectorySaver is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    DOMTrajectorySaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif /* XMLTRAJECTORYSAVER_HPP_ */
