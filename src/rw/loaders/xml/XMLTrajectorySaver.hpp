#ifndef XMLTRAJECTORYSAVER_HPP_
#define XMLTRAJECTORYSAVER_HPP_

#include <rw/trajectory/Trajectory.hpp>

#include <string>


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
class XMLTrajectorySaver
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
private:
    XMLTrajectorySaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif /* XMLTRAJECTORYSAVER_HPP_ */
