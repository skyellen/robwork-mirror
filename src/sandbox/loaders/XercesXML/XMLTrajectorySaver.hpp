/*
 * XMLTrajectorySaver.hpp
 *
 *  Created on: Nov 28, 2008
 *      Author: lpe
 */

#ifndef XMLTRAJECTORYSAVER_HPP_
#define XMLTRAJECTORYSAVER_HPP_

#include <rw/trajectory/Trajectory.hpp>

#include <string>

class XMLTrajectorySaver
{
public:

    virtual ~XMLTrajectorySaver() {};


    static bool save(const rw::trajectory::QTrajectory& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Vector3DTrajectory& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Rotation3DTrajectory& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Transform3DTrajectory& trajectory, const std::string& filename);
private:
    XMLTrajectorySaver() {};
};

#endif /* XMLTRAJECTORYSAVER_HPP_ */
