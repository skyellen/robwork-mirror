/*
 * TrajectoryLoader.hpp
 *
 *  Created on: Nov 12, 2008
 *      Author: lpe
 */

#ifndef TRAJECTORYLOADER_HPP_
#define TRAJECTORYLOADER_HPP_


#include <rw/trajectory/Trajectory.hpp>
#include <rw/math/Q.hpp>

#include <xercesc/dom/DOMElement.hpp>
#include <string>
class TrajectoryLoader
{
public:
    TrajectoryLoader(const std::string& filename);
    virtual ~TrajectoryLoader();

    enum Type { Q_TYPE = 0, VECTOR3D_TYPE, ROTATION3D_TYPE, TRANSFORM3D_TYPE};

    Type getType(const std::string& filename);


    rw::trajectory::Trajectory<rw::math::Q> getQTrajectory();

private:
    void readTrajectory(xercesc::DOMElement* element);

    rw::trajectory::Trajectory<rw::math::Q> _qTrajectory;
};

#endif /* TRAJECTORYLOADER_HPP_ */
