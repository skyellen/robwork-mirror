/*
 * ODETactileSensor.hpp
 *
 *  Created on: 13-09-2009
 *      Author: jimali
 */

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <sensors/SimulatedTactileSensor.hpp>

#include <vector>
#include <ode/ode.h>

#ifndef ODETACTILESENSOR_HPP_
#define ODETACTILESENSOR_HPP_

class ODETactileSensor {
public:
    ODETactileSensor(SimulatedTactileSensor *sens);

    virtual ~ODETactileSensor(){};

    void addFeedback(const std::vector<dJointFeedback*>& fback, const std::vector<dContactGeom> &g, dynamics::Body* b, int body);

    void clear();

    void update(double dt, rw::kinematics::State& state);

//    void setContacts(const rw::proximity::MultiDistanceResult& res,
//                     rw::math::Transform3D<> wTa,
//                     rw::math::Transform3D<> wTb);

private:
    std::vector<rw::math::Transform3D<> > _wTa,_wTb;

    std::vector<std::vector<dJointFeedback*> > _feedback;
    std::vector<std::vector<dContactGeom> > _geoms;
    //std::vector<rw::proximity::MultiDistanceResult> _contacts;

    std::vector<dynamics::Body*> _rwBody;
    SimulatedTactileSensor *_rwsensor;
    //rw::math::Vector3D<> point;
    std::vector<int> _bodyIdx;

};


#endif /* ODETACTILESENSOR_HPP_ */
