/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "PlannerUtil.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/Jacobian.hpp>
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::pathplanning;


PlannerUtil::PlannerUtil(Device* device, const State& state, CollisionDetector* detector):
    _device(device),
    _state(state),
    _collisionDetector(detector)
{
}


PlannerUtil::~PlannerUtil() {
}

void PlannerUtil::setState(rw::kinematics::State& state) {
    _state = state;
}


Q PlannerUtil::randomConfig() const
{
    RW_ASSERT(NULL != _device);

    Q qn(_device->getDOF());
    for (size_t i = 0; i < qn.size(); i++)
        qn[i] = Math::Ran();

    return unNormalize(qn);
}

bool PlannerUtil::inCollision(const Q& q) const
{
    RW_ASSERT(NULL != _device);
    State state = _state;
    _device->setQ(q, state);

    return _collisionDetector->inCollision(state);
}

bool PlannerUtil::inCollision(const rw::math::Q& q1, const rw::math::Q& q2, int samples) const {
    double delta = 1.0/(double)samples;
    for (int i = 1; i<samples; i++) {
        Q qnew = (1-delta*i)*q1 + (delta*i)*q2;
        if (inCollision(qnew))
            return true;
    }
    return false;
}

Q PlannerUtil::normalize(const Q& q) const
{
    RW_ASSERT(NULL != _device);

    Q qn(q.size());
    std::pair<Q, Q> bounds = _device->getBounds();
    for(size_t i = 0; i<qn.size(); i++){
        qn[i] = (q[i] - bounds.first[i]) / (bounds.second[i] - bounds.first[i]);
    }
    return qn;
}

Q PlannerUtil::unNormalize(const Q& qn) const
{
    RW_ASSERT(NULL != _device);

    Q q(qn.size());
    std::pair<Q, Q> bounds = _device->getBounds();
    for(size_t i = 0; i<q.size(); i++){
        q[i] = qn[i] * (bounds.second[i] - bounds.first[i]) + bounds.first[i];
    }
    return q;
}


Q PlannerUtil::estimateMotionWeights(Frame* frame, EstimateType type, size_t samples) const {
    RW_ASSERT(_device != NULL);
    
    if (frame == NULL)
        frame = _device->getEnd();
    
    size_t n = _device->getDOF();
    State state = _state;
    Q ws = Q(boost::numeric::ublas::zero_vector<double>(n));
    for (size_t i = 0; i<samples; i++) {
        Q q = randomConfig();
        _device->setQ(q, state);
        Jacobian jac = _device->baseJframe(frame, state);
        for (size_t j = 0; j<n; j++) {
            double dx = jac(0,j);
            double dy = jac(1,j);
            double dz = jac(2,j);
            double w = sqrt(dx*dx + dy*dy + dz*dz);
            switch (type) {
            case WORSTCASE:
                if (w > ws(j))
                    ws(j) = w;
                break;
            case AVERAGE:
                ws(j) += w;
                break;                
            }
        }
    }
    if (type == AVERAGE)
        ws /= samples;
    return ws;
}
