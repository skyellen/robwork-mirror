#include "IKMetaSolver.hpp"

#include <rw/math/Math.hpp>

using namespace rw::invkin;

using namespace rw::math;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;

IKMetaSolver::IKMetaSolver(IterativeIK* iksolver,
                           DeviceModel* device,
                           CollisionDetector* collisionDetector):
   _iksolver(iksolver),
   _collisionDetector(collisionDetector),
   _device(device)
{
    _bounds = device->getBounds();
    _dof = _device->getDOF();

}

IKMetaSolver::~IKMetaSolver()
{
}

bool IKMetaSolver::betweenLimits(const Q& q) const {
    for (size_t i = 0; i<q.size(); i++) {
        if (q(i)<_bounds.first(i) || q(i) > _bounds.second(i) ) {
            return false;
        }
    }
    return true;
}

Q IKMetaSolver::getRandomConfig() const {
    Q q(_dof);
    for (int i = 0; i<_dof; i++) {
        q(i) = Math::Ran(_bounds.first(i), _bounds.second(i));
    }
    return q;
}

std::vector<Q> IKMetaSolver::solve(const Transform3D<>& baseTend,
                                   const State& stateDefault,
                                   size_t cnt,
                                   bool stopatfirst) const {
    State state(stateDefault);
    std::vector<Q> result;
    while (cnt>0) {
        _device->setQ(getRandomConfig(), state);
        std::vector<Q> solutions = _iksolver->solve(baseTend, state);

        for (std::vector<Q>::iterator it = solutions.begin(); it != solutions.end(); ++it) {
            if (betweenLimits(*it)) {
                if (_collisionDetector != NULL) {
                    _device->setQ(*it, state);
                    if (_collisionDetector->inCollision(state))
                        continue;
                }
                result.push_back(*it);
                if (stopatfirst) {
                    return result;
                }
            }
        }
        cnt--;
    }
    return result;
}

std::vector<Q> IKMetaSolver::solve(const Transform3D<>& baseTend, const State& defaultState) const {
    return solve(baseTend, defaultState, _maxAttempts, _stopAtFirst);
}

void IKMetaSolver::setMaxAttempts(size_t maxAttempts) {
    _maxAttempts = maxAttempts;
}

void IKMetaSolver::setStopAtFirst(bool stopAtFirst) {
    _stopAtFirst = stopAtFirst;
}
