#include "IKMetaSolver.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>

using namespace rw::invkin;

using namespace rw::math;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::pathplanning;


IKMetaSolver::IKMetaSolver(IterativeIKPtr iksolver,
                           const DevicePtr device,
                           CollisionDetectorPtr collisionDetector) :
    _iksolver(iksolver),
    _collisionDetector(collisionDetector),
    _constraint(NULL),
    _device(device)
{
    initialize();
}

IKMetaSolver::IKMetaSolver(IterativeIKPtr iksolver,
                           const rw::models::DevicePtr device,
                           rw::pathplanning::QConstraintPtr constraint):
   _iksolver(iksolver),
   _collisionDetector(NULL),
   _constraint(constraint),
   _device(device)
{
    initialize();
}


void IKMetaSolver::initialize() {
    _proximityLimit = 1e-5;
    _bounds = _device->getBounds();
    _dof = _device->getDOF();
}

IKMetaSolver::~IKMetaSolver() {}

bool IKMetaSolver::betweenLimits(const Q& q) const
{
    for (size_t i = 0; i<q.size(); i++) {
        if (q(i) < _bounds.first(i) || _bounds.second(i) < q(i))
            return false;
    }
    return true;
}

Q IKMetaSolver::getRandomConfig() const
{
    Q q(_dof);
    for (int i = 0; i < (int)_dof; i++) {
        q(i) = Math::ran(_bounds.first(i), _bounds.second(i));
    }
    return q;
}

/*
 * Check for doublets an only add if the new solution if different from previous
 */
void IKMetaSolver::addSolution(const rw::math::Q& q, std::vector<Q>& res) const {
    if (_proximityLimit<=0) {
        res.push_back(q);
        return;
    }

    for (std::vector<Q>::iterator it = res.begin(); it != res.end(); ++it) {
        double d = MetricUtil::distInf(q, *it);
        if (d <= _proximityLimit) //If different is less than the threshold we don't wish to add it
            return;
    }
    res.push_back(q);
}

std::vector<Q> IKMetaSolver::solve(const Transform3D<>& baseTend,
                                   const State& stateDefault,
                                   size_t cnt,
                                   bool stopatfirst) const
{
    if (_constraint == NULL && _collisionDetector != NULL) {
        _constraint = QConstraint::make(_collisionDetector, _device, stateDefault);
    }

    State state(stateDefault);
    std::vector<Q> result;
    while (cnt > 0) {
        std::vector<Q> solutions = _iksolver->solve(baseTend, state);
        _device->setQ(getRandomConfig(), state);
        for (std::vector<Q>::iterator it = solutions.begin();
             it != solutions.end();
             ++it)
        {
            if (betweenLimits(*it)) {
                if (_constraint != NULL) {
                    if (_constraint->inCollision(*it))
                        continue;
                }
                addSolution(*it, result);
                //result.push_back(*it);
                if (stopatfirst) {
                    return result;
                }
            }
        }
        cnt--;
    }
    return result;
}

std::vector<Q> IKMetaSolver::solve(const Transform3D<>& baseTend,
                                   const State& defaultState) const
{
    return solve(baseTend, defaultState, _maxAttempts, _stopAtFirst);
}

void IKMetaSolver::setMaxAttempts(size_t maxAttempts)
{
    _maxAttempts = maxAttempts;
}

void IKMetaSolver::setStopAtFirst(bool stopAtFirst)
{
    _stopAtFirst = stopAtFirst;
}

void IKMetaSolver::setProximityLimit(double limit) {
    _proximityLimit = limit;
}
