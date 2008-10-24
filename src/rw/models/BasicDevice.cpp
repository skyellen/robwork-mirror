/*********************************************************************
 * RobWork Version 0.3
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

#include "BasicDevice.hpp"

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

typedef BasicDevice::iterator I;
typedef BasicDevice::const_iterator CI;

namespace
{
    void warnIfNotSet(const std::vector<Joint*>& joints)
    {
        typedef std::vector<Joint*> S;
        typedef S::const_iterator I;

        S seq = joints;
        sort(seq.begin(), seq.end());

        if (seq.empty())
            RW_WARN("Empty sequence of joints for BasicDevice.");
        else {
            I q = seq.begin();
            for (I p = q++; q != seq.end(); ++p, ++q) {
                if (*p == *q)
                    RW_WARN(
                        "Duplicate joint "
                        << (**p).getName()
                        << " for BasicDevice.");
            }
        }
    }
}

BasicDevice::BasicDevice(const std::vector<Joint*>& joints) :
    _joints(joints)
{
    // A little sanity checking goes a long way.
    warnIfNotSet(joints);
}

void BasicDevice::setQ(const Q& q, State& state) const
{
    if (q.size() != size())
        RW_THROW(
            "setQ() called for device of size "
            << (int)size()
            << " and q of size "
            << (int)q.size());

    int i = 0;
    for (CI p = begin(); p != end(); ++p, ++i)
        p->setQ(state, &q[i]);
}

Q BasicDevice::getQ(const State& state) const
{
    Q q(size());

    int i = 0;
    for (CI p = begin(); p != end(); ++p, ++i)
        q[i] = p->getQ(state)[0];

    return q;
}

std::pair<Q, Q> BasicDevice::getBounds() const
{
    const Q q(size());
    std::pair<Q, Q> bounds(q, q);
    int i = 0;
    for (CI p = begin(); p != end(); ++p, ++i) {
        const std::pair<double, double> pair = p->getBounds();
        bounds.first[i] = pair.first;
        bounds.second[i] = pair.second;
    }

    return bounds;
}

void BasicDevice::setBounds(const std::pair<Q, Q>& bounds)
{
    RW_ASSERT(size() == bounds.first.size() && size() == bounds.second.size());

    int i = 0;
    for (I p = begin(); p != end(); ++p, ++i) {
        p->setBounds(std::make_pair(bounds.first[i], bounds.second[i]));
    }
}

Q BasicDevice::getVelocityLimits() const
{
    typedef Q::const_iterator QI;

    Q limits(size());

    int i = 0;
    for (CI p = begin(); p != end(); ++p, ++i)
        limits[i] = p->getMaxVelocity();

    return limits;
}

void BasicDevice::setVelocityLimits(const Q& vellimits)
{
    RW_ASSERT(size() == vellimits.size());

    int i = 0;
    for (I p = begin(); p != end(); ++p, ++i)
        p->setMaxVelocity(vellimits[i]);
}

Q BasicDevice::getAccelerationLimits() const
{
    Q limits(size());

    int i = 0;
    for (CI p = begin(); p != end(); ++p, ++i)
        limits[i] = p->getMaxAcceleration();

    return limits;
}

void BasicDevice::setAccelerationLimits(const Q& q)
{
    RW_ASSERT(size() == q.size());

    int i = 0;
    for (I p = begin(); p != end(); ++p, ++i)
        p->setMaxAcceleration(q[i]);
}
