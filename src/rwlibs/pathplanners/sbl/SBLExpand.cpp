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

#include "SBLExpand.hpp"
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Constants.hpp>
#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <boost/foreach.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;

namespace
{
    Q expandUniform(
        const Q& q,
        const SBLExpand::QBox& outer,
        const SBLExpand::QBox& inner,
        double scale)
    {
        Q result(q.size());
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            const double lower =
                std::max(
                    outer.first[i],
                    q[i] + scale * inner.first[i]);

            const double upper =
                std::min(
                    outer.second[i],
                    q[i] + scale * inner.second[i]);

            if (lower <= upper) {
                result[i] = Math::ran(lower, upper);
            }
            else
                return Q();
        }

        return result;
    }

    SBLExpand::QBox makeInner(const SBLExpand::QBox& outer, double ratio)
    {
        const Q radius = (0.5 * ratio) * (outer.second - outer.first);
        return SBLExpand::QBox(-radius, radius);
    }

    // Sigh.
    Vector3D<> getLinear(const Jacobian& jac, int row, int col)
    {
        return Vector3D<>(jac(row + 0, col), jac(row + 1, col), jac(row + 2, col));
    }

    Vector3D<> getAngular(const Jacobian& jac, int row, int col)
    {
        return Vector3D<>(jac(row + 3, col), jac(row + 4, col), jac(row + 5, col));
    }

    Q radiusForJacobian(
        const Jacobian& jac,
        double angle_max,
        double disp_max,
        double min_angle_vel,
        double min_disp_vel)
    {
        // A metric other than the max norm could be appropiate here.

        Q minRadius(jac.size2());
        for (size_t i = 0; i < minRadius.size(); i++) minRadius[i] = -1;

        for (size_t row = 0; row < jac.size1(); row += 6) {
            for (size_t col = 0; col < jac.size2(); col++) {

                const double raw_angle_vel = getAngular(jac, row, col).norm2();
                const double raw_disp_vel = getLinear(jac, row, col).norm2();

                const double angle_vel = std::max(min_angle_vel, raw_angle_vel);
                const double disp_vel = std::max(min_disp_vel, raw_disp_vel);

                const double value =
                    std::min(angle_max / angle_vel, disp_max / disp_vel);

                double& minValue = minRadius[col];
                if (minValue < 0 || value < minValue) minValue = value;
            }
        }
        return minRadius;
    }

    double estimateDiameter(
        const Device& device,
        const State& state,
        int maxCnt)
    {
        QSamplerPtr sampler = QSampler::makeUniform(device.getBounds());
        std::vector<Q> qs;
        for (int cnt = 0; cnt < maxCnt; ++cnt) {
            qs.push_back(sampler->sample());
        }

        double maxDist = -1;
        BOOST_FOREACH(const Q& q1, qs) {
            BOOST_FOREACH(const Q& q2, qs) {
                State s1 = state;
                device.setQ(q1, s1);

                State s2 = state;
                device.setQ(q2, s2);

                const double dist =
                    MetricUtil::dist2(
                        Kinematics::worldTframe(device.getEnd(), s1).P(),
                        Kinematics::worldTframe(device.getEnd(), s2).P());

                if (maxDist < 0 || dist > maxDist) maxDist = dist;
            }
        }
        return maxDist;
    }

    class UniformBox : public SBLExpand
    {
    public:
        UniformBox(
            const QBox& outer,
            const QBox& inner)
            :
            _outer(outer),
            _inner(inner)
        {}

    private:
        rw::math::Q doExpand(const rw::math::Q& q)
        {
            return expandUniform(q, _outer, _inner, 1);
        }

    private:
        QBox _outer;
        QBox _inner;
    };

    class ShrinkingUniformBox : public SBLExpand
    {
    public:
        ShrinkingUniformBox(
            QConstraintPtr constraint,
            const QBox& outer,
            const QBox& inner)
            :
            _constraint(constraint),
            _outer(outer),
            _inner(inner)
        {}

    private:
        rw::math::Q doExpand(const rw::math::Q& q)
        {
            for (double denom = 1;; ++denom) {
                const double scale = 1 / denom;
                const Q qn = expandUniform(q, _outer, _inner, scale);
                if (qn.empty() || !_constraint->inCollision(qn))
                    return qn;
            }
            return Q();
        }

    private:
        QConstraintPtr _constraint;
        QBox _outer;
        QBox _inner;
    };

    class JacobianShrinkingUniformBox : public SBLExpand
    {
    public:
        JacobianShrinkingUniformBox(
            QConstraintPtr constraint,
            const QBox& outer,
            JacobianCalculatorPtr jacobian,
            DevicePtr device,
            const State& state,
            double angle_max,
            double disp_max)
            :
            _constraint(constraint),
            _outer(outer),
            _jacobian(jacobian),
            _device(device),
            _state(state),
            _angle_max(angle_max),
            _disp_max(disp_max)
        {}

    private:
        rw::math::Q doExpand(const rw::math::Q& q)
        {
            State state = _state;
            _device->setQ(q, state);
            const Jacobian jac = _jacobian->get(state);

            const Q radius = radiusForJacobian(
                jac,
                _angle_max,
                _disp_max,
                0.1 * Deg2Rad, // A small minimum angular velocity
                1e-4); // A small minimum positional velocity.

            const QBox inner(-radius, radius);
            for (double denom = 1;; ++denom) {
                const double scale = 1 / denom;
                const Q qn = expandUniform(q, _outer, inner, scale);
                if (qn.empty() || !_constraint->inCollision(qn))
                    return qn;
            }
            return Q();
        }

    private:
        QConstraintPtr _constraint;
        QBox _outer;
        JacobianCalculatorPtr _jacobian;
        DevicePtr _device;
        State _state;
        double _angle_max;
        double _disp_max;
    };
}

SBLExpandPtr SBLExpand::makeUniformBox(
    const QBox& outer,
    const QBox& inner)
{
    return ownedPtr(new UniformBox(outer, inner));
}

SBLExpandPtr SBLExpand::makeUniformBox(
    const QBox& outer,
    double ratio)
{
    RW_ASSERT(ratio > 0);
    return makeUniformBox(outer, makeInner(outer, ratio));
}

SBLExpandPtr SBLExpand::makeShrinkingUniformBox(
    QConstraintPtr constraint,
    const QBox& outer,
    const QBox& inner)
{
    return ownedPtr(new ShrinkingUniformBox(constraint, outer, inner));
}

SBLExpandPtr SBLExpand::makeShrinkingUniformBox(
    QConstraintPtr constraint,
    const QBox& outer,
    double ratio)
{
    return makeShrinkingUniformBox(constraint, outer, makeInner(outer, ratio));
}

SBLExpandPtr SBLExpand::makeShrinkingUniformJacobianBox(QConstraintPtr constraint,
                                                        DevicePtr device,
                                                        const State& state,
                                                        JacobianCalculatorPtr jacobian,
                                                        double angle_max,
                                                        double disp_max)
{
    if (!jacobian)
        jacobian = device->baseJCend(state);

    if (angle_max < 0) angle_max = 45 * Deg2Rad;

    if (disp_max < 0) {
        const double diameter = estimateDiameter(*device, state, 200);
        std::cout << "Diameter: " << diameter << "\n";
        disp_max = 0.10 * diameter;
    }

    return ownedPtr(
        new JacobianShrinkingUniformBox(
            constraint,
            device->getBounds(),
            jacobian,
            device,
            state,
            angle_max,
            disp_max));
}
