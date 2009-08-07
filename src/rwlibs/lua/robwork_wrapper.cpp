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


#include "robwork_wrapper.hpp"
#include "Output.hpp"
#include "RobWork.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Models.hpp>
#include <rw/common/StringUtil.hpp>

#include <rw/pathplanning/QIKSampler.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>

#include <iostream>
using namespace std;

#include <sstream>
#include <algorithm>

#define NS rwlibs::lua::internal
using namespace NS;

namespace
{
    string eToString(const rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}

//----------------------------------------------------------------------
// Q
//----------------------------------------------------------------------

Q::Q(int n, double* vals) : _q(n)
{
    for (int i = 0; i < n; i++)
        _q[i] = vals[i];
}

bool Q::operator==(const Q& x) { return get() == x.get(); }

std::string Q::__tostring() const { return toString(get()); }

//----------------------------------------------------------------------
// Rotation3D
//----------------------------------------------------------------------

Vector3D::Vector3D(double vals[3]) : _vector(vals[0], vals[1], vals[2])
{}

std::string Vector3D::__tostring() const { return toString(get()); }

Vector3D Vector3D::operator*(double scale) const
{
    return Vector3D(get() * scale);
}

Vector3D Vector3D::operator+(const Vector3D& other) const
{
    return Vector3D(get() + other.get());
}

Vector3D Vector3D::operator-(const Vector3D& other) const
{
    return Vector3D(get() - other.get());
}

bool Vector3D::operator==(const Vector3D& x) { return get() == x.get(); }

const rw::math::Vector3D<>& Vector3D::get() const { return _vector; }

//----------------------------------------------------------------------
// Rotation3D
//----------------------------------------------------------------------

Rotation3D::Rotation3D(double vals[9]) :
    _rotation(
        vals[0], vals[1], vals[2],
        vals[3], vals[4], vals[5],
        vals[6], vals[7], vals[8])
{}

Rotation3D Rotation3D::operator*(const Rotation3D& other) const
{
    return Rotation3D(get() * other.get());
}

Vector3D Rotation3D::operator*(const Vector3D& other) const
{
    return Vector3D(get() * other.get());
}

Rotation3D Rotation3D::inverse() const
{
    return Rotation3D(rw::math::inverse(get()));
}

std::string Rotation3D::__tostring() const { return toString(get()); }

const rw::math::Rotation3D<>& Rotation3D::get() const { return _rotation; }

//----------------------------------------------------------------------
// Transform3D
//----------------------------------------------------------------------

Transform3D::Transform3D(
    const Vector3D& position,
    const Rotation3D& rotation) :
    _transform(position.get(), rotation.get())
{}

Transform3D Transform3D::operator*(const Transform3D& other) const
{
    return Transform3D(get() * other.get());
}

Vector3D Transform3D::operator*(const Vector3D& other) const
{
    return Vector3D(get() * other.get());
}

Transform3D Transform3D::inverse() const
{
    return Transform3D(rw::math::inverse(get()));
}

Vector3D Transform3D::p() const { return Vector3D(get().P()); }
Vector3D Transform3D::P() const { return Vector3D(get().P()); }

Rotation3D Transform3D::r() const { return Rotation3D(get().R()); }
Rotation3D Transform3D::R() const { return Rotation3D(get().R()); }

std::string Transform3D::__tostring() const { return toString(get()); }

const rw::math::Transform3D<>& Transform3D::get() const { return _transform; }

//======================================================================
// Kinematics
//======================================================================

//----------------------------------------------------------------------
// State
//----------------------------------------------------------------------

State State::copy() { return State(get()); }

//----------------------------------------------------------------------
// Frame
//----------------------------------------------------------------------

Frame::Frame(rw::kinematics::Frame* frame) :
    _frame(frame)
{}

Transform3D Frame::wt(const State& state) const
{
    return rw::kinematics::Kinematics::worldTframe(&get(), state.get());
}

Transform3D Frame::to(const Frame& frame, const State& state) const
{
    return rw::kinematics::Kinematics::frameTframe(
        &get(), &frame.get(), state.get());
}

std::string Frame::attachFrame(Frame& parent, State& state)
{
    try {
        get().attachTo(&parent.get(), state.get());
        return "";
    } catch (const rw::common::Exception& e) {
        return eToString(e);
    }
}

std::string Frame::__tostring() const { return toString(get()); }

//======================================================================
// Models
//======================================================================

//----------------------------------------------------------------------
// Device
//----------------------------------------------------------------------

Device::Device(rw::models::DevicePtr device) :
    _device(device)
{}

void Device::setQ(const Q& q, State& state) const
{
    get().setQ(q.get(), state.get());
}

Q Device::getQ(const State& state) const
{
    return Q(get().getQ(state.get()));
}

Frame Device::getBase() { return Frame(get().getBase()); }

Frame Device::getEnd() { return Frame(get().getEnd()); }

std::string Device::__tostring() const { return toString(get()); }

//----------------------------------------------------------------------
// WorkCell
//----------------------------------------------------------------------

State WorkCell::getDefaultState() const
{ return State(_workcell->getDefaultState()); }

std::string WorkCell::__tostring() const { return toString(get()); }

Frame WorkCell::getWorldFrame() const
{
    return Frame(get().getWorldFrame());
}

bool WorkCell::internal_has() const
{ return _workcell != 0; }

const std::string& WorkCell::internal_getErrorMessage() const
{ return _errorMessage; }

WorkCell::WorkCell(rw::models::WorkCellPtr workcell)
    :
    _workcell(workcell)
{}

WorkCell::WorkCell(const std::string& errorMessage)
    :
    _errorMessage(errorMessage)
{}

//======================================================================
// Other
//======================================================================

//----------------------------------------------------------------------
// Output
//----------------------------------------------------------------------

Output::Output(void* userdata)
{
    assert(userdata);
    _output = (rwlibs::lua::Output*)userdata;
}

void Output::write(const std::string& txt)
{
    _output->write(txt);
}

//----------------------------------------------------------------------
// Path
//----------------------------------------------------------------------

Path::Path(int len, State* states)
{
    for (int i = 0; i < len; i++)
        _path.push_back(states[i].get());
}

int Path::size() const { return (int)_path.size(); }
bool Path::empty() const { return _path.empty(); }

State Path::getEndState() const
{
    if (empty()) RW_THROW("Empty path");

    return State(_path.back());
}

Path Path::operator+(const Path& other) const
{
    robwork::StatePath result = _path;
    result.insert(result.end(), other._path.begin(), other._path.end());
    return Path(result);
}

//----------------------------------------------------------------------
// CollisionDetector
//----------------------------------------------------------------------

bool CollisionDetector::inCollision(const State& state) const
{
    return get().inCollision(state.get());
}

//----------------------------------------------------------------------
// PathPlanner
//----------------------------------------------------------------------

Path PathPlanner::query(
    const Q& from,
    const Q& to)
{
    std::vector<robwork::Q> path;
    _toQ->query(from.get(), to.get(), path);
    return Path(
        robwork::Models::getStatePath(
            *_device,
            path,
            _state));
}

Path PathPlanner::query(
    const Q& from,
    const Transform3D& to)
{
    std::vector<robwork::Q> path;
    _toT->query(from.get(), to.get(), path);
    return Path(
        robwork::Models::getStatePath(
            *_device,
            path,
            _state));
}

//----------------------------------------------------------------------
// Math utility functions
//----------------------------------------------------------------------

Rotation3D NS::rpy(double roll, double pitch, double yank)
{
    return Rotation3D(rw::math::RPY<>(roll, pitch, yank).toRotation3D());
}

Rotation3D NS::eaa(double x, double y, double z)
{
    return Rotation3D(rw::math::EAA<>(x, y, z).toRotation3D());
}

Rotation3D NS::inverse(const Rotation3D& val)
{
    return Rotation3D(inverse(val.get()));
}

Transform3D NS::inverse(const Transform3D& val)
{
    return Transform3D(inverse(val.get()));
}

//----------------------------------------------------------------------
// Models utility functions
//----------------------------------------------------------------------

WorkCell NS::loadWorkCell(const std::string& file)
{
    try {
        return WorkCell(rw::loaders::WorkCellLoader::load(file));
    } catch (const rw::common::Exception& e) {
        return WorkCell(eToString(e));
    }
}

WorkCell NS::makeWorkCell(void* userdata)
{
    rw::models::WorkCell* workcell = (rw::models::WorkCell*)userdata;
    RW_ASSERT(workcell);
    return WorkCell(workcell);
}

CollisionDetector NS::makeCollisionDetector(void* userdata)
{
    robwork::CollisionDetector* strategy = (robwork::CollisionDetector*)userdata;
    RW_ASSERT(strategy);
    return CollisionDetector(strategy);
}

CollisionDetector NS::makeCollisionDetectorFromStrategy(
    WorkCell& workcell, CollisionStrategy& strategy)
{
    return CollisionDetector(
        robwork::CollisionDetector::make(
            workcell.getPtr(), strategy.getPtr()));
}

CollisionStrategy NS::makeCollisionStrategy(void* userdata)
{
    robwork::CollisionStrategy* strategy = (robwork::CollisionStrategy*)userdata;
    RW_ASSERT(strategy);
    return CollisionStrategy(strategy);
}

rw::kinematics::Frame* NS::findFrame(
    const WorkCell& workcell, const std::string& name)
{
    return workcell.get().findFrame(name);
}

rw::models::Device* NS::findDevice(
    WorkCell& workcell, const std::string& name)
{
    return workcell.get().findDevice(name);
}

namespace
{
    std::string quote(const std::string& str)
    { return rw::common::StringUtil::quote(str); }
}

std::string NS::gripFrame(State& state, Frame& item, Frame& gripper)
{
    try {
		robwork::Kinematics::gripFrame(state.get(), item.get(), gripper.get());
        return "";
    } catch (const rw::common::Exception& e) {
        return eToString(e);
    }
}

Device NS::makeCompositeDevice(
    const std::string& name,
    Frame& base,
    int len,
    Device* devices,
    Frame& end,
    const State& state)
{
    vector<rw::models::DevicePtr> devs;
    for (int i = 0; i < len; i++) devs.push_back(&devices[i].get());
    return Device(
        rw::common::ownedPtr(
            new rw::models::CompositeDevice(
                &base.get(),
                devs,
                &end.get(),
                name,
                state.get())));
}

Device NS::makeCompositeDevice(
    const std::string& name,
    int len,
    Device* devices,
    const State& state)
{
    Frame base(devices[0].get().getBase());
    Frame end(devices[len - 1].get().getEnd());
    return makeCompositeDevice(name, base, len, devices, end, state);
}

#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
namespace
{
    class OptimizingPlanner : public robwork::QToQPlanner
    {
    public:
        OptimizingPlanner(
            robwork::QToQPlannerPtr planner,
            const robwork::PlannerConstraint& constraint)
            :
            _planner(planner),
            _optimizer(
                constraint,
                robwork::MetricFactory::makeInfinity<robwork::Q>())
        {}

    private:
        bool doQuery(
            const robwork::Q& from,
            const robwork::Q& to,
            robwork::QPath& result,
            const robwork::StopCriteria& stop)
        {
            robwork::QPath path;
            _planner->query(from, to, path, stop);

            const double maxTime = 10;
            const robwork::QPath optimized = _optimizer.shortCut(
                path,
                5000,
                maxTime,
                15 * robwork::Deg2Rad);

            result.insert(result.end(), optimized.begin(), optimized.end());
			return !optimized.empty();
        }

        robwork::QToQPlannerPtr _planner;
        robwork::PathLengthOptimizer _optimizer;
    };

    robwork::QToQPlannerPtr makeOptimizingPlanner(
        robwork::QToQPlannerPtr planner,
        const robwork::PlannerConstraint& constraint)
    {
        return robwork::ownedPtr(new OptimizingPlanner(planner, constraint));
    }

    robwork::QToQPlannerPtr makeDefaultQToQPlanner(
        const robwork::PlannerConstraint& constraint,
        robwork::DevicePtr device)
    {
        robwork::SBLSetup setup = robwork::SBLSetup::make(constraint, device);
        return makeOptimizingPlanner(
            robwork::SBLPlanner::makeQToQPlanner(setup),
            constraint);
    }

    robwork::QToTPlannerPtr makeDefaultQToTPlanner(
        robwork::QToQPlannerPtr toQ,
        robwork::DevicePtr dev,
        const robwork::State& state,
        const robwork::PlannerConstraint& constraint)
    {
        robwork::QIKSamplerPtr ikSampler =
            robwork::QIKSampler::make(dev, state, NULL, NULL);

        return
            robwork::QToTPlanner::makeToNearest(
                toQ,
                robwork::QIKSampler::makeConstrained(
                    ikSampler,
                    constraint.getQConstraintPtr(),
                    10),
                robwork::PlannerUtil::normalizingInfinityMetric(
                    dev->getBounds()),
                10);
    }
}

PathPlanner NS::makePathPlanner(
    Device& device,
    Frame& frame,
    const State& state,
    CollisionDetector& workcellDetector,
    void* pathPlannerFactoryRaw)
{
    // We construct a new collision detector by filtering away the pairs of
    // geometries we know can't be in collision.
    robwork::CollisionDetectorPtr detector = robwork::CollisionDetector::make(
        workcellDetector.get(), device.get(), state.get());

    // The simple (old) version that doesn't filter away geometries:
    /*
      robwork::CollisionDetectorPtr detector = workcellDetector.getPtr();
    */

	robwork::DevicePtr dev = device.getPtr();

    // If we are using another tool then wrap the device:
    if (&frame.get() != dev->getEnd())
		dev = robwork::Models::makeDevice(dev, state.get(), NULL, &frame.get());

    robwork::PlannerConstraint constraint = robwork::PlannerConstraint::make(
        detector,
        dev,
        state.get());

    // If a path planner factory has been registered:
    if (pathPlannerFactoryRaw) {
        robwork::PathPlannerFactory* factory =
            (robwork::PathPlannerFactory*)pathPlannerFactoryRaw;

        robwork::PathPlannerFactory::Planner planners =
            factory->make(dev, state.get(), constraint);

        if (!planners.toQ)
            planners.toQ = makeDefaultQToQPlanner(constraint, dev);

        if (!planners.toT)
            planners.toT = makeDefaultQToTPlanner(
                planners.toQ, dev, state.get(), constraint);

        return PathPlanner(
            planners.toQ, planners.toT, dev, state.get());
    }

    // Otherwise construct a default planner.
    else {
        robwork::SBLSetup setup = robwork::SBLSetup::make(constraint, dev);

        robwork::QToQPlannerPtr toQ = makeOptimizingPlanner(
            robwork::SBLPlanner::makeQToQPlanner(setup),
            constraint);
        robwork::QIKSamplerPtr ikSampler =
            robwork::QIKSampler::make(dev, state.get(), NULL, NULL);
        robwork::QToTPlannerPtr toT =
            robwork::SBLPlanner::makeQToTPlanner(setup, ikSampler);

        return PathPlanner(toQ, toT, dev, state.get());
    }
}

//----------------------------------------------------------------------
// Kinematics utility functions
//----------------------------------------------------------------------

State NS::makeState(void* userdata)
{
    rw::kinematics::State* state = (rw::kinematics::State*)userdata;
    RW_ASSERT(state);
    return State(*state);
}

void NS::writeState(void* userdata, const State& state)
{
    rw::kinematics::State* ptr = (rw::kinematics::State*)userdata;
    RW_ASSERT(ptr);
    *ptr = state.get();

    // Call the state changed event handler.
    RobWork::getStateChangedListener()(*ptr);
}

//----------------------------------------------------------------------
// State path utilities
//----------------------------------------------------------------------

void NS::storeStatePath(
    const WorkCell& workcell, const Path& path, const std::string& file)
{
    rw::loaders::PathLoader::storeVelocityTimedStatePath(
        workcell.get(), path.get(), file);
}
