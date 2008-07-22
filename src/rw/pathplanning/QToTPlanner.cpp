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

#include "QToTPlanner.hpp"
#include "QSampler.hpp"

using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::invkin;
using namespace rw::models;
using namespace rw::kinematics;

namespace
{
    class RegionPlanner : public QToTPlanner
    {
    public:
        RegionPlanner(
            QToQSamplerPlannerPtr planner,
            IterativeIKPtr solver,
            DevicePtr device,
            const State& state,
            int maxAttempts)
            :
            _planner(planner),
            _solver(solver),
            _device(device),
            _state(state),
            _maxAttempts(maxAttempts)
        {}

    private:
        bool doQuery(
            const rw::math::Q& from,
            const rw::math::Transform3D<>& baseTend,
            Path& path,
            const StopCriteria& stop)
        {
            QSamplerPtr sampler = QSampler::makeIterativeIK(
                _solver,
                _device,
                _state,
                baseTend,
                _maxAttempts);

            return _planner->query(from, *sampler, path, stop);
        }

    private:
        QToQSamplerPlannerPtr _planner;
        IterativeIKPtr _solver;
        DevicePtr _device;
        State _state;
        int _maxAttempts;
    };
}

std::auto_ptr<QToTPlanner> QToTPlanner::make(
    QToQSamplerPlannerPtr planner,
    IterativeIKPtr solver,
    DevicePtr device,
    const State& state,
    int maxAttempts)
{
    typedef std::auto_ptr<QToTPlanner> T;
    return T(new RegionPlanner(planner, solver, device, state, maxAttempts));
}
