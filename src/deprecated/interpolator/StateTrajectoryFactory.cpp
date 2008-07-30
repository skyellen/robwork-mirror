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

#include "StateTrajectoryFactory.hpp"
#include "StateTrajectory.hpp"
#include "PointTimeIndex.hpp"
#include "PointTimeIndexFactory.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>

#define NS StateTrajectoryFactory

using namespace rw::interpolator;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

typedef StateTrajectoryFactory::TimedState TimedState;

namespace
{
    /**
       @brief True iff <code>|x| < accuracy</code>.
    */
    inline bool isZero(double x, double accuracy = 1e-14)
    {
        return fabs(x) < accuracy;
    }

    // LinearStateTrajectory is a trajectory build by connecting configurations by
    // straight line segments

    class LinearStateTrajectory : public StateTrajectory
    {
    public:
        LinearStateTrajectory(
            std::auto_ptr<PointTimeIndex> indexer,
            const std::vector<State>& path)
            :
            _path(path),
            _indexer(indexer)
        {}

        State get(double time) const
        {
            const PointTimeIndex::Key& key = _indexer->getKey(time);

            const int index = key._index;
            if (index == -1)
                RW_THROW(
                    "Time "
                    << time
                    << " is out of path range "
                    << "[0, "
                    << _indexer->getEndTime()
                    << "].");

            // Now find the points in question.
            const State& a = _path[index];
            const State& b = _path[index + 1];

            // The start and end time:
            const double a_time = key._start;
            const double b_time = key._end;

            // Special case if the segment is of length zero.
            if (isZero(b_time - a_time)) return a;

            // The relative position on the segment:
            const double pos = (time - a_time) / (b_time - a_time);

            // The configuration for that instant in time:
            State state = a + pos * (b - a);

            // Use the tree state of 'a'.
            a.setTreeStateInState(state);

            // Return the state.
            return state;
        }

        double getEndTime() const
        {
            return _indexer->getEndTime();
        }

    private:
        std::vector<State> _path;
        std::auto_ptr<PointTimeIndex> _indexer;
    };

    class EmptyStateTrajectory : public StateTrajectory
    {
    public:
        EmptyStateTrajectory() {}

        State get(double time) const
        {
            RW_THROW("get() called on empty state trajectory.");
            // To avoid a compiler error:
            return get(0);
        }

        double getEndTime() const { return -1; }
    };

    std::vector<State> getStatePath(const std::vector<TimedState>& path)
    {
        std::vector<State> result;
        typedef std::vector<TimedState>::const_iterator I;
        for (I p = path.begin(); p != path.end(); ++p)
            result.push_back(p->getValue());
        return result;
    }
}

std::auto_ptr<StateTrajectory> NS::makeLinear(
    const WorkCell& workcell,
    const std::vector<State>& path)
{
    typedef std::auto_ptr<StateTrajectory> T;
    return T(
        new LinearStateTrajectory(
            PointTimeIndexFactory::make(workcell, path),
            path));
}

std::auto_ptr<StateTrajectory> NS::makeLinear(
    const std::vector<TimedState>& path)
{
    typedef std::auto_ptr<StateTrajectory> T;
    return T(
        new LinearStateTrajectory(
            PointTimeIndexFactory::make(path),
            getStatePath(path)));
}

std::auto_ptr<StateTrajectory> NS::makeEmpty()
{
    typedef std::auto_ptr<StateTrajectory> T;
    return T(new EmptyStateTrajectory());
}
