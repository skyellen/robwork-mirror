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
