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

#include "StopCriteria.hpp"

#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>

using namespace rw::pathplanning;
using namespace rw::common;

namespace
{
    class StopTime : public StopCriteria
    {
    public:
        StopTime(double time) :
            _end(time)
        {}

    private:
        bool doStop() const
        {
            const double time = _timer.getTime();
            if (time > _end)
                return true;
            else
                return false;
        }

        StopCriteriaPtr doInstance() const
        {
            return ownedPtr(new StopTime(_end));
        }

    private:
        double _end;
        Timer _timer;
    };

    class StopFixed : public StopCriteria
    {
    public:
        StopFixed(bool value) : _value(value) {}

    private:
        bool doStop() const { return _value; }

        StopCriteriaPtr doInstance() const
        {
            return ownedPtr(new StopFixed(_value));
        }

        bool _value;
    };

    typedef boost::function<bool ()> BoostFunction;

    class StopByFun : public StopCriteria
    {
    public:
        StopByFun(BoostFunction fun)
            :
            _fun(fun)
        {}

    private:
        bool doStop() const { return _fun(); }

        StopCriteriaPtr doInstance() const
        {
            return ownedPtr(new StopByFun(_fun));
        }

    private:
        BoostFunction _fun;
    };

    class StopByFlag : public StopCriteria
    {
    public:
        StopByFlag(bool* flag) :
            _flag(flag)
        {
            RW_ASSERT(flag);
        }

    private:
        bool doStop() const { return *_flag; }

        StopCriteriaPtr doInstance() const
        {
            return ownedPtr(new StopByFlag(_flag));
        }

    private:
        bool* _flag;
    };

    class StopCnt : public StopCriteria
    {
    public:
        StopCnt(int cnt) : _maxCnt(cnt), _cnt(0) {}

    private:
        bool doStop() const
        {
            return ++_cnt > _maxCnt;
        }

        StopCriteriaPtr doInstance() const
        {
            return ownedPtr(new StopCnt(_maxCnt));
        }

    private:
        int _maxCnt;
        mutable int _cnt;
    };

    class StopEither : public StopCriteria
    {
    public:
        StopEither(const std::vector<StopCriteriaPtr>& criteria)
            : _criteria(criteria)
        {}

    private:
        bool doStop() const
        {
            BOOST_FOREACH(const StopCriteriaPtr& stop, _criteria) {
                if (stop->stop())
                    return true;
            }
            return false;
        }

        StopCriteriaPtr doInstance() const
        {
            std::vector<StopCriteriaPtr> criteria;
            BOOST_FOREACH(const StopCriteriaPtr& stop, _criteria) {
                criteria.push_back(stop->instance());
            }
            return ownedPtr(new StopEither(criteria));
        }

    private:
        std::vector<StopCriteriaPtr> _criteria;
    };
}

//----------------------------------------------------------------------
// StopCriteria

bool StopCriteria::stop() const
{
    return doStop();
}

StopCriteriaPtr StopCriteria::instance() const
{
    return doInstance();
}

//----------------------------------------------------------------------
// Constructors

StopCriteriaPtr StopCriteria::stopAfter(double time)
{
    return ownedPtr(new StopTime(time));
}

StopCriteriaPtr StopCriteria::stopNever()
{
    return ownedPtr(new StopFixed(false));
}

StopCriteriaPtr StopCriteria::stopNow()
{
    return ownedPtr(new StopFixed(true));
}

StopCriteriaPtr StopCriteria::stopByFlag(bool* stop)
{
    return ownedPtr(new StopByFlag(stop));
}

StopCriteriaPtr StopCriteria::stopByFun(BoostFunction fun)
{
    return ownedPtr(new StopByFun(fun));
}

StopCriteriaPtr StopCriteria::stopCnt(int cnt)
{
    return ownedPtr(new StopCnt(cnt));
}

StopCriteriaPtr StopCriteria::stopEither(
    const std::vector<StopCriteriaPtr>& criteria)
{
    return ownedPtr(new StopEither(criteria));
}

StopCriteriaPtr StopCriteria::stopEither(
    const StopCriteriaPtr& a,
    const StopCriteriaPtr& b)
{
    std::vector<StopCriteriaPtr> criteria;
    criteria.push_back(a);
    criteria.push_back(b);
    return stopEither(criteria);
}
