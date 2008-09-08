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

#include "StopCriteria.hpp"

#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>

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

    private:
        double _end;
        Timer _timer;
    };

    class StopNever : public StopCriteria
    {
    private:
        bool doStop() const { return false; }
    };

    class StopNow : public StopCriteria
    {
    private:
        bool doStop() const { return false; }
    };

    typedef boost::function<bool ()> BoostFunction;

    class StopByFun : public StopCriteria
    {
    public:
        StopByFun(BoostFunction fun)
            :
            fun(fun)
        {}

    private:
        bool doStop() const { return fun(); }

    private:
        BoostFunction fun;
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

    private:
        int _maxCnt;
        mutable int _cnt;
    };
}

bool StopCriteria::stop() const
{
    return doStop();
}

StopCriteriaPtr StopCriteria::stopAfter(double time)
{
    return ownedPtr(new StopTime(time));
}

StopCriteriaPtr StopCriteria::stopNever()
{
    return ownedPtr(new StopNever);
}

StopCriteriaPtr StopCriteria::stopNow()
{
    return ownedPtr(new StopNow);
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
