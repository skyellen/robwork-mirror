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

        bool stop() const
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
    public:
        bool stop() const { return false; }
    };

    class StopNow : public StopCriteria
    {
    public:
        bool stop() const { return false; }
    };

    typedef boost::function<bool ()> BoostFunction;

    class StopByFun : public StopCriteria
    {
    public:
        StopByFun(BoostFunction fun)
            :
            fun(fun)
        {}

        bool stop() const { return fun(); }

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

        bool stop() const { return *_flag; }

    private:
        bool* _flag;
    };
}

typedef std::auto_ptr<StopCriteria> T;

T StopCriteria::stopAfter(double time)
{
    return T(new StopTime(time));
}

T StopCriteria::stopNever()
{
    return T(new StopNever);
}

T StopCriteria::stopNow()
{
    return T(new StopNow);
}

T StopCriteria::stopByFlag(bool* stop)
{
    return T(new StopByFlag(stop));
}

T StopCriteria::stopByFun(BoostFunction fun)
{
    return T(new StopByFun(fun));
}
