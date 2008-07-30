#include "PointTimeIndex.hpp"

#include <rw/common/macros.hpp>
#include <map>

using namespace rw::interpolator;

namespace
{
    typedef PointTimeIndex::Key Key;

    struct Compare
    {
        bool operator()(const Key& a, const Key& b) const
        {
            // The use of <= versus < is deliberate.
            return
                a._start <= b._start &&
                a._end < b._end;
        }
    };

    typedef std::map<Key, bool, Compare> StepSet;
    typedef std::vector<Key> StepArray;

    const Key nothingKey(0, 0, -1);

    // Build the mapping and return the end time.
    double buildStepSet(const std::vector<double>& timeSteps, StepSet& mapping)
    {
        // We traverse the list and insert a key for each element.
        double start = 0;
        const int len = (int)timeSteps.size();
        for (int i = 0; i < len; i++) {
            const double val = timeSteps[i];
            RW_ASSERT(val >= 0);

            const double end = start + val;
            mapping.insert(
                std::make_pair(Key(start, end, i), true));

            start = end;
        }

        return start;
    }

    // Build the mapping and return the end time.
    double buildStepArray(
        const std::vector<double>& timeSteps,
        StepArray& array)
    {
        // We traverse the list and append a key for each element.
        double start = 0;

        const int len = (int)timeSteps.size();
        for (int i = 0; i < len; i++) {
            const double val = timeSteps[i];
            RW_ASSERT(val >= 0);

            const double end = start + val;
            array.push_back(Key(start, end, i));

            start = end;
        }

        return start;
    }

    bool useStepSet(const std::vector<double>& timeSteps)
    {
        // To be sure of a speedup you should use about 18 ~= 35 / 2 here.
        return timeSteps.size() > 35;
    }

    Key lookupStepSet(double time, const StepSet& mapping)
    {
        // We add a special case here, because our mapping does not capture this
        // case as is.
        if (time == 0) {
            if (mapping.empty()) return nothingKey;
            else return (*mapping.begin()).first;
        }

        typedef StepSet::const_iterator I;
        const Key key(time, time, -1);
        const I pos = mapping.find(key);
        if (pos == mapping.end())
            return nothingKey;
        else
            return pos->first;
    }

    Key lookupStepArray(double time, const StepArray& array)
    {
        if (time == 0) {
            if (array.empty()) return nothingKey;
            else return array.front();
        }
        else if (time < 0) return nothingKey;
        else {
            typedef StepArray::const_iterator I;
            for (I p = array.begin(); p != array.end(); ++p) {
                const Key& key = *p;
                if (key._end >= time)
                    return key;
            }

            return nothingKey;
        }
    }
}

struct PointTimeIndex::Impl
{
    std::vector<double> timeSteps;
    double endTime;

    StepSet mapping;
    StepArray array;

    // Depending on the number of elements either a linear search or a tree
    // search is done. This is great example of over-engineering, but you can
    // actually measure the speed-up on a benchmark, and the break even point is
    // pretty close to the value chosen in useStepSet() if searches are done to
    // the center of the array on average. So herhaps one should use half the
    // value, just to be sure of a speedup...

    Impl(const std::vector<double>& timeSteps) :
        timeSteps(timeSteps)
    {
        if (useStepSet(timeSteps)) 
            endTime = buildStepSet(timeSteps, mapping);
        else
            endTime = buildStepArray(timeSteps, array);
    }

    Key getKey(double time)
    {
        if (useStepSet(timeSteps))
            return lookupStepSet(time, mapping);
        else
            return lookupStepArray(time, array);
    }
};

Key PointTimeIndex::getKey(double time)
{ return impl->getKey(time); }

PointTimeIndex::PointTimeIndex(const std::vector<double>& timeSteps) :
    impl(new Impl(timeSteps))
{}

PointTimeIndex::~PointTimeIndex()
{ delete impl; }

double PointTimeIndex::getEndTime() const
{ return impl->endTime; }

const std::vector<double>& PointTimeIndex::getTimeSteps() const
{ return impl->timeSteps; }
