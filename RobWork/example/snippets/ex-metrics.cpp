#include <rw/math/Metric.hpp>
#include <rw/math/MetricFactory.hpp>

using rw::common::Ptr;
using namespace rw::math;

void metricExample()
{
    typedef Vector3D<> V;
    typedef Ptr<Metric<V> > VMetricPtr;

    VMetricPtr m1 = MetricFactory::makeManhattan<V>();
    VMetricPtr m2 = MetricFactory::makeEuclidean<V>();
    VMetricPtr mInf = MetricFactory::makeInfinity<V>();

    const V a(0, 0, 0);
    const V b(1, 1, 1);

    std::cout
        << m1->distance(a, b) << " is " << 3.0 << "\n"
        << m2->distance(a, b) << " is " << sqrt(3.0) << "\n"
        << mInf->distance(a, b) << " is " << 1 << "\n";
}
