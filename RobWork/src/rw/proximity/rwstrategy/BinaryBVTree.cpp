#include "BinaryBVTree.hpp"

#include <rw/geometry/Triangle.hpp>

using namespace rw::proximity;

template class BinaryBVTree<rw::geometry::OBB<double>, rw::geometry::Triangle<double> >;
template class BinaryBVTree<rw::geometry::OBB<float>, rw::geometry::Triangle<float> >;
