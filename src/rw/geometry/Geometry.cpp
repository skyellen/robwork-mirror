#include "Geometry.hpp"

using namespace rw::geometry;

Geometry::Geometry(const Geometry& other)
{
    _id = other._id;
}

Geometry& Geometry::operator=(const Geometry& other)
{
    _id = other._id;
    return *this;
}
