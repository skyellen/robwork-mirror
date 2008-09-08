#include "GeometryFace.hpp"

using namespace rw::geometry;
using namespace rw::math;

const std::vector<Face<float> >& GeometryFace::getFaces() const {
    return *_faces;
}
