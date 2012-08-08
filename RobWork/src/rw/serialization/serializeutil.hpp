#ifndef SERIALIZEUTIL_HPP
#define SERIALIZEUTIL_HPP

#include <rw/math/Q.hpp>
#include "InputArchive.hpp"
#include "OutputArchive.hpp"

namespace serialization {

    void save(const rw::math::Q& tmp, OutputArchive& oar, const std::string& id);

    rw::math::Q* load(rw::math::Q* tmp, InputArchive& iar, const std::string& id);

}

#endif
