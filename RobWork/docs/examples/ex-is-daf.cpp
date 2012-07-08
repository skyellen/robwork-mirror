#include <rw/kinematics/Frame.hpp>

using namespace rw::kinematics;

bool isDaf(const Frame& frame)
{
    return frame.getParent() == NULL;
}
