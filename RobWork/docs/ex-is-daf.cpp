#include <rw/kinematics/Frame.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

bool isDaf(const Frame& frame)
{
    return frame.getParent() == NULL;
}
