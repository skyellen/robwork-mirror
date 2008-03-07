#include "Action.hpp"

#include <rw/common/macros.hpp>

using namespace rw::task;
using namespace std;

AttachFrameAction::AttachFrameAction(
    rw::kinematics::MovableFrame *child,
    rw::kinematics::Frame *parent)
    :
    _child(child),
    _parent(parent)
{
    RW_ASSERT(_child);
    RW_ASSERT(_parent);
}
