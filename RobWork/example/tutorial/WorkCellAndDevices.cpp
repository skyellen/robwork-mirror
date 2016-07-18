#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

using rw::common::Log;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;

int main(int argc, char** argv) {

    WorkCell::Ptr wc = WorkCellLoader::Factory::load("SimpleWorkCell.wc.xml");
    if (wc.isNull())
    	RW_THROW("Could not load workcell!");

    Log::infoLog() << "Name of workcell: " << wc->getName() << std::endl;
    // get the default state
    State state = wc->getDefaultState();
    Frame* worldFrame = wc->getWorldFrame();
    // find a frame by name, remember NULL is a valid return
    Frame* frame = wc->findFrame("FixedFrameName");
    // find a frame by name, but with a specific frame type
    FixedFrame* fframe = wc->findFrame<FixedFrame>("FixedFrameName");
    MovableFrame* mframe = wc->findFrame<MovableFrame>("MovableFrameName");
    // find a device by name
    Device::Ptr device = wc->findDevice("SerialDeviceName");
    SerialDevice::Ptr sdevice = wc->findDevice<SerialDevice>("SerialDeviceName");



    // calculate the transform from one frame to another
    Transform3D<> fTmf = Kinematics::frameTframe(frame, mframe, state);
    // calculate the transform from world to frame
    Transform3D<> wTmf = Kinematics::worldTframe( mframe, state );
    // we can find the world to frame transform by a little jogling
    Transform3D<> wTf = wTmf * inverse(fTmf);
    // test if frame is a dynamic attachable frame
    if( Kinematics::isDAF( mframe ) ){
        // attach mframe to end of serial device
        Kinematics::gripFrame(mframe, sdevice->getEnd(), state);
    }


    // get device base frame
    Frame *base = sdevice->getBase();
    // get device end effector
    Frame *end = sdevice->getEnd();
    // calculate base to end transform
    Transform3D<> bTe = sdevice->baseTend(state);
    // or just base to any frame
    Transform3D<> bTmf = sdevice->baseTframe(mframe, state);
    // get device name
    std::string sdevicename = sdevice->getName();
    // the degrees of freedom of this device
    int dof = sdevice->getDOF();
    // set the configuration of the device to zero
    sdevice->setQ( Q::zero(dof) , state );

    return 0;
}
