#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

int main(int argc, char** argv) {

    WorkCell::Ptr wc = WorkCellLoader::load("SimpleWorkCell.wc.xml");

    // Simple JacobianIKSolver
    State state = wc->getDefaultState();

    JacobianIKSolver *sol = new JacobianIKSolver(device,state);


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
    if( Kinematics::isDAF( *mframe ) ){
        // attach mframe to end of serial device
        Kinematics::gripFrame(state, *mframe, *sdevice->getEnd() );
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
}
