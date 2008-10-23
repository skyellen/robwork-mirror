#include "RWXMLFile.hpp"

using namespace rw::kinematics;
using namespace rw::models;

namespace {

	struct DummyWorkcell {
	public:
		bool isFrameInDevice(Frame &frame){

		}

		bool isFrameInDevice(Frame &frame, Device &dev){

		}

		WorkCell *wc;
		std::vector<Frame*> frames;
		std::map<Frame*,Device*> frameToDevice;
		std::vector<Device*> devices;
	};

	// method for writing a property
	void writeProperty(std::ostream &ostr){
		ostr << "<Property ";
		ostr << "</Property>" << std::endl;
	}

	// method for writing the frame transform
	void writeFrameTransform(Frame &frame, std::ostream &ostr ){

	}

	void writeJoint(Joint &joint, std::ostream &ostr){

	}

	// method for writing a frame
	void writeFrame(Frame &frame, std::ostream &ostr){

	}

	std::string getDeviceType(Device &dev){
	    if( dynamic_cast<SerialDevice>(&dev) ){
	        return std::string("SerialDevice");
	    } else if(dynamic_cast<SerialDevice>(&dev) ){
	    } else if(dynamic_cast<TreeDevice>(&dev) ){
	    } else if(dynamic_cast<ParallelDevice>(&dev) ){
	    } else if(dynamic_cast<MobileDevice>(&dev) ){
	    } else if(dynamic_cast<SerialDevice>(&dev) ){
	    }
	}

	void writeDevice(DummyWorkcell& wc, Device &dev, std::ostream &ostr){
	    std::string devType = getDeviceType(dev);

	    // write device joint structure


		// write any frame that is related to the device but that isnt a joint
		BOOST_FOREACH(Frame* frame, wc.frames){
			if( !wc.isFrameInDevice(frame,dev) )
				continue;
			writeFrame(*frame, ostr);
		}
	}

	void writeWorkcell(DummyWorkcell& dwc, std::ostream &ostr){
		ostr << "<WorkCell name=\"" << dwc.wc->getName() << "\">" << std::endl;

	    // write all frames that does not belong to a device
		ostr << "<!-- First we list all workcell frames -->" << std::endl;
		BOOST_FOREACH(Frame* frame, wc.frames){
			if( wc.isFrameInDevice(frame) )
				continue;
			writeFrame(*frame, ostr);
		}

		ostr << "<!-- Next we list all devices in the workcell -->" << std::endl;
		// now write the devices
		BOOST_FOREACH(Device* dev, wc.devices){
			writeDevice(wc, *dev, ostr)
		}

		// write the collision setup to a file

		ostr << "</WorkCell>" << std::endl;
	}

}


void RWXMLFile::SaveWorkCell(rw::models::WorkCell& wc,
						 const rw::kinematics::State& initState,
						 const std::string& filename)
{
	// construct file
	std::fstream fstr(filename.c_str());

	// construct the DummyWorkcell
	DummyWorkcell dwc;
	dwc.wc = &wc;

	// write the workcell to file
	writeWorkcell(dwc, fstr);
}
