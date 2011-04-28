/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "RWXMLFile.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <rw/math/Constants.hpp>
#include <stack>

#include <rw/math.hpp>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>

#include <boost/foreach.hpp>

using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::common;

int clvl=0;

namespace {
    std::string wlvl(){
        std::stringstream sstr;
        for(int i=0;i<clvl;i++)
            sstr << " ";
        return sstr.str();
    }

	struct DummyWorkcell {
	public:
		bool isFrameInDevice(Frame *frame){
		    return frameToDevice.find(frame)!=frameToDevice.end();
		}

		bool isFrameInDevice(Frame *frame, Device *dev){
		    if(isFrameInDevice(frame))
		        return frameToDevice[frame] == dev;
		    return false;
		}

		bool isFrameParentToDevice(Frame *frame){
		    return parentToDevice.find(frame)!=parentToDevice.end();
		}

		void init(){
			Frame *world = wc->getWorldFrame();
			//std::vector<>Kinematics::findAllFrames(world, state);
			devices = wc->getDevices();
			frames = Kinematics::findAllFrames(world, state);
			BOOST_FOREACH(Device::Ptr devp, devices){
			    Device *dev = devp.get();
				std::vector<Frame*> dframes = Kinematics::findAllFrames( dev->getBase() );
				std::string dname = dev->getName();
				BOOST_FOREACH(Frame* dframe, dframes){
					std::string fname = dframe->getName();

					if(fname.length() > dname.length()){
						if(dname == fname.substr(0,dname.length())){
							frameToDevice[dframe] = dev;
						}
					}
				}

				if(TreeDevice *tdev = dynamic_cast<TreeDevice*>(dev)){
					BOOST_FOREACH(Frame* endf, tdev->getEnds())
							isEndEffector[endf] = true;
				} else {
					isEndEffector[dev->getEnd()] = true;
				}


				parentToDevice[dev->getBase()->getParent()] = dev;
			}

		}

		State state;
		WorkCell *wc;
		std::vector<Frame*> frames;
		std::map<Frame*,Device*> frameToDevice;
		std::map<Frame*,Device*> parentToDevice;
		std::map<Frame*,bool> isEndEffector;
		std::vector<Device::Ptr> devices;
	};

	// remove the device scope name from frame
	std::string scopedName(Device* dev, Frame *frame){
		std::string fname = frame->getName();
		std::string dname = dev->getName();
		std::cout << dname << "-->" << fname << std::endl;
		if(fname.length()>dname.length()){
			if( dname == fname.substr(0,dname.length())  ){
				return fname.substr(dname.length()+1, fname.length()-(dname.length()+1));
			}
		}
		return fname;


	}

	std::string scopedName(DummyWorkcell &dwc, Frame *frame){
		if(dwc.isFrameInDevice(frame)){
			return scopedName(dwc.frameToDevice[frame], frame);
		}

		return frame->getName();
	}

	std::string scopedName(DummyWorkcell &dwc, Frame *sframe, Frame *frame){
		std::cout << "scoped name" << std::endl;
		std::cout << sframe->getName() << "-->" << frame->getName() << std::endl;
		if(dwc.isFrameInDevice(frame)){
			std::cout << "scoped me" << std::endl;
			if(dwc.isFrameInDevice(sframe,dwc.frameToDevice[frame]))
				return scopedName(dwc.frameToDevice[frame], frame);
		}
		std::cout << "scoped " << std::endl;
		return frame->getName();
	}

	std::string getDeviceType(Device &dev){
	        if( dynamic_cast<SerialDevice*>(&dev) ){
	            return std::string("SerialDevice");
	        } else if(dynamic_cast<TreeDevice*>(&dev) ){
	            return std::string("TreeDevice");
	        } else if(dynamic_cast<ParallelDevice*>(&dev) ){
	            return std::string("ParallelDevice");
	        } else if(dynamic_cast<MobileDevice*>(&dev) ){
	            return std::string("MobileDevice");
	        } else {
	            RW_THROW("unknown device type!");
	        }
	        return "";
	    }

	// method for writing a property
	void writeProperty(std::ostream &ostr){
		ostr << "<Property ";
		ostr << "</Property>\n";
	}

	// method for writing the frame transform
	void writeTransform(Transform3D<> t3d, std::ostream &ostr ){
	    RPY<> rpy(t3d.R());
	    Vector3D<> p = t3d.P();
	    ostr << wlvl();
	    ostr << "<RPY>" << rpy(0)*Rad2Deg << " " << rpy(1)*Rad2Deg << " " << rpy(2)*Rad2Deg << "</RPY> ";
        ostr << "<Pos>" << p(0) << " " << p(1) << " "<< p(2) << "</Pos>\n";
	}

	void writeRevoluteJoint(DummyWorkcell& dwc, RevoluteJoint &joint, std::ostream &ostr){
        ostr << wlvl()
             << "<Joint name=\"" << scopedName(dwc, &joint)
             << "\" refframe=\"" << scopedName(dwc, &joint, joint.getParent(dwc.state))
             << "\" type=\"Revolute\">\n";

        clvl++;
        writeTransform(joint.getTransform(dwc.state), ostr );
        Q pmin = joint.getBounds().first;
        Q pmax = joint.getBounds().second;
        ostr << wlvl() << "<PosLimit min=\""<<pmin(0)*Rad2Deg<<"\" max=\""<<pmax(0)*Rad2Deg<<"\" />\n";

        Q vel = joint.getMaxAcceleration();
        ostr << wlvl() << "<VelLimit max=\""<<vel(0)*Rad2Deg<<"\" />\n";

        Q acc = joint.getMaxAcceleration();
        ostr << wlvl() << "<AccLimit max=\""<<acc(0)*Rad2Deg<<"\" />\n";

        clvl--;
        ostr << wlvl() << "</Joint>\n";
	}

   void writePrismaticJoint(DummyWorkcell& dwc, PrismaticJoint &joint, std::ostream &ostr){
        ostr << wlvl()
             << "<Joint name=\"" << scopedName(dwc,&joint)
             << "\" refframe=\"" << scopedName(dwc,&joint, joint.getParent(dwc.state))
             << "\" type=\"Prismatic\">\n";
        clvl++;
        writeTransform(joint.getTransform(dwc.state), ostr );
        Q pmin = joint.getBounds().first;
        Q pmax = joint.getBounds().second;
        ostr << wlvl() << "<PosLimit min=\""<<pmin(0)<<"\" max=\""<<pmax(0)<<"\" />\n";

        Q vel = joint.getMaxAcceleration();
        ostr << wlvl() << "<VelLimit max=\""<<vel(0)<<"\" />\n";

        Q acc = joint.getMaxAcceleration();
        ostr << wlvl() << "<AccLimit max=\""<<acc(0)<<"\" />\n";

        clvl--;
        ostr << wlvl() << "</Prismatic>\n";
    }

	void writeDependentRevoluteJoint(DummyWorkcell& dwc, DependentRevoluteJoint &joint, std::ostream &ostr){
       ostr << wlvl()
            << "<Joint name=\"" << scopedName(dwc, &joint)
            << "\" refframe=\"" << scopedName(dwc, &joint, joint.getParent(dwc.state))
            << "\" type=\"Revolute\">\n";

       clvl++;
       writeTransform(joint.getFixedTransform(), ostr );
       ostr << wlvl()
			<< "<Depend on\"" << scopedName(dwc,&joint.getOwner(),&joint)
			<< "\" gain=\"" << joint.getScale()
			<< "\" offset=\""<< joint.getOffset()*Rad2Deg<< "\" />\n";
       clvl--;
       ostr << wlvl() << "</Joint>\n";
	}

  void writeDependentPrismaticJoint(DummyWorkcell& dwc, DependentPrismaticJoint &joint, std::ostream &ostr){
       ostr << wlvl()
            << "<Joint name=\"" << scopedName(dwc,&joint)
            << "\" refframe=\"" << scopedName(dwc,&joint, joint.getParent(dwc.state))
            << "\" type=\"Prismatic\">\n";
       clvl++;
       writeTransform(joint.getFixedTransform(), ostr );
       ostr << wlvl()
			<< "<Depend on\"" << scopedName(dwc,&joint.getOwner(),&joint)
			<< "\" gain=\"" << joint.getScale()
			<< "\" offset=\""<< joint.getOffset()<< "\" />\n";
       clvl--;
       ostr << wlvl() << "</Prismatic>\n";
   }

   // method for writing a frame
    void writeFixedFrame(DummyWorkcell& dwc,FixedFrame &frame, std::ostream &ostr){
        ostr << wlvl()
             << "<Frame name=\"" << scopedName(dwc,&frame)
             << "\" refframe=\"" << scopedName(dwc,&frame,frame.getParent(dwc.state)) << "\">\n";

        clvl++;
        writeTransform(frame.getTransform(dwc.state), ostr );
        clvl--;
        ostr << wlvl() << "</Frame>\n";
    }

    // method for writing a frame
     void writeMovableFrame(DummyWorkcell& dwc,MovableFrame &frame, std::ostream &ostr){
    	 ostr << wlvl()
              << "<Frame name=\"" << scopedName(dwc,&frame)
              << "\" refframe=\"" << scopedName(dwc,&frame, frame.getParent(dwc.state))
              << "\" type=\"Movable\">\n";

         clvl++;
         writeTransform(frame.getTransform(dwc.state), ostr );
         clvl--;
         ostr << wlvl() << "</Frame>\n";
     }

	// method for writing a frame
	void writeFrame(DummyWorkcell& dwc, Frame &frame, std::ostream &ostr){
		// world frame is implicit, and should not be defined in the file
		if(&frame==dwc.wc->getWorldFrame())
			return;
		if(frame.getParent()==NULL){
			RW_WARN("frame: " << frame.getName() << " is DAF!");
		}

        if( FixedFrame *ff = dynamic_cast<FixedFrame*>(&frame) ){
            writeFixedFrame(dwc, *ff, ostr );
        } else if(MovableFrame *mf = dynamic_cast<MovableFrame*>(&frame) ){
        	writeMovableFrame(dwc, *mf, ostr );
        } else if(RevoluteJoint *rj = dynamic_cast<RevoluteJoint*>(&frame) ){
            writeRevoluteJoint(dwc,*rj,ostr);
        } else if(PrismaticJoint *pj = dynamic_cast<PrismaticJoint*>(&frame) ){
            writePrismaticJoint(dwc,*pj,ostr);
        } else if(DependentRevoluteJoint* rdj = dynamic_cast<DependentRevoluteJoint*>(&frame) ){
        	writeDependentRevoluteJoint(dwc,*rdj,ostr);
        } else if(DependentPrismaticJoint* rdj = dynamic_cast<DependentPrismaticJoint*>(&frame) ){
        	writeDependentPrismaticJoint(dwc,*rdj,ostr);
        } else {
            RW_THROW("unknown Frame type!");
        }
	}

   void writeCollisionInfo(DummyWorkcell& dwc,CollisionModelInfo &info, Frame *frame, std::ostream &ostr){
        std::stringstream sstr;
        sstr << scopedName(dwc, frame) << "Geo";
        ostr << wlvl()
             << "<CollisionModel name=\"" <<  sstr.str()
             << "\" refframe=\"" << scopedName(dwc, frame) << "\">\n";

        clvl++;
        writeTransform(info.getTransform(), ostr );
        std::string id = info.getGeoString();
        if(id[0]!='#'){
            ostr << wlvl() << "<Polytope file=\"" << id << "\" />\n";
        }
        clvl--;
        ostr << wlvl() << "</CollisionModel>\n";
    }

	void writeDrawableInfo(DummyWorkcell& dwc,DrawableModelInfo &info, Frame *frame, std::ostream &ostr){
        std::stringstream sstr;
        sstr << scopedName(dwc, frame) << "Geo";
	    ostr << wlvl()
             << "<Drawable name=\"" <<  sstr.str()
             << "\" refframe=\"" << scopedName(dwc, frame)
             << "\" colmodel=\"Disabled\">\n";

        clvl++;
        writeTransform(info.getTransform(), ostr );
        std::string id = info.getId();
        if(id[0]!='#'){
            ostr << wlvl() << "<Polytope file=\"" << id << "\" />\n";
        }
        clvl--;
        ostr << wlvl() << "</Drawable>\n";
	}

	void writeDevice(DummyWorkcell& wc, Device &dev, std::ostream &ostr){
		ostr << "\n";
	    std::string devType = getDeviceType(dev);
	    ostr << wlvl() << "<" << devType << " name=\""<< dev.getName() <<"\">\n";
	    clvl++;
	    ostr << "\n";
	    // write device joint structure
	    std::vector<Frame*> flist;
	    std::stack<Frame*> frames;
	    frames.push( dev.getBase() );
	    while(!frames.empty()){
	        Frame *frame = frames.top();
	        frames.pop();
	        flist.push_back(frame);
	        writeFrame(wc, *frame, ostr);
	        BOOST_FOREACH(Frame &child, frame->getChildren(wc.state) ){
	            if( !wc.isFrameInDevice(&child, &dev) )
	                continue;
	            frames.push(&child);
	        }
	    }
	    ostr << "\n";
	    // write all drawables and collision models
	    ostr << "<!-- drawables -->\n";
	    BOOST_FOREACH(Frame* frame, flist){
	        // first we insert the drawables
	        std::vector<DrawableModelInfo> infos = DrawableModelInfo::get(frame);
            BOOST_FOREACH(DrawableModelInfo info, infos){
                writeDrawableInfo(wc, info, frame, ostr);
            }
	    }

	    ostr << "\n";
       // write all drawables and collision models
        ostr << "<!-- Collision models -->\n";
        BOOST_FOREACH(Frame* frame, flist){
            // first we insert the drawables
            std::vector<CollisionModelInfo> infos = CollisionModelInfo::get(frame);
            BOOST_FOREACH(CollisionModelInfo info, infos){
                writeCollisionInfo(wc, info, frame, ostr);
            }
        }

		clvl--;
		ostr << wlvl() << "</" << devType << ">\n";
	}

	void writeWorkcell(DummyWorkcell& dwc, std::ostream &ostr){
		ostr << "<WorkCell name=\"" << dwc.wc->getName() << "\">\n";
		clvl++;
	    // write all frames that does not belong to a device
		ostr << "<!-- First we list all workcell frames -->\n";
		BOOST_FOREACH(Frame* frame, dwc.frames){
			if( dwc.isFrameInDevice(frame) )
				continue;
			if( dwc.isFrameParentToDevice(frame))
				continue;
			writeFrame(dwc, *frame, ostr);
		}

		ostr << "<!-- Next we list all devices in the workcell -->\n";
		// now write the devices
		BOOST_FOREACH(Device::Ptr dev, dwc.devices){
			Frame *parent = dev->getBase()->getParent();
			writeFrame(dwc, *parent, ostr);
			writeDevice(dwc, *dev, ostr);
		}

		// write the collision setup to a file
		clvl--;
		ostr << "</WorkCell>\n";
	}

}


void RWXMLFile::saveWorkCell(rw::models::WorkCell& wc,
						 const rw::kinematics::State& initState,
						 const std::string& filename)
{
	// construct file
	std::ofstream fstr( filename.c_str() );
	clvl = 0;
	// construct the DummyWorkcell
	DummyWorkcell dwc;
	dwc.wc = &wc;
	dwc.state = initState;

	dwc.init();
	// write the workcell to file
	writeWorkcell(dwc, fstr);
}
