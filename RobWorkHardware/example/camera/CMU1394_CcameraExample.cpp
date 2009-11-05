
#include <rwhw/camera/CMU1394Camera_c.hpp>
#include <windows.h>

using namespace rwhw;
using namespace rw::sensor;

int main(int argc, char** argv){
    std::cout << "Getting list of camera handles."  << std::endl;

    std::vector<CMU1394Camera*> cameras = CMU1394Camera::getCameraHandles();

    std::cout << "Number of Cameras: " << cameras.size() << std::endl;

    for(size_t i=0; i<cameras.size() ; i++){
        std::cout << "Camera name: " << cameras[i]->getName() << std::endl;
        std::cout << "Camera model info: " << cameras[i]->getModelInfo() << std::endl;
    }

    if(cameras.size()<1)
        return 0;

    if(!cameras[0]->initialize()){
        std::cout << "Stopped at initialize!!!"<< std::endl;
        return 0;
    }

    if(!cameras[0]->start()){
        std::cout << "Stopped at trying to start!!!"<< std::endl;
        return 0;
    }

    double framerate = cameras[0]->getFrameRate();
    std::cout << "Frame rate: " << framerate << std::endl;

    if( cameras[0]->isShutterAvailable() ){
    	std::cout << "Shutter available!! \n"
				  << "- Current Val: " << cameras[0]->getShutter() << std::endl;
    	std::pair<double,double> range = cameras[0]->getShutterBounds();
    	std::cout << "- Range      : [" << range.first << ";" << range.second << "]" << std::endl;
    }

    while(!cameras[0]->isImageReady()){
        cameras[0]->acquire();
        //std::cout << "Acquire." << std::endl;
    }
    std::cout << "Image grabbed!" << std::endl;
    const Image *img = cameras[0]->getImage();
    std::cout << "Image size: " << img->getWidth() << " : " << img->getHeight() << std::endl;

    img->saveAsPGM("testimage.pgm");
    img->saveAsPGMAscii("testimageAscii.pgm");

    cameras[0]->stop();

    return 0;
}
