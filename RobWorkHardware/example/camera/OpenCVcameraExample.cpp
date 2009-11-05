#include <rwhw/camera/OpenCVCamera.hpp>

using namespace rwhw;
using namespace rw::sensor;
int main(int argc, char** argv){

   std::cout << "Getting list of camera handles."  << std::endl;

    std::vector<OpenCVCamera*> cameras = OpenCVCamera::getCameraHandles();

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
    double fps = cameras[0]->getFrameRate();
    std::cout << "Frame rate: " << fps << std::endl;


    cameras[0]->acquire();

    while(!cameras[0]->isImageReady()){

        //std::cout << "Acquire." << std::endl;
    }


    std::cout << "Image grabbed!" << std::endl;
    const Image* img = cameras[0]->getImage();
    RW_ASSERT(img);
    std::cout << "Image size: " << img->getWidth() << " : " << img->getHeight() << std::endl;

    img->saveAsPPM("c:/testImg1.ppm");

    cameras[0]->stop();

    return 0;
}
