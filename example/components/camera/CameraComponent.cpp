#include "CameraComponent.hpp"

using namespace RTT;
using namespace rwlibs::components;

CameraComponent::CameraComponent(const std::string& name, rw::core::sensor::Camera* camera=NULL)
    : RTT::TaskContext(name),
      _imageOut("Image2D"),
      _capture_time("TimeStamp"),
      _fetchImageCmd("fetchImage",
                      &CameraComponent::acquireImage,
                      &CameraComponent::isImageAcquired,
                      this),
      _camera(camera),
      _imageFetched(false)
{
    this->ports()->addPort(&_imageOut,"The last acquired image");
    this->ports()->addPort(&_capture_time,"Timestamp of last acquired image");

    this->commands()->addCommand(&_fetchImageCmd,"Use this command to acquire an image.");
}

CameraComponent::~CameraComponent()
{

}

const rw::core::sensor::Image* CameraComponent::getImage(){
    return _camera->getImage();
}

bool CameraComponent::startup(){
    bool succes = true;
    if(_camera==NULL)
        return false;
    /*if( !_camera->isInitialized() )
        succes &= _camera->initialize();
    if( !_camera->isStarted() )
        succes &= _camera->start();

    if( !succes ){
        Logger::log() << Logger::Error << "(CaptureCamera) Could not initialize capturing..." << Logger::endl;
        return false;
    }*/

    Logger::log() << Logger::Info << "(CameraComponent) started..." << Logger::endl;

    // initialize data flow
    _imageOut.Set( NULL );
    //_imageOut.Set( _camera->getImage() );
    _capture_time.Set(TimeService::Instance()->ticksGet());

    return succes;
}

void CameraComponent::update(){
    _camera->acquire();
    //while( _camera->isImageReady() ){
        _capture_time.Set(TimeService::Instance()->ticksGet());
        _imageOut.Set( _camera->getImage() );
        //const rw::core::sensor::Image *p = _camera->getImage();
        if(_camera==NULL)
            std::cout << "NULL" << std::endl;
        //std::cout << p << std::endl;
    //}
    //_capture_time.Set(TimeService::Instance()->ticksGet());
}

void CameraComponent::shutdown(){
    _camera->stop();
    Logger::log() << Logger::Info << "(CameraComponent) stopping Capture..." << Logger::endl;
}

// command
bool CameraComponent::acquireImage(){
    return true;
}

// command completion condition
bool CameraComponent::isImageAcquired() const {
    return _imageFetched;
}

