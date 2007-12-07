#ifndef rwlibs_components_camera_CAMERACOMPONENT_HPP_
#define rwlibs_components_camera_CAMERACOMPONENT_HPP_

#include <string>

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Command.hpp>
#include <rtt/Properties.hpp>

#include <rtt/TimeService.hpp>
#include <rtt/Time.hpp>

#include <core/sensor/Camera.hpp>
#include <core/sensor/Image.hpp>

namespace rwlibs { namespace components {

    /** @addtogroup components */
    /*@{*/


    class CameraComponent : public RTT::TaskContext
    {
    public:
        
        // This component outputs a Image and a timestamp
        RTT::WriteDataPort< const rw::core::sensor::Image* > _imageOut;
        
        /// Dataport which contains grabbing timestamp
        RTT::WriteDataPort< RTT::TimeService::ticks > _capture_time;
        
        // Command to start fetch image
        RTT::Command<bool(void)> _fetchImageCmd;
    
    public:
        /**
         * @brief constructor
         * @param name [in] the name of the camera component
         * @param camera [in] a pointer to a camera interface
         */
        CameraComponent(const std::string& name = "Camera", rw::core::sensor::Camera* camera);
        
        /**
         * @brief default destructor 
         */
        virtual ~CameraComponent();
    
    protected:
        // method
        const rw::core::sensor::Image* getImage();
        // command
        bool acquireImage();
        // command completion condition
        bool isImageAcquired() const;
    
        // methods implemented according to TaskContext interface
        bool startup();
    
        void update();
    
        void shutdown();
    
    private:
        CameraComponent(){};
        rw::core::sensor::Camera *_camera;
        bool _imageFetched;
        rw::core::sensor::Image *_camImage;
    };
    
    /* @} */

}} // end namespaces


#endif /*CAMERACOMPONENT_HPP_*/
