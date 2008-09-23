#ifndef rwlibs_simulation_GLScanner3D_HPP
#define rwlibs_simulation_GLScanner3D_HPP

#include <rw/sensor/Scanner3D.hpp>

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/PerspectiveTransform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

namespace rwlibs { namespace simulation {

    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief This simulated 3D scanner implementation use the OpenGL depth buffer for
     * doing depth tests. The scanner assimilates 3D range devices such as the Swissranger.
     */
    class GLScanner3D: public rw::sensor::Scanner3D {
    public:

        /**
         * @brief constructor
         * @param width [in] width of image
         * @param height [in] height of image
         * @param fov [in] the vertical field of view angle in degree
         * @param drawer [in] the WorkCellGLDrawer that draws the OpenGL scene
         * @param state [in] the state of the workcell
         */
        GLScanner3D(
            int width, int height, double fov,
            rw::kinematics::Frame* frame,
            const std::string& name,
            rwlibs::drawable::WorkCellGLDrawer *drawer)
            :
            rw::sensor::Scanner3D(frame,name),
            _fieldOfView(fov),_drawer(drawer),
            _img(width,height)
        {}

        /**
         * @brief destructor
         */
        virtual ~GLScanner3D();

        /**
         * @copydoc rw::sensor::Sensor::update
         */
        void update(double dt, const rw::kinematics::State& state);

        const rw::sensor::Image3D& getImage(){ return _img;};

        double getFrameRate(){ return _framerate; };

        void setFrameRate(double framerate){
            _framerate = framerate;
        }

        void acquire(){

        }

        bool isImageReady() = 0;
    private:

        double _fieldOfView; // in the y-axis
        rwlibs::drawable::WorkCellGLDrawer *_drawer;
        double _framerate;
        rw::sensor::Image3D _img;
    };

    /* @} */

}} // end namespaces

#endif // end include guard
