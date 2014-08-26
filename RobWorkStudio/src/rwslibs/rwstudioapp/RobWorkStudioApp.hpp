/*
 * RobWorkStudioApp.hpp
 *
 *  Created on: Mar 1, 2012
 *      Author: jimali
 */

#ifndef RWS_ROBWORKSTUDIOAPP_HPP_
#define RWS_ROBWORKSTUDIOAPP_HPP_
#include <boost/thread.hpp>
#include "RobWorkStudio.hpp"

namespace rws {

    /**
     * @brief a RobWorkStudio main application which is instantiated in its own thread.
     */
    class RobWorkStudioApp
     {
     public:
        RobWorkStudioApp(const std::string& args);
        ~RobWorkStudioApp();

        void start();
        void run();

        bool isRunning(){ return _isRunning; }

        RobWorkStudio * getRobWorkStudio(){return _rwstudio;};

        RobWorkStudio *_rwstudio;
        std::string _args;
        boost::thread *_thread;
        bool _isRunning;
     };
}

#endif /* ROBWORKSTUDIOAPP_HPP_ */
