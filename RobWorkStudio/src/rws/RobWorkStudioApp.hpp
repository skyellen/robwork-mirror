/*
 * RobWorkStudioApp.hpp
 *
 *  Created on: Mar 1, 2012
 *      Author: jimali
 */

#ifndef RWS_ROBWORKSTUDIOAPP_HPP_
#define RWS_ROBWORKSTUDIOAPP_HPP_

#include "RobWorkStudio.hpp"

namespace rws {

    /**
     * @brief a RobWorkStudio main application which is instantiated in its own thread.
     */
    class RobWorkStudioApp : public QThread
     {
     public:
        RobWorkStudioApp(const std::string& args);
        ~RobWorkStudioApp();
         void run();

         RobWorkStudio *_rwstudio;
         std::string _args;
     };
}

#endif /* ROBWORKSTUDIOAPP_HPP_ */
