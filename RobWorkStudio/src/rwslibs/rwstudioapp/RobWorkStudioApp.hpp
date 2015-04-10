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

#ifndef RWS_ROBWORKSTUDIOAPP_HPP_
#define RWS_ROBWORKSTUDIOAPP_HPP_
#include <boost/thread.hpp>
#include "RobWorkStudio.hpp"

namespace rws {

    /**
     * @brief a RobWorkStudio main application which may be instantiated in its own thread.
     */
    class RobWorkStudioApp
     {
     public:
    	/**
    	 * constructor
    	 * @param args [in] command line arguments for RobWorkStudio
    	 */
        RobWorkStudioApp(const std::string& args);

        //! destructor
        virtual ~RobWorkStudioApp();

        /**
         * @brief start robworkstudio in its own thread
         */
        void start();

        /**
         * @brief start robworkstudio in this thread. Notice this method call will
         * block until robworkstudio is exited.
         */
        void run();

        /**
         * @brief check if robworkstudio is running
         * @return true if running false otherwise
         */
        bool isRunning(){ return _isRunning; }

        /**
         * @brief get handle to the running RobWorkStudio instance.
         * @notice do not directly change Qt visualization objects, this will
         * produce segfaults. Instead use Qt events and the post* handles on
         * RobWorkStudio interface.
         * @return handle to RobWorkStudio
         */
        RobWorkStudio * getRobWorkStudio(){return _rwstudio;};

     private:
        RobWorkStudio *_rwstudio;
        std::string _args;
        boost::thread *_thread;
        bool _isRunning;
     };
}

#endif /* ROBWORKSTUDIOAPP_HPP_ */
