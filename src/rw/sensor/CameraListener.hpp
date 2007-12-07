/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_sensor_CameraListener_HPP
#define rw_sensor_CameraListener_HPP

/**
 * @file CameraListener.hpp
 */

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief interface used for listening for camera events
     */
    class CameraListener
    {
    protected:
        /**
         * @brief constructor
         */
        CameraListener();

    public:
        /**
         * @brief destructor
         */
        virtual ~CameraListener();

        /**
         * @brief called when the camera wish to signal a change.
         */
        virtual void notifyChanged() = 0;

    private:
        CameraListener(const CameraListener&);
        CameraListener& operator=(const CameraListener&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
