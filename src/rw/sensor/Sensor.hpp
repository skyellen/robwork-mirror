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

#ifndef rw_sensor_Sensor_HPP
#define rw_sensor_Sensor_HPP

/**
 * @file Sensor.hpp
 */

#include <string>

namespace rw { namespace kinematics {
    class Frame;
}} // end namespaces

namespace rw { namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

    /**
     * @brief a generel sensor interface.
     */
    class Sensor
    {
    protected:
        /**
         * @brief constructor
         *
         * @param frame [in] Positioning of the frame in space.
         * @param name [in] the name of this sensor
         * @param description [in] description of the sensor
         * @param identifier [in] the identifier of the this sensor (type)
         *
         */
        Sensor(
            kinematics::Frame* frame,
            const std::string& name,
            const std::string& description,
            int identifier);

        /**
         * @brief sets the name of this sensor
         * @param name [in] name of this sensor
         */
        void setName(const std::string& name) { _name = name; }

        /**
         * @brief sets the description of this sensor
         * @param description [in] description of this sensor
         */
        void setDescription(const std::string& description)
        { _description = description; }

    public:
        /**
         * @deprecated Use getName.
         */
        const std::string& getSensorName() const;

        /**
         * @brief returns the name of this sensor
         * @return name of sensor
         */
        const std::string& getName() const { return _name; }

        /**
         * @brief returns the identifier of this sensor
         * @return identifier of this sensor
         */
        int getIdentifier() const { return _identifier; }

        /**
         * @brief returns a description of this sensor
         * @return reference to this sensors description
         */
        const std::string& getDescription() const { return _description; }

        /**
         * @brief The frame to which the sensor is attached.
         *
         * The frame can be NULL.
         */
        kinematics::Frame* getFrame() const { return _frame; }

        /**
         * @brief Sets the frame to which the sensor should be attached
         *
         * @param frame The frame, which can be NULL
         */
        void setFrame(kinematics::Frame* frame) { _frame = frame; }

    private:
        kinematics::Frame* _frame;
        std::string _name;
        std::string _description;
        int _identifier;
    };

    /** @} */
}} // end namespaces

#endif // end include guard
