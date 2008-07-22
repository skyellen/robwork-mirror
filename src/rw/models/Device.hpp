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

#ifndef rw_models_Device_HPP
#define rw_models_Device_HPP

/**
 * @file Device.hpp
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <boost/shared_ptr.hpp>

#include <string>
#include <ostream>

namespace rw { namespace math { class Jacobian; }}

namespace rw { namespace kinematics {
    class Frame;
    class State;
}}

namespace rw { namespace models {

    class Joint;
    class DeviceJacobian;

    /** @addtogroup models */
    /*@{*/

    class Device;

    //! A pointer to a Device.
    typedef rw::common::Ptr<Device> DevicePtr;

    /**
     * @brief An abstract device class
     *
     * @todo document this
     */
    class Device
    {
    public:
        /**
         * Constructs a device with a name
         *
         * @param name [in] name of the device
         */
        Device(const std::string& name) :
            _name(name)
        {}

        /**
         * @brief Virtual destructor
         */
        virtual ~Device(){}

        /**
         * @brief Sets configuration vector @f$ \mathbf{q} \in \mathbb{R}^n @f$
         *
         * @param q [in] configuration vector @f$ \mathbf{q} @f$
         * @param state [in] state into which to set @f$ \mathbf{q} @f$
         *
         * @pre q.size() == getDOF()
         */
        virtual void setQ(const math::Q& q, kinematics::State& state) const = 0;

        /**
         * @brief Gets configuration vector @f$ \mathbf{q}\in \mathbb{R}^n @f$
         *
         * @param state [in] state from which which to get @f$ \mathbf{q} @f$
         * @return configuration vector @f$ \mathbf{q} @f$
         */
        virtual math::Q getQ(const kinematics::State& state) const = 0;

        /**
         * @brief Returns the upper @f$ \mathbf{q}_{min} \in \mathbb{R}^n @f$ and
         * lower @f$ \mathbf{q}_{max} \in \mathbb{R}^n @f$ bounds of the joint space
         *
         * @return std::pair containing @f$ (\mathbf{q}_{min}, \mathbf{q}_{max}) @f$
         */
        virtual std::pair<math::Q, math::Q> getBounds() const = 0;

        /**
         * @brief Sets the upper @f$ \mathbf{q}_{min} \in \mathbb{R}^n @f$ and
         * lower @f$ \mathbf{q}_{max} \in \mathbb{R}^n @f$ bounds of the joint space
         *
         * @param bounds [in] std::pair containing
         * @f$ (\mathbf{q}_{min}, \mathbf{q}_{max}) @f$
         */
        virtual void setBounds(const std::pair<math::Q, math::Q>& bounds) = 0;

        /**
         * @brief Returns the maximal velocity of the joints
         * \f$\mathbf{\dot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \dot{\mathbf{q}}_{min}=-\dot{\mathbf{q}}_{max}\f$
         *
         * @return the maximal velocity
         */
        virtual math::Q getVelocityLimits() const = 0;

        /**
         * @brief Sets the maximal velocity of the joints
         * \f$\mathbf{\dot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \dot{\mathbf{q}}_{min}=-\dot{\mathbf{q}}_{max}\f$
         *
         * @param vellimits [in] Q with the maximal velocity
         */
        virtual void setVelocityLimits(const math::Q& vellimits) = 0;

        /**
         * @brief Returns the maximal acceleration of the joints
         * \f$\mathbf{\ddot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \ddot{\mathbf{q}}_{min}=-\ddot{\mathbf{q}}_{max}\f$
         *
         * @return the maximal acceleration
         */
        virtual math::Q getAccelerationLimits() const = 0;

        /**
         * @brief Sets the maximal acceleration of the joints
         * \f$\mathbf{\ddot{q}}_{max}\in \mathbb{R}^n\f$
         *
         * It is assumed that \f$
         * \ddot{\mathbf{q}}_{min}=-\ddot{\mathbf{q}}_{max}\f$
         *
         * @param  acclimits [in] the maximal acceleration
         */
        virtual void setAccelerationLimits(const math::Q& acclimits) = 0;

        /**
         * @brief Returns number of active joints
         * @return number of active joints @f$ n @f$
         */
        virtual size_t getDOF() const = 0;

        /**
         * @brief Returns the name of the device
         * @return name of the device
         */
        std::string getName() const { return _name; }

        /**
         * @brief Sets the name of the Device
         * @param name [in] the new name of the frame
         */
        void setName(const std::string& name) { _name = name; }

        /**
         * @brief a method to return the frame of the base of the
         * device.
         * @return the base frame
         */
        virtual kinematics::Frame* getBase() = 0;

        /**
         * @brief a method to return the frame of the base of the
         * device.
         * @return the base frame
         */
        virtual const kinematics::Frame* getBase() const = 0;

        /**
         * @brief a method to return the frame of the end of the device
         *
         * @return the end frame
         */
        virtual kinematics::Frame* getEnd() = 0;

        /**
         * @brief a method to return the frame of the end of the device
         *
         * @return the end frame
         */
        virtual const kinematics::Frame* getEnd() const = 0;

        /**
         * @brief Calculates the homogeneous transform from base to a frame f
         * @f$ \robabx{b}{f}{\mathbf{T}} @f$
         *
         * @return the homogeneous transform @f$ \robabx{b}{f}{\mathbf{T}} @f$
         */
        math::Transform3D<double> baseTframe(
            const kinematics::Frame* f, const kinematics::State& state) const;

        /**
         * @brief Calculates the homogeneous transform from base to the end frame
         * @f$ \robabx{base}{end}{\mathbf{T}} @f$
         *
         * @return the homogeneous transform @f$ \robabx{base}{end}{\mathbf{T}} @f$
         */
        math::Transform3D<double> baseTend(const kinematics::State& state) const;

        /**
         * @brief Calculates the homogeneous transform from world to base @f$
         * \robabx{w}{b}{\mathbf{T}} @f$
         *
         * @return the homogeneous transform @f$ \robabx{w}{b}{\mathbf{T}} @f$
         */
        math::Transform3D<double> worldTbase(const kinematics::State& state) const;

        /**
         * @brief Calculates the jacobian matrix of the end-effector described
         * in the robot base frame @f$ ^{base}_{end}\mathbf{J}_{\mathbf{q}}(\mathbf{q})
         * @f$
         *
         * @param state [in] State for which to calculate the Jacobian
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^{base}_{end}}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * This method calculates the jacobian relating joint velocities (@f$
         * \mathbf{\dot{q}} @f$) to the end-effector velocity seen from
         * base-frame (@f$ \nu^{ase}_{end} @f$)
         *
         * \f[
         * \nu^{base}_{end} =
         * {^{base}_{end}}\mathbf{J}_\mathbf{q}(\mathbf{q})\mathbf{\dot{q}}
         * \f]
         *
         *
         * The jacobian matrix \f[ {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \f]
         * is defined as:
         *
         * \f[
         * {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         * \frac{\partial ^{base}\mathbf{x}_n}{\partial \mathbf{q}}
         * \f]
         *
         * Where:
         * \f[
         *  {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         *  \left[
         *    \begin{array}{cccc}
         *      {^{base}_1}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) &
         *      {^{base}_2}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) &
         *      \cdots &
         *      {^b_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \\
         *    \end{array}
         *  \right]
         * \f]
         * where \f$ {^{base}_i}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \f$ is defined by
         * \f[
         *  {^{base}_i}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         *  \begin{array}{cc}
         *    \left[
         *      \begin{array}{c}
         *        {^{base}}\mathbf{z}_i \times {^{i}\mathbf{p}_n} \\
         *        {^{base}}\mathbf{z}_i \\
         *      \end{array}
         *    \right] & \textrm{revolute joint}
         *  \end{array}
         * \f]
         * \f[
         *  {^{base}_i}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         *  \begin{array}{cc}
         *    \left[
         *      \begin{array}{c}
         *        {^{base}}\mathbf{z}_i \\
         *        \mathbf{0} \\
         *    \end{array}
         *    \right] & \textrm{prismatic joint} \\
         *  \end{array}
         * \f]
         *
         * By default the method forwards to baseJframe().
         */
        virtual math::Jacobian baseJend(const kinematics::State& state) const = 0;

        /**
         * @brief Calculates the jacobian matrix of a frame f described in the
         * robot base frame @f$ ^{base}_{frame}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * @param frame [in] Frame for which to calculate the Jacobian
         * @param state [in] State for which to calculate the Jacobian
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^{base}_{frame}}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * This method calculates the jacobian relating joint velocities (@f$
         * \mathbf{\dot{q}} @f$) to the frame f velocity seen from base-frame
         * (@f$ \nu^{base}_{frame} @f$)
         *
         * \f[
         * \nu^{base}_{frame} =
         * {^{base}_{frame}}\mathbf{J}_\mathbf{q}(\mathbf{q})\mathbf{\dot{q}}
         * \f]
         *
         *
         * The jacobian matrix \f[ {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) \f]
         * is defined as:
         *
         * \f[
         * {^{base}_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) =
         * \frac{\partial ^{base}\mathbf{x}_n}{\partial \mathbf{q}}
         * \f]
         *
         * By default the method forwards to baseJframes().
         */
        virtual math::Jacobian baseJframe(
            const kinematics::Frame* frame,
            const kinematics::State& state) const;

        /**
           @brief The Jacobian for a sequence of frames.

           A Jacobian is computed for each of the frames and the Jacobians are
           stacked on top of eachother.
        */
        virtual math::Jacobian baseJframes(
            const std::vector<kinematics::Frame*>& frames,
            const kinematics::State& state) const = 0;

        /**
           @brief DeviceJacobian for the end frame.

           By default this method forwards to baseDJframe().
        */
        virtual boost::shared_ptr<DeviceJacobian> baseDJend(
            const kinematics::State& state) const;

        /**
           @brief DeviceJacobian for a particular frame.

           By default this method forwards to baseDJframes().
        */
        virtual boost::shared_ptr<DeviceJacobian> baseDJframe(
            const kinematics::Frame* frame,
            const kinematics::State& state) const;

        /**
           @brief DeviceJacobian for a sequence of frames.
        */
        virtual boost::shared_ptr<DeviceJacobian> baseDJframes(
            const std::vector<kinematics::Frame*>& frames,
            const kinematics::State& state) const = 0;

    private:
        std::string _name;

    private:
        Device(const Device&);
        Device& operator=(const Device&);
    };

    /**
       @brief Streaming operator for devices.
     */
    std::ostream& operator<<(std::ostream& out, const Device& device);

    /*@}*/
}} // end namespaces

#endif // end include guard
