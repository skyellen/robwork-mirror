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


#ifndef DEVICETAB_H
#define DEVICETAB_H

#include <QWidget>

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/VectorND.hpp>
#include <rw/math/Q.hpp>

#include <rw/common/Ptr.hpp>

namespace rw { namespace invkin { class IterativeIK; } }
namespace rw { namespace kinematics { class MovableFrame; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class WorkCell; } }

class QCheckBox;
class QDoubleSpinBox;
class QSlider;
class QGridLayout;
class QComboBox;
class QLabel;

//! @brief Widget for jogging a single value, such as the joint of a device or a Cartesian translation/rotation.
class Slider : public QWidget
{
    Q_OBJECT

public:
	/**
	 * @brief Construct new slider for some adjustable value.
	 * @param title [in] title that identifies the value.
	 * @param low [in] the lowest possible value.
	 * @param high [in] the highest possible value.
	 * @param layout [in] a grid layout - this class uses the 6 first columns in \b row .
	 * @param row [in] the row of \b layout to insert elements in.
	 * @param parent [in] the owner of this widget.
	 */
    Slider(const std::string& title,
           double low,
           double high,
           QGridLayout* layout,
           int row,
           QWidget* parent);

    //! @brief Adjust the values after a change in units.
    void unitUpdated();

    /**
     * @brief The current value chosen.
     * @return the value chosen.
     */
    double value() const;

    /**
     * @brief Change the current value.
     * @param val [in] the new value.
     */
    void setValue(double val);

    /**
     * @brief Set the factor used to convert to a certain unit.
     * @param converter [in] the factor to convert the units used.
     */
    void setUnitConverter(double converter) {
        _toUnit = converter;
    }

    /**
     * @brief Get the factor currently used to convert to used units.
     * @return the factor.
     */
    double getUnitConverter() const {
        return _toUnit;
    }

    /**
     * @brief Set a description of the units used.
     * @param str [in] description of the units.
     */
    void setUnitDescription(const std::string& str){
        _desc = str;
    }

    //void showEndLabel(bool enabled){
    //    _endlabelEnabled = enabled;
    //}

private slots:
    void boxValueChanged(double val);
    void sliderValueChanged(int val);
    void enableDisable(int val);

signals:
	//! @brief Emitted whenever the joint value changes.
    void valueChanged();

private:
    void setSliderValueFromBox(double val);
    void setBoxValueFromSlider(int val);

    double _low;
    double _high;

    QSlider* _slider;
    QDoubleSpinBox* _box;

    bool _boxChanged;
    bool _sliderChanged;

    QLabel *_title, *_lowLabel, *_highLabel;

    double _toUnit;
    std::string _desc;

    //bool _endlabelEnabled;
};

//! Widget for a set of joint sliders.
class JointSliderWidget: public QWidget {
  Q_OBJECT

public:
    //! @brief Constructor.
    JointSliderWidget();

    /**
     * @brief Setup the widget.
     * @param titles [in] titles of the joints.
     * @param bounds [in] lower and upper bounds for the joints.
     * @param q [in] initial values for the joints.
     * @param enablers [in] (optional) set to true to show a checkbox for each slider.
     * @param enableAngularCombined [in] (optional) if enablers are used, control the angular directions as one group (default is false).
     */
    void setup(const std::vector<std::string>& titles,
               const std::pair<rw::math::Q,rw::math::Q>& bounds,
               const rw::math::Q& q,
			   bool enablers = false,
			   bool enableAngularCombined = false);

    /**
     * @brief Set the units.
     * @param converters [in] the factor used to convert to units.
     * @param descriptions [in] a description of the chosen units.
     */
    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    /**
     * @brief Set the values of the joints.
     * @param q [in] new joint values.
     */
    void updateValues(const rw::math::Q& q);

    /**
     * @brief Set the values of the joints where the enablers are unchecked.
     * @param q [in] new joint values.
     */
    void updateInactiveValues(const rw::math::Q& q);

    /**
     * @brief Get the current values of joints.
     * @return the current values.
     */
    rw::math::Q getQ();

    /**
     * @brief Get the enabled state of each slider.
     * @return a vector of the enabled states (empty vector if enablers are not used).
     */
    std::vector<bool> enabledState() const;

signals:
	/**
	 * @brief Emitted when the joint values are changed.
	 * @param q [in] the new values.
	 */
    void valueChanged(const rw::math::Q& q);
    
public slots:
	//! @brief Opens up input dialog for pasting a new set of values.
    void paste();

private slots:
	void valueChanged();
	void angularChanged(int state);
    
private:
    std::vector<Slider*> _sliders;
    std::vector<QCheckBox*> _enablers;

    QGridLayout* _layout;
    bool _enableAngularCombined;
};



//! @brief Widget for a setting a 6D pose.
class TransformSliderWidget: public QWidget {
    Q_OBJECT
public:
	/**
	 * @brief Construct new widget.
	 * @param bounds [in] the lower and upper bounds - 6 elements of each corresponding to x,y,z,R,P and Y.
	 * @param transform [in] the initial transform.
	 * @param useRPY [in] (optional) set to true to use RPY values, false to use EAA values (default is RPY).
     * @param enablers [in] (optional) set to true to show a checkbox for each slider.
     * It is only possible to check/uncheck checkboxes for EAA values (not RPY).
	 */
    TransformSliderWidget(const std::pair<rw::math::Q, rw::math::Q>& bounds, const rw::math::Transform3D<>& transform, bool useRPY = true, bool enablers = false);

    //! @copydoc JointSliderWidget::setUnits
    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    /**
     * @brief Change the transform.
     * @param transform [in] new transform.
     */
    void updateValues(const rw::math::Transform3D<>& transform);

    /**
     * @brief Change the transform where the enablers have been unchecked.
     * @param transform [in] new transform.
     */
    void updateInactiveValues(const rw::math::Transform3D<>& transform);

    /**
     * @brief Get the currently chosen transform.
     * @return the current transform.
     */
    rw::math::Transform3D<> getTransform();

    /**
     * @brief Get the enabled state for the x,y,z,R,P and Y state respectively.
     * @return the enabled state.
     */
    rw::math::VectorND<6, bool> enabledState() const;

signals:
	/**
	 * @brief Emitted when the transform is changed.
	 * @param transform [in] the new transform.
	 */
    void valueChanged(const rw::math::Transform3D<>& transform);
    
public slots:
	//! @brief Opens up dialog for easy pasting of a new pose.
    void paste();
    
private slots:
    void valueChanged(const rw::math::Q& q);
    
private:
    JointSliderWidget* _jointSliderWidget;

    bool _updating;
    const bool _useRPY;
};


//! @brief Widget for a adjusting a MovableFrame.
class MovableFrameTab: public QWidget {
    Q_OBJECT

public:
	/**
	 * @brief Construct a new tab for adjusting a MovableFrame.
	 * @param bounds [in] the lower and upper bounds - 6 elements of each corresponding to x,y,z,R,P and Y.
	 * @param frame [in] the MovableFrame to adjust.
	 * @param workcell [in] the workcell, allowing choice of different reference frames.
	 * @param state [in] the current state.
	 */
    MovableFrameTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
                 rw::kinematics::MovableFrame* frame,
                 rw::models::WorkCell* workcell,
                 const rw::kinematics::State& state);

    //! @copydoc JointSliderWidget::setUnits
    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

   // void setup(const std::pair<rw::math::Q, rw::math::Q>& bounds, rw::kinematics::Frame* frame);

    /**
     * @brief Change the transform.
     * @param state [in] the state with the new configuration.
     */
    void updateValues(const rw::kinematics::State& state);

signals:
	/**
	 * @brief Emitted when the transform is changed.
	 * @param state [in] the new state.
	 */
    void stateChanged(const rw::kinematics::State& state);

private slots:
    void transformChanged(const rw::math::Transform3D<>& transform);
    void refFrameChanged(int index);

private:
    std::vector<Slider*> _sliders;
    QGridLayout* _layout;
    QComboBox* _cmbFrames;
    std::vector<rw::kinematics::Frame*> _frames;
    rw::kinematics::State _state;
    rw::kinematics::MovableFrame* _frame;
    rw::kinematics::Frame* _refframe;

    void doUpdateValues();

    TransformSliderWidget* _transformSliderWidget;
	bool _updating;
};


//! @brief Widget for moving devices in Cartesian space. For inverse kinematics, the rw::invkin::JacobianIKSolver is used.
class CartesianDeviceTab: public QWidget {
    Q_OBJECT
public:
	/**
	 * @brief Construct a new tab for a device that can be moved in Cartesian space.
	 * @param bounds [in] the lower and upper bounds - 6 elements of each corresponding to x,y,z,R,P and Y.
	 * @param device [in] the device to move.
	 * @param workcell [in] the workcell.
	 * @param state [in] the initial state.
	 */
    CartesianDeviceTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
    	rw::common::Ptr<rw::models::Device> device,
		rw::models::WorkCell* workcell,
        const rw::kinematics::State& state);

    //! @copydoc JointSliderWidget::setUnits
    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    //! @copydoc MovableFrameTab::updateValues
    void updateValues(const rw::kinematics::State& state);

signals:
	//! @copydoc MovableFrameTab::stateChanged
    void stateChanged(const rw::kinematics::State& state);

private slots:
    void transformChanged(const rw::math::Transform3D<>& transform);

    void tcpFrameChanged(int index);
    void refFrameChanged(int index);

private:
    QComboBox* _cmbRefFrame;
    QComboBox* _cmbTcpFrame;

    rw::kinematics::State _state;
	rw::common::Ptr<rw::models::Device> _device;
    std::vector<rw::kinematics::Frame*> _frames;
    rw::kinematics::Frame* _tcpFrame;
    rw::kinematics::Frame* _refFrame;

    rw::kinematics::FKRange _baseTref;
    rw::kinematics::FKRange _refTtcp;


    TransformSliderWidget* _transformSliderWidget;
    rw::common::Ptr<rw::invkin::IterativeIK> _iksolver;

    bool _updating;

    void doUpdateValues();
};



#endif //#ifndef DEVICETAB_H
