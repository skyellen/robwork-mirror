/*
 * CompositeCalibration.hpp
 *
 *  Created on: Sep 13, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_

#include "Calibration.hpp"

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief CompositeCalibration combines several calibrations.
 *
 * C can be Calibration or subclasses thereof.
 */
template<class C>
class CompositeCalibration: public Calibration {
public:
	typedef rw::common::Ptr<CompositeCalibration> Ptr;

	/**
	* @brief Constructor.
	*/
	CompositeCalibration();

	/**
	* @brief Constructor.
	*/
	CompositeCalibration(const std::vector<rw::common::Ptr<C> >& calibrations);

	/**
	* @brief Destructor.
	*/
	virtual ~CompositeCalibration();
	
	/**
	 * @return Number of calibrations.
	 */
	int getCalibrationCount() const;

	const rw::common::Ptr<C>& getCalibration(int index) const;

	void addCalibration(rw::common::Ptr<C> calibration);
	
	virtual bool isEnabled() const;

	virtual void setEnabled(bool isEnabled);

	virtual CalibrationParameterSet getParameterSet() const;

	virtual void setParameterSet(const CalibrationParameterSet& parameterSet);
	
	/**
	 * @copydoc Calibration::isApplied()
	 */
	virtual bool isApplied() const;
	
	/**
	 * @copydoc Calibration::apply()
	 */
	virtual void apply();
	
	/**
	 * @copydoc Calibration::revert()
	 */
	virtual void revert();

private:
	rw::common::Ptr<C> findParameter(int parameterIndex, int& localParameterIndex) const;

private:
	std::vector<rw::common::Ptr<C> > _calibrations;
};

template<class C>
CompositeCalibration<C>::CompositeCalibration() {

}

template<class C>
CompositeCalibration<C>::~CompositeCalibration() {

}

template<class C>
const rw::common::Ptr<C>& CompositeCalibration<C>::getCalibration(int index) const {
	return _calibrations[index];
}

template<class C>
void CompositeCalibration<C>::addCalibration(rw::common::Ptr<C> calibration) {
	RW_ASSERT(!calibration->isApplied());
	_calibrations.push_back(calibration);
}

template<class C>
int CompositeCalibration<C>::getCalibrationCount() const {
	return (int)_calibrations.size();
}

template<class C>
bool CompositeCalibration<C>::isEnabled() const {
	for (typename std::vector<rw::common::Ptr<C> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<C> calibration = (*it);
		if (calibration->isEnabled())
			return true;
	}

	return false;
}

template<class C>
void CompositeCalibration<C>::setEnabled(bool isEnabled) {
	for (typename std::vector<rw::common::Ptr<C> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<C> calibration = (*it);
		calibration->setEnabled(isEnabled);
	}
}

template<class C>
CalibrationParameterSet CompositeCalibration<C>::getParameterSet() const {
	int parameterCount = 0;
	for (typename std::vector<rw::common::Ptr<C> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<C> calibration = (*it);
		if (calibration->isEnabled())
			parameterCount += calibration->getParameterSet().getCount();
	}

	CalibrationParameterSet parameterSet(parameterCount);

	int parameterIndex = 0;
	for (typename std::vector<rw::common::Ptr<C> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<C> calibration = (*it);
		
		if (calibration->isEnabled()) {
			const CalibrationParameterSet currentParameterSet = calibration->getParameterSet();
			for (int currentParameterIndex = 0; currentParameterIndex < currentParameterSet.getCount(); currentParameterIndex++, parameterIndex++)
				parameterSet(parameterIndex) = currentParameterSet(currentParameterIndex);
		}
	}

	return parameterSet;
}

template<class C>
void CompositeCalibration<C>::setParameterSet(const CalibrationParameterSet& parameterSet) {
	int parameterIndex = 0;
	for (typename std::vector<rw::common::Ptr<C> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<C> calibration = (*it);
		if (calibration->isEnabled()) {
			CalibrationParameterSet currentParameterSet = calibration->getParameterSet();
			for (int currentParameterIndex = 0; currentParameterIndex < currentParameterSet.getCount(); currentParameterIndex++, parameterIndex++)
				currentParameterSet(currentParameterIndex) = parameterSet(parameterIndex);
			calibration->setParameterSet(currentParameterSet);
		}
	}
}

template<class C>
bool CompositeCalibration<C>::isApplied() const {
	for (typename std::vector<rw::common::Ptr<C> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<C> calibration = (*it);
		if (calibration->isApplied())
			return true;
	}

	return false;
}

template<class C>
void CompositeCalibration<C>::apply() {
	if (isEnabled() == false) {
		return;
		//RW_THROW("Tries to apply a calibration which is not enabled");
	}
	if (isApplied())
		return;
	

	for (typename std::vector<rw::common::Ptr<C> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<C> calibration = (*it);
		if (calibration->isEnabled())
			calibration->apply();
	}

	RW_ASSERT(isApplied());
}

template<class C>
void CompositeCalibration<C>::revert() {
	if (isEnabled() == false) {
		return;
//		RW_THROW("Tries to revert a calibration which is not enabled");
	}

	if (isApplied() == false) {
		return;
	//	RW_THROW("Tries to revert and calibration which is not applied");
	}

	for (typename std::vector<rw::common::Ptr<C> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<C> calibration = (*it);
		if (calibration->isApplied())
			calibration->revert();
	}

	//RW_ASSERT(!isApplied());
}

template<class C>
rw::common::Ptr<C> CompositeCalibration<C>::findParameter(int parameterIndex, int& localParameterIndex) const {

	rw::common::Ptr<C> calibration;
	int minParameterIndex = 0, maxParameterIndex = 0;
	for (typename std::vector<rw::common::Ptr<C> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		calibration = (*it);
		const int parameterCount = calibration->getParameterCount();
		maxParameterIndex += parameterCount;
		if (parameterIndex >= minParameterIndex && parameterIndex < maxParameterIndex)
			break;
		minParameterIndex = maxParameterIndex;
	}

	RW_ASSERT(minParameterIndex != maxParameterIndex);

	localParameterIndex = parameterIndex - minParameterIndex;

	RW_ASSERT(localParameterIndex >= 0);

	return calibration;
}

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_ */
