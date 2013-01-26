/*
 * CompositeJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_COMPOSITEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_COMPOSITEJACOBIAN_HPP_

#include "Jacobian.hpp"

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief CompositeJacobian combines several calibrations.
 *
 * T can be Calibration or subclasses thereof.
 */
template<class T>
class CompositeJacobian: public Jacobian {
public:
	typedef rw::common::Ptr<CompositeJacobian> Ptr;

	/**
	* @brief Constructor.
	*/
	CompositeJacobian();

	/**
	* @brief Destructor.
	*/
	virtual ~CompositeJacobian();

	/**
	* @brief Returns a reference to a vector with pointers to the Jacobian(s) in the CompositeJacobian.
	* @return std::vector with pointers to Jacobian(s)
	*/
	const std::vector<rw::common::Ptr<T> >& getJacobians() const;

	void addJacobian(rw::common::Ptr<T> jacobian);
	
	/**
	 * @copydoc Jacobian::getColumnCount()
	 */
	virtual int getColumnCount() const;

	virtual Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state);

	virtual void takeStep(const Eigen::VectorXd& step);

private:
	std::vector<rw::common::Ptr<T> > _jacobians;
};

template<class T>
CompositeJacobian<T>::CompositeJacobian() {

}

template<class T>
CompositeJacobian<T>::~CompositeJacobian() {

}

template<class T>
const std::vector<rw::common::Ptr<T> >& CompositeJacobian<T>::getJacobians() const {
	return _jacobians;
}

template<class T>
void CompositeJacobian<T>::addJacobian(rw::common::Ptr<T> jacobian) {
	_jacobians.push_back(jacobian);
}

template<class T>
int CompositeJacobian<T>::getColumnCount() const {
	int columnCount = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		const rw::common::Ptr<T> jacobian = (*it);
		columnCount += jacobian->getColumnCount();
	}
	return columnCount;
}

template<class T>
Eigen::MatrixXd CompositeJacobian<T>::computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
	RW_ASSERT(getColumnCount() != 0);

	const int columnCount = getColumnCount();
	Eigen::MatrixXd jacobianMatrix(6, columnCount);
	int columnIndex = 0;
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		rw::common::Ptr<T> jacobian = (*it);
		const int columnCount = jacobian->getColumnCount();
		if (columnCount > 0) {
			jacobianMatrix.block(0, columnIndex, 6, columnCount) = jacobian->computeJacobian(referenceFrame, targetFrame, state);
			columnIndex += columnCount;
		}
	}

	return jacobianMatrix;
}

template<class T>
void CompositeJacobian<T>::takeStep(const Eigen::VectorXd& step) {
	RW_ASSERT(step.rows() != 0);
	RW_ASSERT(step.rows() == getColumnCount());

	// HACK: Fix this.
	int columnIndex = 0;
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		rw::common::Ptr<T> jacobian = (*it);
		const int columnCount = jacobian->getColumnCount();
		if (columnCount > 0) {
			jacobian->takeStep(step.segment(columnIndex, columnCount));
			columnIndex += columnCount;
		}
	}
}

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_COMPOSITEJACOBIAN_HPP_ */
