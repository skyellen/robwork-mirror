/*
 * SerialDeviceCalibration.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#include "SerialDeviceCalibration.hpp"

#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

SerialDeviceCalibration::SerialDeviceCalibration(rw::models::SerialDevice::Ptr serialDevice) :
		_serialDevice(serialDevice), _baseCorrection(Eigen::Affine3d::Identity()), _endCorrection(Eigen::Affine3d::Identity()), _baseEnabled(true), _endEnabled(
				true), _dhEnabled(true) {
	_baseFrame = serialDevice->getBase();
	_endFrame = serialDevice->getEnd();

	const std::vector<rw::models::DHParameterSet> dhParameterSets = rw::models::DHParameterSet::getDHParameters(serialDevice);
	const unsigned int nLinks = serialDevice->getDOF() - 1;
	for (unsigned int linkNo = 0; linkNo < nLinks; linkNo++) {
		if (dhParameterSets[linkNo + 1].isParallel())
			_dhCorrections.push_back(rw::models::DHParameterSet(0.0, 0.0, 0.0, 0.0, true));
		else
			_dhCorrections.push_back(rw::models::DHParameterSet(0.0, 0.0, 0.0, 0.0, dhParameterSets[linkNo + 1].getType()));
	}
}

SerialDeviceCalibration::~SerialDeviceCalibration() {

}

void SerialDeviceCalibration::setEnabled(bool baseEnabled, bool endEnabled, bool dhEnabled) {
	_baseEnabled = baseEnabled;
	_endEnabled = endEnabled;
	_dhEnabled = dhEnabled;
}

Eigen::Affine3d SerialDeviceCalibration::getBaseCorrection() const {
	return _baseCorrection;
}

void SerialDeviceCalibration::setBaseCorrection(const Eigen::Affine3d& correction) {
	_baseCorrection = correction;
}

Eigen::Affine3d SerialDeviceCalibration::getEndCorrection() const {
	return _endCorrection;
}

void SerialDeviceCalibration::setEndCorrection(const Eigen::Affine3d& correction) {
	_endCorrection = correction;
}

std::vector<rw::models::DHParameterSet> SerialDeviceCalibration::getDHCorrections() const {
	return _dhCorrections;
}

void SerialDeviceCalibration::setDHCorrections(const std::vector<rw::models::DHParameterSet>& corrections) {
	_dhCorrections = corrections;
}

rw::kinematics::Frame* SerialDeviceCalibration::getBaseFrame() {
	return _baseFrame;
}

void SerialDeviceCalibration::setBaseFrame(rw::kinematics::Frame* baseFrame) {
	_baseFrame = baseFrame;
}

rw::kinematics::Frame* SerialDeviceCalibration::getEndFrame() {
	return _endFrame;
}

void SerialDeviceCalibration::setEndFrame(rw::kinematics::Frame* endFrame) {
	_endFrame = endFrame;
}

void SerialDeviceCalibration::apply() {
	// Apply base correction
	if (_baseEnabled) {
		rw::kinematics::FixedFrame* baseFrame = (rw::kinematics::FixedFrame*) _baseFrame;
		Eigen::Affine3d correctedBaseTransform = baseFrame->getFixedTransform() * _baseCorrection;
		baseFrame->setTransform(correctedBaseTransform);
	}

	// Apply end correction
	if (_endEnabled) {
		rw::kinematics::FixedFrame* endFrame = (rw::kinematics::FixedFrame*) _endFrame;
		Eigen::Affine3d correctedEndTransform = _endCorrection * endFrame->getFixedTransform();
		endFrame->setTransform(correctedEndTransform);
	}

	// Apply DH corrections
	if (_dhEnabled && _dhCorrections.size() > 0) {
		std::vector<rw::models::DHParameterSet> dhParameterSets = rw::models::DHParameterSet::getDHParameters(_serialDevice);
		std::vector<rw::models::Joint*> joints = _serialDevice->getJoints();

		const unsigned int nLinks = dhParameterSets.size() - 1;
		for (unsigned int linkNo = 0; linkNo < nLinks; linkNo++) {
			double alpha = dhParameterSets[linkNo + 1].alpha() + _dhCorrections[linkNo].alpha();
			double a = dhParameterSets[linkNo + 1].a() + _dhCorrections[linkNo].a();
			if (dhParameterSets[linkNo + 1].isParallel()) {
				double beta = dhParameterSets[linkNo + 1].beta() + _dhCorrections[linkNo].beta();
				double b = dhParameterSets[linkNo + 1].b() + _dhCorrections[linkNo].b();
				rw::models::DHParameterSet dhParameterSet(alpha, a, beta, b, true);
				rw::models::DHParameterSet::set(dhParameterSet, joints[linkNo + 1]);
				joints[linkNo + 1]->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
			} else {
				double d = dhParameterSets[linkNo + 1].d() + _dhCorrections[linkNo].d();
				double theta = dhParameterSets[linkNo + 1].theta() + _dhCorrections[linkNo].theta();
				rw::models::DHParameterSet dhParameterSet(alpha, a, d, theta, dhParameterSets[linkNo + 1].getType());
				rw::models::DHParameterSet::set(dhParameterSet, joints[linkNo + 1]);
				joints[linkNo + 1]->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
			}
		}
	}
}

void SerialDeviceCalibration::revert() {
	// Revert base correction
	if (_baseEnabled) {
		rw::kinematics::FixedFrame* baseFrame = (rw::kinematics::FixedFrame*) _baseFrame;
		Eigen::Affine3d correctedBaseTransform = baseFrame->getFixedTransform() * _baseCorrection.inverse();
		baseFrame->setTransform(correctedBaseTransform);
	}

	// Revert end correction
	if (_endEnabled) {
		rw::kinematics::FixedFrame* endFrame = (rw::kinematics::FixedFrame*) _endFrame;
		Eigen::Affine3d correctedEndTransform = _endCorrection.inverse() * endFrame->getFixedTransform();
		endFrame->setTransform(correctedEndTransform);
	}

	// Revert DH corrections
	if (_dhEnabled && _dhCorrections.size() > 0) {
		std::vector<rw::models::DHParameterSet> dhParameterSets = rw::models::DHParameterSet::getDHParameters(_serialDevice);
		std::vector<rw::models::Joint*> joints = _serialDevice->getJoints();

		const unsigned int nLinks = dhParameterSets.size() - 1;
		for (unsigned int linkNo = 0; linkNo < nLinks; linkNo++) {
			double alpha = dhParameterSets[linkNo + 1].alpha() - _dhCorrections[linkNo].alpha();
			double a = dhParameterSets[linkNo + 1].a() - _dhCorrections[linkNo].a();
			if (dhParameterSets[linkNo + 1].isParallel()) {
				double beta = dhParameterSets[linkNo + 1].beta() - _dhCorrections[linkNo].beta();
				double b = dhParameterSets[linkNo + 1].b() - _dhCorrections[linkNo].b();
				rw::models::DHParameterSet dhParameterSet(alpha, a, beta, b, true);
				rw::models::DHParameterSet::set(dhParameterSet, joints[linkNo + 1]);
				joints[linkNo + 1]->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
			} else {
				double d = dhParameterSets[linkNo + 1].d() - _dhCorrections[linkNo].d();
				double theta = dhParameterSets[linkNo + 1].theta() - _dhCorrections[linkNo].theta();
				rw::models::DHParameterSet dhParameterSet(alpha, a, d, theta, dhParameterSets[linkNo + 1].getType());
				rw::models::DHParameterSet::set(dhParameterSet, joints[linkNo + 1]);
				joints[linkNo + 1]->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
			}
		}
	}

}

void SerialDeviceCalibration::save(QString fileName) {
	QFile file(fileName);
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDeviceCalibration");

	QDomElement elmRoot = document.createElement("DeviceCalibration");
	elmRoot.setAttribute("name", QString::fromStdString(_serialDevice->getName()));

	// Save base correction
	if (_baseEnabled) {
		QDomElement baseFrameCorrection = document.createElement("BaseFrameCorrection");
		baseFrameCorrection.setAttribute("name", QString::fromStdString(_baseFrame->getName()));
		QDomElement baseFrameCorrectionTransform = document.createElement("Transform");
		QString baseTransformTxt;
		for (int rowNo = 0; rowNo < 3; rowNo++)
			for (int colNo = 0; colNo < 4; colNo++)
				baseTransformTxt.append(QString(" %1").arg(_baseCorrection(rowNo, colNo), 0, 'g', 16));
		baseFrameCorrectionTransform.appendChild(document.createTextNode(baseTransformTxt.trimmed()));
		baseFrameCorrection.appendChild(baseFrameCorrectionTransform);
		elmRoot.appendChild(baseFrameCorrection);
	}

	// Save end correction
	if (_endEnabled) {
		QDomElement endFrameCorrection = document.createElement("EndFrameCorrection");
		endFrameCorrection.setAttribute("name", QString::fromStdString(_endFrame->getName()));
		QDomElement endFrameCorrectionTransform = document.createElement("Transform");
		QString endTransformTxt;
		for (int rowNo = 0; rowNo < 3; rowNo++)
			for (int colNo = 0; colNo < 4; colNo++)
				endTransformTxt.append(QString(" %1").arg(_endCorrection(rowNo, colNo), 0, 'g', 16));
		endFrameCorrectionTransform.appendChild(document.createTextNode(endTransformTxt.trimmed()));
		endFrameCorrection.appendChild(endFrameCorrectionTransform);
		elmRoot.appendChild(endFrameCorrection);
	}

	// Save dh corrections
	if (_dhEnabled && _dhCorrections.size() > 0) {
		QDomElement dhCorrections = document.createElement("DHJointCorrections");
		std::vector<rw::models::Joint*> joints = _serialDevice->getJoints();
		const unsigned int nLinks = _dhCorrections.size();
		for (unsigned int linkNo = 0; linkNo < nLinks; linkNo++) {
			QDomElement dhCorrection = document.createElement("DHJointCorrection");
			dhCorrection.setAttribute("name", QString::fromStdString(joints[linkNo + 1]->getName()));

			dhCorrection.setAttribute("alpha", QString("%1").arg(_dhCorrections[linkNo].alpha(), 0, 'g', 16));
			dhCorrection.setAttribute("a", QString("%1").arg(_dhCorrections[linkNo].a(), 0, 'g', 16));
			if (_dhCorrections[linkNo].isParallel()) {
				dhCorrection.setAttribute("offset", QString("%1").arg(_dhCorrections[linkNo].beta(), 0, 'g', 16));
				dhCorrection.setAttribute("b", QString("%1").arg(_dhCorrections[linkNo].b(), 0, 'g', 16));
			} else {
				dhCorrection.setAttribute("offset", QString("%1").arg(_dhCorrections[linkNo].theta(), 0, 'g', 16));
				dhCorrection.setAttribute("d", QString("%1").arg(_dhCorrections[linkNo].d(), 0, 'g', 16));
			}

			dhCorrections.appendChild(dhCorrection);
		}
		elmRoot.appendChild(dhCorrections);
	}

	document.appendChild(elmRoot);

	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << document.toString();

	file.close();
}

SerialDeviceCalibration::Ptr SerialDeviceCalibration::load(rw::models::SerialDevice::Ptr serialDevice, const std::string& fileName) {
	SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice));

	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::ReadOnly | QIODevice::Text | QIODevice::Truncate);

	QDomDocument document("SerialDeviceCalibration");

	if (!document.setContent(&file))
		RW_THROW("Content not set.");

	QDomElement elmRoot = document.documentElement();
	if (elmRoot.tagName() != "DeviceCalibration")
		RW_THROW("Root element not found.");

	QString calibratedDeviceName;
	if (elmRoot.hasAttributes()) {
		QDomNamedNodeMap elmRootAttributes = elmRoot.attributes();
		if (elmRootAttributes.contains("name"))
			calibratedDeviceName = elmRoot.attribute("name");
		else
			RW_THROW("Root element has wrong attribute.");
	} else
		RW_THROW("Root element has no attribute.");

	bool baseEnabled = false, endEnabled = false, dhEnabled = false;
	// Load base frame corrections
	QString baseFrameName;
	Eigen::Affine3d baseCorrectionAffine;
	QDomNodeList baseFrameCorrectionNodes = document.elementsByTagName("BaseFrameCorrection");
	if (!baseFrameCorrectionNodes.isEmpty()) {
		baseEnabled = true;
		QDomNode baseFrameCorrectionNode = baseFrameCorrectionNodes.at(0);
		if (baseFrameCorrectionNode.hasAttributes()) {
			QDomNamedNodeMap baseFrameCorrectionAttributes = baseFrameCorrectionNode.attributes();
			if (baseFrameCorrectionAttributes.contains("name"))
				baseFrameName = baseFrameCorrectionNode.toElement().attribute("name");
			else
				RW_THROW( QString("BaseFrameCorrection needs \"name\" attribute.").toStdString());
		} else
			RW_THROW( QString("BaseFrameCorrection needs \"name\" attribute.").toStdString());

		QDomElement baseFrameCorrectionTransform = baseFrameCorrectionNode.namedItem("Transform").toElement();
		QStringList txtTransformSplitted = baseFrameCorrectionTransform.text().trimmed().split(" ");
		if (txtTransformSplitted.count() != 12)
			RW_THROW( QString("BaseFrameCorrection transform has wrong size (12 numbers).").toStdString());
		for (int rowNo = 0; rowNo < 3; rowNo++)
			for (int colNo = 0; colNo < 4; colNo++)
				baseCorrectionAffine(rowNo, colNo) = txtTransformSplitted[4 * rowNo + colNo].toDouble();
		serialDeviceCalibration->setBaseCorrection(baseCorrectionAffine);
	}

	// Load end frame corrections
	QString endFrameName;
	Eigen::Affine3d endCorrectionAffine;
	QDomNodeList endFrameCorrectionNodes = document.elementsByTagName("EndFrameCorrection");
	if (!endFrameCorrectionNodes.isEmpty()) {
		endEnabled = true;
		QDomNode endFrameCorrectionNode = endFrameCorrectionNodes.at(0);
		if (endFrameCorrectionNode.hasAttributes()) {
			QDomNamedNodeMap endFrameCorrectionAttributes = endFrameCorrectionNode.attributes();
			if (endFrameCorrectionAttributes.contains("name"))
				endFrameName = endFrameCorrectionNode.toElement().attribute("name");
			else
				RW_THROW( QString("EndFrameCorrection needs \"name\" attribute.").toStdString());
		} else
			RW_THROW( QString("EndFrameCorrection needs \"name\" attribute.").toStdString());

		QDomElement endFrameCorrectionTransform = endFrameCorrectionNode.namedItem("Transform").toElement();
		QStringList txtTransformSplitted = endFrameCorrectionTransform.text().trimmed().split(" ");
		if (txtTransformSplitted.count() != 12)
			RW_THROW( QString("EndFrameCorrection transform has wrong size (12 numbers).").toStdString());
		for (int rowNo = 0; rowNo < 3; rowNo++)
			for (int colNo = 0; colNo < 4; colNo++)
				endCorrectionAffine(rowNo, colNo) = txtTransformSplitted[4 * rowNo + colNo].toDouble();
		serialDeviceCalibration->setEndCorrection(endCorrectionAffine);
	}

	// Load DH corrections
	std::vector<rw::models::DHParameterSet> dhParameterSetCorrections;
	std::vector<QString> jointNames;
	QDomNodeList dhCorrectionsNodes = document.elementsByTagName("DHJointCorrections");
	if (!dhCorrectionsNodes.isEmpty()) {
		dhEnabled = true;
		QDomNodeList dhCorrectionNodes = dhCorrectionsNodes.at(0).childNodes();
		for (int jointNo = 0; jointNo < dhCorrectionNodes.size(); jointNo++) {
			double alpha = 0.0, a = 0.0, d = 0.0, theta = 0.0;
			bool parallel = false;
			QDomNode jointNode = dhCorrectionNodes.at(jointNo);
			if (jointNode.hasAttributes()) {
				QDomNamedNodeMap dhParameters = jointNode.attributes();

				if (dhParameters.contains("name"))
					jointNames.push_back(jointNode.toElement().attribute("name"));
				else
					RW_THROW("DHJointCorrection needs \"name\" attribute.");

				if (dhParameters.contains("alpha"))
					alpha = jointNode.toElement().attribute("alpha").toDouble();
				else {
					QString name = jointNames.back();
					RW_THROW(QString("DHJointCorrection with name \"%1\" needs \"alpha\" attribute.").arg(name).toStdString());
				}

				if (dhParameters.contains("a"))
					a = jointNode.toElement().attribute("a").toDouble();
				else {
					QString name = jointNames.back();
					RW_THROW(QString("DHJointCorrection with name \"%1\" needs \"a\" attribute.").arg(name).toStdString());
				}

				if (dhParameters.contains("b")) {
					d = jointNode.toElement().attribute("b").toDouble();
					parallel = true;
				} else if (dhParameters.contains("d"))
					d = jointNode.toElement().attribute("d").toDouble();
				else {
					QString name = jointNames.back();
					RW_THROW(QString("DHJointCorrection with name \"%1\" needs \"b\" or \"d\" attribute.").arg(name).toStdString());
				}

				if (dhParameters.contains("offset"))
					theta = jointNode.toElement().attribute("offset").toDouble();
				else {
					QString name = jointNames.back();
					RW_THROW(QString("DHJointCorrection with name \"%1\" needs \"offset\" attribute.").arg(name).toStdString());
				}

				if (parallel)
					dhParameterSetCorrections.push_back(rw::models::DHParameterSet(alpha, a, theta, d, parallel));
				else
					dhParameterSetCorrections.push_back(rw::models::DHParameterSet(alpha, a, d, theta));
			} else
				RW_THROW("DHJointCorrection needs attributes.");
		}
		serialDeviceCalibration->setDHCorrections(dhParameterSetCorrections);
	}

	serialDeviceCalibration->setEnabled(baseEnabled, endEnabled, dhEnabled);

	return serialDeviceCalibration;
}


SerialDeviceCalibration::Ptr SerialDeviceCalibration::getCalibration(rw::models::SerialDevice::Ptr serialDevice) {
	SerialDeviceCalibration::Ptr serialDeviceCalibration = rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice));
}

void SerialDeviceCalibration::setCalibration(SerialDeviceCalibration::Ptr serialDeviceCalibration) {

}

}
}
