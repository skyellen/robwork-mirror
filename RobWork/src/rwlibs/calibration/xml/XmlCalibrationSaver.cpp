/*
 * XmlCalibrationSaver.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: bing
 */

#include "XmlCalibrationSaver.hpp"

#include <QtCore>
#include <QtXml/qdom.h>

namespace rwlibs {
namespace calibration {

class ElementCreator {
public:
	ElementCreator(QDomDocument* document) :
			_document(document) {

	}

	template<class T>
	QDomElement createElement(T object);

private:
	QDomDocument* _document;
};

template<>
QDomElement ElementCreator::createElement<FixedFrameCalibration::Ptr>(FixedFrameCalibration::Ptr calibration) {
	QDomElement element = _document->createElement("FixedFrameCalibration");

	element.setAttribute("frame", QString::fromStdString(calibration->getFrame()->getName()));

	QDomElement transformElement = _document->createElement("Transform");

	transformElement.setAttribute("isPostCorrection", QString::number(calibration->isPostCorrection()));

	QString transformString;
	rw::math::Transform3D<> correction = calibration->getCorrectionTransform();
	for (int rowIndex = 0; rowIndex < 3; rowIndex++)
		for (int colIndex = 0; colIndex < 4; colIndex++)
			transformString.append(QString(" %1").arg(correction(rowIndex, colIndex), 0, 'g', 16));
	transformElement.appendChild(_document->createTextNode(transformString.trimmed()));
	element.appendChild(transformElement);

	return element;
}

template<>
QDomElement ElementCreator::createElement<DHLinkCalibration::Ptr>(DHLinkCalibration::Ptr calibration) {
	QDomElement element = _document->createElement("DHLinkCalibration");

	element.setAttribute("joint", QString::fromStdString(calibration->getJoint()->getName()));

	const CalibrationParameterSet parameterSet = calibration->getParameterSet();
	if (parameterSet(DHLinkCalibration::PARAMETER_A).isEnabled())
		element.setAttribute("a", QString("%1").arg(parameterSet(DHLinkCalibration::PARAMETER_A), 0, 'g', 16));
	if (parameterSet(DHLinkCalibration::PARAMETER_B).isEnabled())
		element.setAttribute("b", QString("%1").arg(parameterSet(DHLinkCalibration::PARAMETER_B), 0, 'g', 16));
	if (parameterSet(DHLinkCalibration::PARAMETER_D).isEnabled())
		element.setAttribute("d", QString("%1").arg(parameterSet(DHLinkCalibration::PARAMETER_D), 0, 'g', 16));
	if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).isEnabled())
		element.setAttribute("alpha", QString("%1").arg(parameterSet(DHLinkCalibration::PARAMETER_ALPHA), 0, 'g', 16));
	if (parameterSet(DHLinkCalibration::PARAMETER_BETA).isEnabled())
		element.setAttribute("beta", QString("%1").arg(parameterSet(DHLinkCalibration::PARAMETER_BETA), 0, 'g', 16));
	if (parameterSet(DHLinkCalibration::PARAMETER_THETA).isEnabled())
		element.setAttribute("theta", QString("%1").arg(parameterSet(DHLinkCalibration::PARAMETER_THETA), 0, 'g', 16));

	return element;
}

QDomDocument createDOMDocument(SerialDeviceCalibration::Ptr calibration) {
	QDomDocument document("SerialDeviceCalibration");

	QDomElement rootElement = document.createElement("SerialDeviceCalibration");

	ElementCreator creator(&document);

	if (!calibration->getBaseCalibration().isNull()) {
		QDomElement baseElement = document.createElement("BaseCalibration");
		baseElement.appendChild(creator.createElement<FixedFrameCalibration::Ptr>(calibration->getBaseCalibration()));
		rootElement.appendChild(baseElement);
	}

	if (!calibration->getEndCalibration().isNull()) {
		QDomElement endElement = document.createElement("EndCalibration");
		endElement.appendChild(creator.createElement<FixedFrameCalibration::Ptr>(calibration->getEndCalibration()));
		rootElement.appendChild(endElement);
	}

	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	const int linkCalibrationCount = compositeLinkCalibration->getCalibrationCount();
	if (linkCalibrationCount > 0) {
		QDomElement linkCalibrationElement = document.createElement("LinkCalibrations");
		for (int calibrationIndex = 0; calibrationIndex < linkCalibrationCount; calibrationIndex++) {
			DHLinkCalibration::Ptr linkCalibration = compositeLinkCalibration->getCalibration(calibrationIndex);
			linkCalibrationElement.appendChild(creator.createElement<DHLinkCalibration::Ptr>(linkCalibration));
		}
		rootElement.appendChild(linkCalibrationElement);
	}

	document.appendChild(rootElement);

	return document;
}

void XmlCalibrationSaver::save(SerialDeviceCalibration::Ptr calibration, std::string fileName) {
	QFile file(QString::fromStdString(fileName));
	file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate);

	QTextStream textStream(&file);
	textStream.setRealNumberPrecision(16);
	textStream << createDOMDocument(calibration).toString();

	file.close();
}

void XmlCalibrationSaver::save(SerialDeviceCalibration::Ptr calibration, std::ostream& ostream) {
	ostream << createDOMDocument(calibration).toString().toStdString();
}

}
}
