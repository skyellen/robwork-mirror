/**
 * @file DesignDialog.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <QDialog>
#include "Gripper.hpp"
#include "ui_DesignDialog.h"

class QLineEdit;
class QPushButton;
class QRadioButton;



/**
 * @class DesignDialog
 * @brief Dialog for gripper design in the plugin.
 */
class DesignDialog : public QDialog
{
	Q_OBJECT
	
	public:
	// constructors
		/// Constructor
		DesignDialog(QWidget* parent=0, rw::models::Gripper::Ptr gripper=0, std::string wd="");
		
		/// Destructor
		virtual ~DesignDialog() {}
		
	// methods
		rw::models::Gripper::Ptr getGripper() { return _gripper; }
		std::string getWorkingDirectory() { return _wd; }
		bool isChanged() const { return _changed; }
		
	private slots:
		void guiEvent();
	
	private:
	// methods
		void _createGUI();
		void _updateGUI();
		void _updateGripper();
		
	// data
		rw::models::Gripper::Ptr _gripper;
		bool _changed;
		
		// GUI
		Ui::designDialog ui;
		
		/*QLineEdit* _nameEdit;
		QLineEdit* _lengthEdit;
		QLineEdit* _widthEdit;
		QLineEdit* _depthEdit;
		QLineEdit* _chfDepthEdit;
		QLineEdit* _chfAngleEdit;
		QLineEdit* _cutDepthEdit;
		QLineEdit* _cutAngleEdit;
		QLineEdit* _cutRadiusEdit;
		QLineEdit* _tcpPosEdit;
		QLineEdit* _forceEdit;
		QLineEdit* _jawdistEdit;
		QLineEdit* _openingEdit;
		QRadioButton* _prismaticButton;
		QRadioButton* _cylindricalButton;
		QLineEdit* _basedxEdit;
		QLineEdit* _basedyEdit;
		QLineEdit* _basedzEdit;
		QLineEdit* _experimentsEdit;
		QLineEdit* _successesEdit;
		QLineEdit* _samplesEdit;
		QLineEdit* _shapeEdit;
		QLineEdit* _coverageEdit;
		QLineEdit* _successEdit;
		QLineEdit* _wrenchEdit;
		QLineEdit* _topwrenchEdit;
		QLineEdit* _qualityEdit;
		
		QPushButton* _okButton;
		QPushButton* _cancelButton;
		QPushButton* _applyButton;
		QPushButton* _defaultButton;*/
		
		std::string _wd;
};
