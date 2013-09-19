/**
 * @file DesignDialog.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <QDialog>
#include "Gripper.hpp"

class QLineEdit;
class QPushButton;
class QRadioButton;



/**
 * @class DesignDialog
 * @brief Dialog for gripper design
 */
class DesignDialog : public QDialog
{
	Q_OBJECT
	
	public:
		/**
		 * @brief Constructor
		 */
		DesignDialog(QWidget* parent=0, rw::models::Gripper::Ptr gripper=0, std::string wd="");
		
		//! @brief Destructor
		virtual ~DesignDialog() {}
		
		//! @brief Get gripper
		rw::models::Gripper::Ptr getGripper() { return _gripper; }
		
		//! @brief Get working directory
		std::string getWorkingDirectory() { return _wd; }
		
	private slots:
		void guiEvent();
	
	private:
		// methods
		void _createGUI();
		void _updateGUI();
		void _updateGripper();
		
		rw::models::Gripper::Ptr _gripper;
		
		// GUI
		QLineEdit* _nameEdit;
		QLineEdit* _lengthEdit;
		QLineEdit* _widthEdit;
		QLineEdit* _depthEdit;
		QLineEdit* _chfDepthEdit;
		QLineEdit* _chfAngleEdit;
		QLineEdit* _cutDepthEdit;
		QLineEdit* _cutAngleEdit;
		QLineEdit* _tcpPosEdit;
		QLineEdit* _forceEdit;
		QLineEdit* _jawdistEdit;
		QLineEdit* _openingEdit;
		QRadioButton* _prismaticButton;
		QRadioButton* _cylindricalButton;
		
		QPushButton* _okButton;
		QPushButton* _cancelButton;
		QPushButton* _applyButton;
		QPushButton* _loadButton;
		QPushButton* _saveButton;
		QPushButton* _defaultButton;
		
		std::string _wd;
};
