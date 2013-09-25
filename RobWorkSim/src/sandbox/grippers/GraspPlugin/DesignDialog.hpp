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
		QLineEdit* _nameEdit;
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
		
		QPushButton* _okButton;
		QPushButton* _cancelButton;
		QPushButton* _applyButton;
		QPushButton* _defaultButton;
		
		std::string _wd;
};
