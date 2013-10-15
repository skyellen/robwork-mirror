/**
 * @file TaskDialog.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <QDialog>
#include "TaskDescription.hpp"

class QLineEdit;
class QPushButton;
class QRadioButton;
class QComboBox;



/**
 * @class TaskDialog
 * @brief Dialog for editing TaskDescription in gripper evaluation plugin..
 */
class TaskDialog : public QDialog
{
	Q_OBJECT
	
	public:
	// constructors
		/// Constructor
		TaskDialog(QWidget* parent=0, TaskDescription::Ptr td=0, std::string wd="");
		
		/// Destructor
		virtual ~TaskDialog() {}
		
	// methods
		
		
	private slots:
		void guiEvent();
		void guiEvent(int index);
	
	private:
	// methods
		void createGUI();
		void updateGUI();
		void updateTaskDescription();
		
	// data
		TaskDescription::Ptr _td;
		bool _changed;
		
		// GUI
		QPushButton* _okButton;
		QPushButton* _applyButton;
		QPushButton* _cancelButton;
		
		QComboBox* _targetCombo;
		QComboBox* _gripperCombo;
		QComboBox* _gripperTCPCombo;
		QComboBox* _gripperMovableCombo;
		QComboBox* _controllerCombo;
		
		QLineEdit* _baseShapeEdit;
		QLineEdit* _baseCoverageEdit;
		QLineEdit* _baseSuccessEdit;
		QLineEdit* _baseWrenchEdit;
		QLineEdit* _weightShapeEdit;
		QLineEdit* _weightCoverageEdit;
		QLineEdit* _weightSuccessEdit;
		QLineEdit* _weightWrenchEdit;
		
		QLineEdit* _intLimitEdit;
		QLineEdit* _wreLimitEdit;
		
		QLineEdit* _teachDistEdit;
		QLineEdit* _coverageDistEdit;
		
		std::string _wd;
};

