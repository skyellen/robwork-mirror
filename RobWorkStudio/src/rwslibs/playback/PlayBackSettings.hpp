

#ifndef PLAYBACKSETTINGS_HPP
#define PLAYBACKSETTINGS_HPP

#include <QDialog>

class QDoubleSpinBox;
class QLineEdit;
class QComboBox;
class QShowEvent;

//! @brief Dialog for changing the settings of playback and recordings.
class PlayBackSettings: public QDialog
{
    Q_OBJECT
public:
	//! @brief Constructor.
    PlayBackSettings();

    //! @brief Destructor.
    virtual ~PlayBackSettings();

    /**
     * @brief Get the update rate.
     * @return the update rate.
     */
    double getUpdateRate();

    /**
     * @brief Get the filename to record images to.
     * @return the filename.
     */
    QString getRecordFilename();

    /**
     * @brief Get the filetype of images to record.
     * @return the filetype.
     */
    QString getRecordFileType();

    /**
     * @brief The the amount to scale the speed of playback.
     * @return the scale.
     */
    double getScale(){ return _theScale;}

protected:
    /**
     * @brief Setup the dialog when it is shown.
     * @param event [in] the event (not used).
     */
    void showEvent(QShowEvent* event);

private slots:
    void browse();
    void ok();
    void scale();
    void cancel();

private:
    double _updateRate, _theScale;
    QString _recordFilename;
    QString _recordType;

    QDoubleSpinBox* _spnUpdateRate;
    QDoubleSpinBox* _spnScale;
    QLineEdit* _edtFilename;
    QComboBox* _cmbType;
};

#endif //end include guard
