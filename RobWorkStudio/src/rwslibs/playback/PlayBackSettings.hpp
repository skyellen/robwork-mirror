

#ifndef PLAYBACKSETTINGS_HPP
#define PLAYBACKSETTINGS_HPP

#include <QDialog>

class QDoubleSpinBox;
class QLineEdit;
class QComboBox;
class QShowEvent;

class PlayBackSettings: public QDialog
{
    Q_OBJECT
public:
    PlayBackSettings();
    virtual ~PlayBackSettings();

    double getUpdateRate();

    QString getRecordFilename();

    QString getRecordFileType();

    double getScale(){ return _theScale;}

protected:
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
