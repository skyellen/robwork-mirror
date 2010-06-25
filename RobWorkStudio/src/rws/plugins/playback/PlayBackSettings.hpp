

#ifndef PLAYBACKSETTINGS_HPP
#define PLAYBACKSETTINGS_HPP

#include <QWidget>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QShowEvent>

class PlayBackSettings: public QDialog
{
    Q_OBJECT
public:
    PlayBackSettings();
    virtual ~PlayBackSettings();

    double getUpdateRate();

    QString getRecordFilename();

    QString getRecordFileType();
protected:
    void showEvent(QShowEvent* event);

private slots:
    void browse();
    void ok();
    void cancel();

private:
    double _updateRate;
    QString _recordFilename;
    QString _recordType;

    QDoubleSpinBox* _spnUpdateRate;
    QLineEdit* _edtFilename;
    QComboBox* _cmbType;
};

#endif //end include guard
