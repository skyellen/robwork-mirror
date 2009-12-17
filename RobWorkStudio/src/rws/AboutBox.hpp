#ifndef ABOUTBOX_HPP
#define ABOUTBOX_HPP

#include <QDialog>
#include "ui_AboutBox.h"

class AboutBox : public QDialog
{
    Q_OBJECT

public:
    AboutBox(const QString& version, const QString& revision, QWidget *parent = 0);
    ~AboutBox();
    void addPluginAboutText(const QString& title, const QString& text);
    void addPluginAboutWidget(const QString& title, QWidget* widget);
private:
    Ui::AboutBoxClass ui;

private slots:
    void on_btnOk_clicked();
};

#endif // ABOUTBOX_HPP
 