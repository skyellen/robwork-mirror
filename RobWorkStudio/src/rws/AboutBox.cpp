#include "AboutBox.hpp"
#include <RobWorkConfig.hpp>
//#include <RobWorkStudioConfig.hpp>

#include "ui_AboutBox.h"

using namespace rws;

AboutBox::AboutBox(const QString& version, const QString& revision, QWidget *parent)
    : QDialog(parent)
{
    ui = new Ui_AboutBoxClass();
    ui->setupUi(this);
    QPixmap logo(":/images/rw_logo_64x64.png");
    if(!logo.isNull())
        ui->lblLogo->setPixmap( logo );
    ui->lblVersion->setText(version);
    ui->lblRevision->setText(RW_REVISION);
//    ui->lblRWSRevision->setText(RWS_REVISION);
    ui->tabWidget->clear();
    ui->grpAddons->setVisible(false);
}

AboutBox::~AboutBox()
{
 
}
 
void AboutBox::addPluginAboutText(const QString& title, const QString& text) {
    QLabel* lbl = new QLabel(text);
    ui->tabWidget->addTab(lbl, title);
    ui->grpAddons->setVisible(true);
} 

void AboutBox::addPluginAboutWidget(const QString& title, QWidget* widget) {
    ui->tabWidget->addTab(widget, title);
    ui->grpAddons->setVisible(true);
} 
 
void AboutBox::on_btnOk_clicked() 
{
    accept();
}
