#include "AboutBox.hpp"
 
using namespace rws;

AboutBox::AboutBox(const QString& version, const QString& revision, QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    QPixmap logo(":/images/rw_logo_64x64.png");
    if(!logo.isNull())
        ui.lblLogo->setPixmap( logo );
    ui.lblVersion->setText(version);
    ui.lblRevision->setText(revision);
    ui.tabWidget->clear();
    ui.grpAddons->setVisible(false);
}

AboutBox::~AboutBox()
{
 
}
 
void AboutBox::addPluginAboutText(const QString& title, const QString& text) {
    QLabel* lbl = new QLabel(text);
    ui.tabWidget->addTab(lbl, title);
    ui.grpAddons->setVisible(true);
} 

void AboutBox::addPluginAboutWidget(const QString& title, QWidget* widget) {
    ui.tabWidget->addTab(widget, title);
    ui.grpAddons->setVisible(true); 
} 
 
void AboutBox::on_btnOk_clicked() 
{
    accept();
}
