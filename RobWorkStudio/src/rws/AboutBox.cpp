#include "AboutBox.hpp"

AboutBox::AboutBox(const QString& version, const QString& revision, QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    ui.lblLogo->setPixmap(QPixmap(":/images/rw_logo_64x64.png"));
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