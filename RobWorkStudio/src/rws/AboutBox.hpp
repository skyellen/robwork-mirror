#ifndef RWS_ABOUTBOX_HPP
#define RWS_ABOUTBOX_HPP

#include <QDialog>
#include "ui_AboutBox.h"

namespace rws {

/**
 * @brief AboutBox in RobWorkStudio.
 *
 * Displays the RobWork/RobWorkStudio Copyright and license information.
 * Provides a place for Plugins/Addons to visualize their "about" information.
 */
class AboutBox : public QDialog
{
    Q_OBJECT
public:
    /**
     * @brief Constructs about box displaying \b version and \b revision    
     */ 
    AboutBox(const QString& version, const QString& revision, QWidget *parent = 0);

    /**
     * @brief Destructor
     */
    ~AboutBox();

    /**
     * @brief Adds about text for a plugin.
     * @param name [in] Name of the plugin
     * @param text [in] The text to be displayed
     */
    void addPluginAboutText(const QString& title, const QString& text);

    /**
     * @brief Adds widget with about box information for a plugin.
     * @param name [in] Name of the plugin
     * @param widget [in] The widget to display
     */
    void addPluginAboutWidget(const QString& title, QWidget* widget);
private:
    Ui::AboutBoxClass ui;

private slots:
    void on_btnOk_clicked();
};

}

#endif // ABOUTBOX_HPP
 
