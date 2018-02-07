
#ifndef RWS_HELPASSISTANT_HPP
#define RWS_HELPASSISTANT_HPP

#include <QtCore/QString>

QT_BEGIN_NAMESPACE
class QProcess;
QT_END_NAMESPACE

//! @brief Help assistant for RobWorkStudio.
class HelpAssistant
{
public:
	//! @brief Constructor.
    HelpAssistant();

	//! @brief Destructor.
    virtual ~HelpAssistant();

    /**
     * @brief Show the help assistant.
     * @param paths [in] a list of paths to search for the documentation files.
     */
    bool showDocumentation(const QStringList &paths);

    /**
     * @brief Go to the given URL in documentation.
     * @param url [in] the URL.
     */
    void gotoURL(const std::string& url);

    //! @brief Show only the page and hide as much as possible else.
    void minimumView();

private:
    bool startAssistant(QString collectionFileName);
    QProcess *proc;
};

#endif
