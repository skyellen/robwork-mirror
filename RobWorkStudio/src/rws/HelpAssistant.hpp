
#ifndef RWS_HELPASSISTANT_HPP
#define RWS_HELPASSISTANT_HPP

#include <QtCore/QString>

QT_BEGIN_NAMESPACE
class QProcess;
QT_END_NAMESPACE


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
    void showDocumentation(const QStringList &paths);
    
private:
    bool startAssistant(QString collectionFileName);
    QProcess *proc;
};

#endif
