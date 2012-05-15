
#ifndef RWS_HELPASSISTANT_HPP
#define RWS_HELPASSISTANT_HPP

#include <QtCore/QString>

QT_BEGIN_NAMESPACE
class QProcess;
QT_END_NAMESPACE


class HelpAssistant
{
public:
    HelpAssistant();
    virtual ~HelpAssistant();

    void showDocumentation(const QStringList &paths);
    
private:
    bool startAssistant(QString collectionFileName);
    QProcess *proc;
};

#endif
