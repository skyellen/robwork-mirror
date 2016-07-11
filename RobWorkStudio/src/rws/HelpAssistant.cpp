
// this stuff is very much borrowed from the Qt example homepage

#include <QtCore/QByteArray>
#include <QtCore/QDir>
#include <QtCore/QLibraryInfo>
#include <QtCore/QProcess>

#include <QMessageBox>

#include "HelpAssistant.hpp"
#include "../RobWorkStudioConfig.hpp"

HelpAssistant::HelpAssistant()
    : proc(0)
{
}

HelpAssistant::~HelpAssistant()
{
    if (proc && proc->state() == QProcess::Running) {
        proc->terminate();
        proc->waitForFinished(3000);
    }
    delete proc;
}

void HelpAssistant::showDocumentation(const QStringList& paths)
{
    QStringList files;
    // search for the help file
    QString filename( "robwork_help-v");
    filename.append("RWS_VERSION");
    filename.append(".qhc");
    files.append(filename);
    files.append("docs/"+filename);
    files.append("../docs/"+filename);
    files.append("../../docs/"+filename);

    QString absfilename;
    for(int j=0;j<paths.size();j++){
        QString path = paths[j];

        for(int i=0;i<files.size();i++){
            QString file = path;
            file.append("/");
            file.append(files[i]);
            if( QFile::exists( file ) ){
                absfilename = file;
                break;
            }
            //std::cout << std::endl;
        }
        if(absfilename.size()!=0)
            break;
    }


    if(absfilename.size()==0){
        QMessageBox msgBox;
        msgBox.setText("The RobWork help files could not be located. \nMake sure they are properly installed.");
        msgBox.exec();
        return;
    }

    //std::cout << "start assistant: " << absfilename.toStdString() << std::endl;
    if (!startAssistant(absfilename))
        return;

    QByteArray ba("SetSource ");
    ba.append("qthelp://rws/doc/index.html\n");
    proc->write(ba);
}

bool HelpAssistant::startAssistant(QString collectionFileName)
{
    if (!proc)
        proc = new QProcess();

    if (proc->state() != QProcess::Running) {
        QString app = QLibraryInfo::location(QLibraryInfo::BinariesPath) + QDir::separator();
#if !defined(Q_OS_MAC)
        app += QLatin1String("assistant");
#else
        app += QLatin1String("Assistant.app/Contents/MacOS/Assistant");    
#endif

        QStringList args;
        args << QLatin1String("-collectionFile")
             << QLatin1String(collectionFileName.toStdString().c_str() )
             << QLatin1String("-enableRemoteControl");

        proc->start(app, args);

        if (!proc->waitForStarted()) {
            QMessageBox::critical(0, QObject::tr("RobWork Help"),
                QObject::tr("Unable to launch Qt Assistant (%1)").arg(app));
            return false;
        }    
    }
    return true;
}
