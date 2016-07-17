#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include <rws/RobWorkStudioPlugin.hpp>

class QPushButton;

class SamplePlugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
    SamplePlugin();
	virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void clickEvent();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    QPushButton* _btn0,*_btn1;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
