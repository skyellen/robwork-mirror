#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

class SamplePlugin: public RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( RobWorkStudioPlugin )
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
