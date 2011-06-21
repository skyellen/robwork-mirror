#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

class SamplePlugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
    SamplePlugin();
	virtual ~SamplePlugin();

	// functions inherited from RobworkStudioPlugin, are typically used but can be optional
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    void clickEvent();

    void stateChangedListener(const rw::kinematics::State& state);
private:
    QPushButton* _btn0,*_btn1;
};

#endif /*SAMPLEPLUGIN_HPP*/
