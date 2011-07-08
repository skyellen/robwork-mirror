#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/rw.hpp>

#include <rwlibs/opengl/RenderFrame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "ui_SamplePlugin.h"

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
    SamplePlugin();
	virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();
    void readUpdateFromUDP();
private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    QTimer *_timer;

    boost::asio::io_service *io_service;
    boost::asio::ip::udp::endpoint* endpoint;
    boost::asio::ip::udp::socket* socket;
    boost::asio::ip::udp::resolver *resolver;

    rwlibs::opengl::RenderFrame::Ptr _renderFrame;
    std::vector<rw::graphics::DrawableNode::Ptr> _drawables;

    std::vector<rw::math::Transform3D<> > _transforms;
    std::vector<std::string> _toFrameName;

    rw::models::Device::Ptr _rightArm, _leftArm;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
