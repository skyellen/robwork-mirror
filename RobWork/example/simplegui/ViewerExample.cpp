#include <SimpleGLViewer.h>

#include <EventListener.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <iostream>

class MyListener: public EventListener {
    public:
        MyListener(){};
        virtual ~MyListener(){};

        void event(EventListener::EventType type, void *data){
            std::cout << "EventType: " << type << " ";
            switch(type){
                case(EventListener::e_MENUITEM_EVENT):
                    std::cout << "MENUITEM_EVENT" << std::endl;
                    break;
                case(EventListener::e_SPECIAL_KEY_EVENT):
                    std::cout << "SPECIAL_KEY_EVENT: " << (*((unsigned char*)data)) << std::endl;
                    break;
                case(EventListener::e_KEY_EVENT):
                    std::cout << "KEY_EVENT: " << (*((unsigned char*)data)) << std::endl;
                    break;
                default:
                    std::cout << "default" << std::endl;
            }
        };
};

int main(int argc, char** argv){
    MyListener listener;
    rw::models::WorkCell::Ptr _workcell;
    Menu testMenu1("test1");
    testMenu1.addMenuItem(new MenuItem("Testname a",1,&listener));
    testMenu1.addMenuItem(new MenuItem("Testname c",2,&listener));
    testMenu1.addMenuItem(new MenuItem("Testname d",3,&listener));
    Menu testMenu2("test2");
    testMenu2.addMenuItem(new MenuItem("Testname a",4,&listener));
    testMenu2.addMenuItem(new MenuItem("Testname c",5,&listener));
    testMenu2.addMenuItem(new MenuItem("Testname d",6,&listener));

    if(argc==2){
        std::string xmlfile(argv[1]);
        std::cout << " | Load Workcell: " << xmlfile << std::endl;
        _workcell = rw::loaders::WorkCellLoader::load(xmlfile);
        std::cout << " | set workcell" << std::endl;
        SimpleGLViewer::setWorkcellModel( _workcell );
    }
    std::cout << " | Init" << std::endl;
    SimpleGLViewer::init(argc, argv);
    std::cout << " | Listener" << std::endl;
    SimpleGLViewer::setKeyListener(&listener);
    SimpleGLViewer::addMenu(&testMenu1);
    SimpleGLViewer::addMenu(&testMenu2);
    std::cout << " | Start" << std::endl;
    SimpleGLViewer::start();
    return 1;
}
