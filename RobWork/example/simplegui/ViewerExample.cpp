#include <SimpleGLViewer.hpp>

#include <EventListener.hpp>
#include <Menu.hpp>
#include <MenuItem.hpp>

#include <rw/loaders/WorkCellLoader.hpp>

#include <iostream>

using rw::common::ownedPtr;

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

    const SimpleGLViewer::Ptr viewer = ownedPtr(new SimpleGLViewer());
    if(argc==2){
        std::string xmlfile(argv[1]);
        std::cout << " | Load Workcell: " << xmlfile << std::endl;
        _workcell = rw::loaders::WorkCellLoader::Factory::load(xmlfile);
        std::cout << " | set workcell" << std::endl;
        viewer->setWorkcell( _workcell );
    }
    std::cout << " | Init" << std::endl;
    viewer->init(argc, argv);
    std::cout << " | Listener" << std::endl;
    viewer->setKeyListener(&listener);
    viewer->addMenu(&testMenu1);
    viewer->addMenu(&testMenu2);
    std::cout << " | Start" << std::endl;
    viewer->start();
    return 1;
}
