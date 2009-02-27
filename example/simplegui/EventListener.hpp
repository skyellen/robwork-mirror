#ifndef EVENTLISTENER_HPP_
#define EVENTLISTENER_HPP_

class EventListener {
public:
   /* enum EventType{MENUITEM_EVENT,MENU_EVENT,SPECIAL_KEY_EVENT,KEY_EVENT};
*/
    enum EventType{e_MENUITEM_EVENT,e_SPECIAL_KEY_EVENT,e_KEY_EVENT};

    virtual ~EventListener(){};

    virtual void event(EventType type, void *data) = 0;

};

#endif /*EVENTLISTENER_HPP_*/
