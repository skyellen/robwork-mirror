#ifndef MENU_HPP_
#define MENU_HPP_

#include <string>
#include <vector>

class MenuItem;

class Menu
{
public:
    Menu(const std::string& name): _name(name){};
    virtual ~Menu(){};

    std::vector<Menu*> getMenus(){ return _menus;};
    std::vector<MenuItem*> getMenuItems(){ return _items;};

    std::string getName(){return _name;};

    void addMenu(Menu* menu){
        _menus.push_back(menu);
    }

    void addMenuItem(MenuItem* item){
        _items.push_back(item);
    }

private:
    std::vector< Menu* > _menus;
    std::vector< MenuItem* > _items;
    std::string _name;
};

#endif /*MENU_HPP_*/
