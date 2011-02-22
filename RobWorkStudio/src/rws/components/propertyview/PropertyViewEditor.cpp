/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "PropertyViewEditor.hpp"

using namespace rw::common;
using namespace rw::math;

PropertyViewEditor::PropertyViewEditor(QWidget *parent): QtTreePropertyBrowser(parent)
{
    _variantFactory = new QtVariantEditorFactory();

    _variantManager = new QtVariantPropertyManager();

    connect(_variantManager, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(slotValueChanged(QtProperty *, const QVariant &)));
    connect(_variantManager, SIGNAL(propertyDestroyed(QtProperty *)),
                this, SLOT(slotPropertyDestroyed(QtProperty *)));


    this->setFactoryForManager(_variantManager, _variantFactory);
    //_variantEditor->addProperty(topItem);

    this->setPropertiesWithoutValueMarked(true);
    this->setRootIsDecorated(false);

}

namespace {

    template<class A>
	void setRWPropValue(PropertyBase::Ptr &rwbase, const A& value){
        Property<A> *rwprop =  dynamic_cast<Property<A>* >( rwbase.get() );
        rwprop->setValue(value);
        rwbase->changedEvent().fire(rwprop);
    }

}

void PropertyViewEditor::slotValueChanged(QtProperty *property, const QVariant &value)
{
    std::string identifier = property->propertyName().toStdString();
    //std::cout << "slotValueChanged: " << identifier << std::endl;
    // now change the value of the property

    if( _qtPropToRwProp.find(property) == _qtPropToRwProp.end() ){
        // its a propertymap that is changed...

    } else {
        // its a property
		PropertyBase::Ptr rwbase = _qtPropToRwProp[property];
        switch( rwbase->getType().getId() ){
        case PropertyType::String:{ setRWPropValue(rwbase, value.toString().toStdString() ); break; }
        case PropertyType::Float:{  setRWPropValue<float>(rwbase,value.toDouble() ); break; }
        case PropertyType::Double:{ setRWPropValue<double>(rwbase,value.toDouble() ); break; }
        case PropertyType::Int:{ setRWPropValue<int>(rwbase,value.toInt() ); break; }
        case PropertyType::Bool:{ setRWPropValue<bool>(rwbase,value.toBool() ); break; }
        case PropertyType::PropertyMap:{
            break;
        }
        default:
            break;
        }


    }


    propertyChanged(identifier);
}

void PropertyViewEditor::slotPropertyDestroyed(QtProperty *property)
{
    //std::cout << "slotPropertyDestroyed" << std::endl;
}


QtProperty* PropertyViewEditor::update(PropertyMap *map, std::string propname){
    std::cout << "X " << propname << std::endl;
    QtProperty *topItem = _variantManager->addProperty(QtVariantPropertyManager::groupTypeId(),
                                                       QLatin1String(propname.c_str()));

    PropertyMap::Range range = map->getProperties();
    for(;range.first!=range.second;++range.first ){
        std::cout << "Y " << std::endl;
        std::string identifier = (*range.first)->getIdentifier();
        std::cout << "X " << identifier << std::endl;
        std::string desc = (*range.first)->getDescription();
        int type_id = (*range.first)->getType().getId();

        QtVariantProperty *item = NULL;
        switch(type_id){
        case PropertyType::String:{
            std::string value = map->get<std::string>(identifier);
            item = _variantManager->addProperty(QVariant::String, QLatin1String(identifier.c_str()));
            item->setValue(value.c_str());
            topItem->addSubProperty(item);
            break;
        }
        case PropertyType::Float:{
            float value = map->get<float>(identifier);
            item = _variantManager->addProperty(QVariant::Double, QLatin1String(identifier.c_str()));
            item->setValue(value);

            item->setAttribute(QLatin1String("singleStep"), 0.1);
            item->setAttribute(QLatin1String("decimals"), 3);
            topItem->addSubProperty(item);
            break;
        }
        case PropertyType::Double:{
            double value = map->get<double>(identifier);
            item = _variantManager->addProperty(QVariant::Double, QLatin1String(identifier.c_str()));
            item->setValue(value);
            item->setAttribute(QLatin1String("singleStep"), 0.1);
            item->setAttribute(QLatin1String("decimals"), 3);
            topItem->addSubProperty(item);
            break;
        }
        case PropertyType::Int:{
            int value = map->get<int>(identifier);
            item = _variantManager->addProperty(QVariant::Int,QLatin1String(identifier.c_str()));
            item->setValue(value);
            item->setAttribute(QLatin1String("minimum"), 0);
            item->setAttribute(QLatin1String("maximum"), 100);
            item->setAttribute(QLatin1String("singleStep"), 10);
            topItem->addSubProperty(item);
            break;
        }
        case PropertyType::Bool:{
            bool value = map->get<bool>(identifier);
            item = _variantManager->addProperty(QVariant::Bool, QLatin1String(identifier.c_str()));
            item->setValue(value);
            topItem->addSubProperty(item);
            break;
        }
        case PropertyType::StringList:{
            std::vector<std::string> value = map->get<std::vector<std::string> >(identifier);
            QStringList list;
            std::cout << "String list" << std::endl;
            BOOST_FOREACH(std::string str, value){ list << QString(str.c_str()); };
            std::cout << "String list" << std::endl;
            item = _variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), QLatin1String(identifier.c_str()));
            std::cout << "String list" << std::endl;
            item->setAttribute("enumNames", list);
            item->setValue(1);
            std::cout << "String list" << std::endl;
            topItem->addSubProperty(item);
            std::cout << "String list" << std::endl;
            break;
        }
        case PropertyType::PropertyMap:{
            PropertyMap *pmap = map->getPtr<PropertyMap>(identifier);
            QtProperty *qprop = update(pmap,identifier);
            _qtPropToRwPropMap[qprop] = pmap;
            topItem->addSubProperty(qprop);
            break;
        }
        default:{
            PropertyBase::Ptr base = _map->findPropertyBase(identifier);
            //if(base!=NULL)
                //std::cout << "The type is: " << typeid(*base).name() << std::endl;
            break;
        }
        }

        if(item!=NULL){
            _qtPropToRwProp[item] = (*range.first);
            _qtPropToRwPropMap[item] = map;
            if(desc!=""){
                item->setToolTip( desc.c_str() );
                item->setWhatsThis( desc.c_str() );
            }
        }
    }
    return topItem;
}

void PropertyViewEditor::update()
{
    std::cout << "8" << std::endl;
    this->clear();
    std::cout << "8" << std::endl;
    _variantManager->clear();
    std::cout << "8" << std::endl;
    if(_map == NULL)
        return;
    std::cout << "9" << std::endl;
    QtProperty *topItem = update(_map.get(), "Root");
    std::cout << "9" << std::endl;
    //_variantEditor->addProperty(topItem);
    std::cout << "9" << std::endl;
    this->addProperty(topItem);
    std::cout << "A" << std::endl;
}

