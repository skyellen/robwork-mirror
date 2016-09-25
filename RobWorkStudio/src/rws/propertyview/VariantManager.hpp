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


#ifndef VARIANTMANAGER_HPP_
#define VARIANTMANAGER_HPP_

#include <QObject>
#include "qtvariantproperty.h"

/**
 * @brief Specialization of QtVariantPropertyManager - not currently in implemented/in use.
 * @deprecated This class is marked for removal.
 */
class VariantManager : public QtVariantPropertyManager
{
    Q_OBJECT
public:
	/**
	 * @brief Constructor.
	 * @param parent [in] owner of this widget.
	 * @deprecated This class is marked for removal.
	 */
    VariantManager(QObject *parent = 0);

    //! @brief Destructor.
    ~VariantManager();

    /**
     * @brief Not documented.
     * @deprecated This class is marked for removal.
     */
    virtual QVariant value(const QtProperty *property) const;

    /**
     * @brief Not documented.
     * @deprecated This class is marked for removal.
     */
    virtual int valueType(int propertyType) const;

    /**
     * @brief Not documented.
     * @deprecated This class is marked for removal.
     */
    virtual bool isPropertyTypeSupported(int propertyType) const;

    /**
     * @brief Not documented.
     * @deprecated This class is marked for removal.
     */
    QString valueText(const QtProperty *property) const;

public slots:
	/**
	 * @brief Not documented.
	 * @deprecated This class is marked for removal.
	 */
    virtual void setValue(QtProperty *property, const QVariant &val);

protected:
	/**
	 * @brief Not documented.
	 * @deprecated This class is marked for removal.
	 */
    virtual void initializeProperty(QtProperty *property);

    /**
     * @brief Not documented.
     * @deprecated This class is marked for removal.
     */
    virtual void uninitializeProperty(QtProperty *property);

private slots:
    void slotValueChanged(QtProperty *property, const QVariant &value);
    void slotPropertyDestroyed(QtProperty *property);

private:
    struct Data {
        QVariant value;
        QtVariantProperty *x;
        QtVariantProperty *y;
    };
    QMap<const QtProperty *, Data> propertyToData;
    QMap<const QtProperty *, QtProperty *> xToProperty;
    QMap<const QtProperty *, QtProperty *> yToProperty;
};


#endif /* VARIANTMANAGER_HPP_ */
